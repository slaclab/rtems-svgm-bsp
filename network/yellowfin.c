/* yellowfin.c: A Packet Engines G-NIC ethernet driver for RTEMS. */
/*
	Written 1997-2001 by Donald Becker.

	RTEMS port 2001 by Till Straumann, <strauman@slac.stanford.edu>

	This software may be used and distributed according to the terms of
	the GNU General Public License (GPL), incorporated herein by reference.
	Drivers based on or derived from this code fall under the GPL and must
	retain the authorship, copyright and license notice.  This file is not
	a complete program and may only be used when the entire operating
	system is licensed under the GPL.

	This driver is for the Packet Engines G-NIC PCI Gigabit Ethernet adapter.
	It also supports the Symbios Logic version of the same chip core.

	The author may be reached as becker@scyld.com, or C/O
	Scyld Computing Corporation
	410 Severn Ave., Suite 210
	Annapolis MD 21403

	Support and updates available at
	http://www.scyld.com/network/yellowfin.html
*/

#define YELLOWFIN_TASK_NAME		"yfin"

/* These identify the driver base version and may not be removed. */
static const char version1[] =
"yellowfin.c:v1.07-rtems  8/2/2001  Written by Donald Becker <becker@scyld.com>, 10/16/2001 Till Straumann <strauman@slac.stanford.edu>\n";
static const char version2[] =
"  http://www.scyld.com/network/yellowfin.html\n";

/* The user-configurable values.
   These may be modified when a driver module is loaded.*/

int yellowfin_debug = 1;				/* 1 normal messages, 0 quiet .. 7 verbose. */
/* Maximum events (Rx packets, etc.) to handle at each event reception. */
static int max_interrupt_work = 20;
#ifdef YF_PROTOTYPE						/* Support for prototype hardware errata. */
/* System-wide count of bogus-rx frames. */
static int bogus_rx = 0;
static int dma_ctrl = 0x004A0263; 		/* Constrained by errata */
static int fifo_cfg = 0x0020;			/* Bypass external Tx FIFO. */
#elif YF_NEW							/* A future perfect board :->.  */
static int dma_ctrl = 0x00CAC277;		/* Override when loading module! */
static int fifo_cfg = 0x0028;
#else
static int dma_ctrl = 0x004A0263; 		/* Constrained by errata */
static int fifo_cfg = 0x0020;			/* Bypass external Tx FIFO. */
#endif

/* Set the copy breakpoint for the copy-only-tiny-frames scheme.
   Setting to > 1514 effectively disables this feature. */
static int rx_copybreak = 0;

/* Used to pass the media type, etc.
   No media types are currently defined.  These exist for driver
   interoperability.
*/
#define MAX_UNITS 8				/* More are supported, limit only on options */
static int options[MAX_UNITS] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int full_duplex[MAX_UNITS] = {-1, -1, -1, -1, -1, -1, -1, -1};

/* Do ugly workaround for GX server chipset errata. */
static int gx_fix = 0;

/* Operational parameters that are set at compile time. */

/* Keep the ring sizes a power of two for efficiency.
   Making the Tx ring too long decreases the effectiveness of channel
   bonding and packet priority.
   There are no ill effects from too-large receive rings. */
#define TX_RING_SIZE	16
#define TX_QUEUE_SIZE	12		/* Must be > 4 && <= TX_RING_SIZE */
#define RX_RING_SIZE	32		/* TSILL_TODO was 64, reduced because RTEMS ran out of mbufs */
/* TODO: use of MBUF clusters (4k each) seems to be quite wasteful
 * for ethernet...
 */

/* Operational parameters that usually are not changed. */
/* Time in jiffies before concluding the transmitter is hung. */
#define TX_TIMEOUT  (2*HZ)
#define PKT_BUF_SZ		1536			/* Size of each temporary Rx buffer.*/

#if !defined(__OPTIMIZE__)
#warning  You must compile this file with the correct options!
#warning  See the last lines of the source file.
#error You must compile this driver with "-O".
#endif

#ifdef TSILL_TODO

#include <linux/config.h>
#if defined(CONFIG_SMP) && ! defined(__SMP__)
#define __SMP__
#endif
#if defined(MODULE) && defined(CONFIG_MODVERSIONS) && ! defined(MODVERSIONS)
#define MODVERSIONS
#endif

#include <linux/version.h>
#include <linux/module.h>
#if LINUX_VERSION_CODE < 0x20300  &&  defined(MODVERSIONS)
#include <linux/modversions.h>
#endif

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/malloc.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <asm/processor.h>		/* Processor type for cache alignment. */
#include <asm/unaligned.h>
#include <asm/bitops.h>
#include <asm/io.h>

#ifdef INLINE_PCISCAN
#include "k_compat.h"
#else
#include "pci-scan.h"
#include "kern_compat.h"
#endif

/* Condensed operations for readability. */
#define virt_to_le32desc(addr)  cpu_to_le32(virt_to_bus(addr))
#define le32desc_to_virt(addr)  bus_to_virt(le32_to_cpu(addr))

#if (LINUX_VERSION_CODE >= 0x20100)  &&  defined(MODULE)
char kernel_version[] = UTS_RELEASE;
#endif

MODULE_AUTHOR("Donald Becker <becker@scyld.com>");
MODULE_DESCRIPTION("Packet Engines Yellowfin G-NIC Gigabit Ethernet driver");
MODULE_PARM(max_interrupt_work, "i");
MODULE_PARM(mtu, "i");
MODULE_PARM(debug, "i");
MODULE_PARM(rx_copybreak, "i");
MODULE_PARM(options, "1-" __MODULE_STRING(MAX_UNITS) "i");
MODULE_PARM(full_duplex, "1-" __MODULE_STRING(MAX_UNITS) "i");
MODULE_PARM(gx_fix, "i");

#else /* TSILL_TODO */

#ifndef KERNEL
#define KERNEL
#endif

#include <rtems.h>
#include <rtems/bspIo.h>				/* printk */
#include <stdio.h>						/* printf for statistics */
#include <string.h>
#include <bsp/irq.h>
#include <libcpu/byteorder.h>			/* st_le32 & friends */
#include <libcpu/io.h>					/* inp & friends */
#include <bsp.h>

/* our memory address seen from the PCI bus */
#define virt_to_bus(addr)	((addr)+PCI_DRAM_OFFSET)	/* on CHRP :-) */
/* and back... */
#define bus_to_virt(addr)	((addr)-PCI_DRAM_OFFSET)	/* on CHRP :-) */

#define le32_to_cpu(var)	ld_le32((volatile unsigned *)&var)
#ifdef __PPC
#define get_unaligned(addr)	(*(addr))
#endif

#include <rtems/error.h>
#include "yf_rtemscompat.h"
#include <bsp/pci.h>
#include <rtems/rtems_bsdnet.h>

#include <sys/param.h>
#include <sys/mbuf.h>

#include <sys/socket.h>
#include <sys/sockio.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>

/* RTEMS event to kill the daemon */
#define KILL_EVENT		RTEMS_EVENT_1
/* RTEMS event to restart the transmitter */
#define RESTART_EVENT	RTEMS_EVENT_2
/* RTEMS events used by the ISR */
#define RX_EVENT		RTEMS_EVENT_3
#define TX_EVENT		RTEMS_EVENT_4
#define ERR_EVENT		RTEMS_EVENT_5

#define ALL_EVENTS (KILL_EVENT|RESTART_EVENT|RX_EVENT|TX_EVENT|ERR_EVENT)

/* NOTE: (surprise...)
 * bsdnet redefines malloc to its own rtems_bsdnet_malloc
 */

/* TODO, for the moment, the synergy method is hardcoded.
 *       Should the driver be moved out to the libchip dir,
 *       then we must probably invoke a BSP specific routine here
 */
#ifndef BSP_YELLOWFIN_SUPPLY_HWADDR
#define BSP_YELLOWFIN_SUPPLY_HWADDR(dest, instance) svgm_get_hwaddr(dest,instance)	

/* only one instance supported */
static void
svgm_get_hwaddr(unsigned char *dest,int instance)
{
unsigned char	*nvram_id=(unsigned char*)0xffe9e778;
int 		i;
	/* first three bytes are the SYNERGY manufacturer ID */
	*(dest++)=0x00;
	*(dest++)=0x80;
	*(dest++)=0xF6;
	for (i=0; i<3; i++)
		*dest++ = *nvram_id++;
}
#endif

/* map pci interrupt to 'name' [the shared PPC irq interface could be better designed] */
#define	BSP_PCIIRQ2NAME(irq) ((irq)+BSP_PCI_IRQ_LOWEST_OFFSET)
#define	BSP_NAME2PCIIRQ(name) ((name)-BSP_PCI_IRQ_LOWEST_OFFSET)

/* Condensed operations for readability. */
#ifdef TSILL_TODO
#define virt_to_le32desc(addr)  cpu_to_le32(virt_to_bus(addr))
#endif
#define le32desc_to_virt(addr)  bus_to_virt((void*)le32_to_cpu(addr))

#endif

/*
				Theory of Operation

I. Board Compatibility

This device driver is designed for the Packet Engines "Yellowfin" Gigabit
Ethernet adapter.  The only PCA currently supported is the G-NIC 64-bit
PCI card.

II. Board-specific settings

PCI bus devices are configured by the system at boot time, so no jumpers
need to be set on the board.  The system BIOS preferably should assign the
PCI INTA signal to an otherwise unused system IRQ line.
Note: Kernel versions earlier than 1.3.73 do not support shared PCI
interrupt lines.

III. Driver operation

IIIa. Ring buffers

The Yellowfin uses the Descriptor Based DMA Architecture specified by Apple.
This is a descriptor list scheme similar to that used by the EEPro100 and
Tulip.  This driver uses two statically allocated fixed-size descriptor lists
formed into rings by a branch from the final descriptor to the beginning of
the list.  The ring sizes are set at compile time by RX/TX_RING_SIZE.

The driver allocates full frame size skbuffs for the Rx ring buffers at
open() time and passes the skb->data field to the Yellowfin as receive data
buffers.  When an incoming frame is less than RX_COPYBREAK bytes long,
a fresh skbuff is allocated and the frame is copied to the new skbuff.
When the incoming frame is larger, the skbuff is passed directly up the
protocol stack and replaced by a newly allocated skbuff.

The RX_COPYBREAK value is chosen to trade-off the memory wasted by
using a full-sized skbuff for small frames vs. the copying costs of larger
frames.  For small frames the copying cost is negligible (esp. considering
that we are pre-loading the cache with immediately useful header
information).  For large frames the copying cost is non-trivial, and the
larger copy might flush the cache of useful data.

IIIC. Synchronization - Original implementation DOES NOT APPLY TO RTEMS

The driver runs as two independent, single-threaded flows of control.  One
is the send-packet routine, which enforces single-threaded use by the
dev->tbusy flag.  The other thread is the interrupt handler, which is single
threaded by the hardware and other software.

The send packet thread has partial control over the Tx ring and 'dev->tbusy'
flag.  It sets the tbusy flag whenever it's queuing a Tx packet. If the next
queue slot is empty, it clears the tbusy flag when finished otherwise it sets
the 'yp->tx_full' flag.

The interrupt handler has exclusive control over the Rx ring and records stats
from the Tx ring.  After reaping the stats, it marks the Tx queue entry as
empty by incrementing the dirty_tx mark. Iff the 'yp->tx_full' flag is set, it
clears both the tx_full and tbusy flags.

IIIC-i. Synchronization - RTEMS port (Till Straumann)

The driver's work is done by _one_single_ task who blocks on an event
while it is idle. An RX or TX interrupt does not perform any work but
simply weaks up the driver task thus keeping interrupt latencies minimal.

Upon reception of an event, the driver task first passes received packets
(possibly this involves copying packets smaller than rx_copybreak) upstream
using 'ether_input()'. When no more packets can be received, the driver first
cleans up TX descriptors and then proceeds dequeuing mbufs from the IF's 
output queue into free descriptors. When no more work can be done,
the driver goes again to sleep.


IV. Notes

Thanks to Kim Stearns of Packet Engines for providing a pair of G-NIC boards.
Thanks to Bruce Faust of Digitalscape for providing both their SYM53C885 board
and an AlphaStation to verifty the Alpha port!

IVb. References

Yellowfin Engineering Design Specification, 4/23/97 Preliminary/Confidential
Symbios SYM53C885 PCI-SCSI/Fast Ethernet Multifunction Controller Preliminary
   Data Manual v3.0
http://cesdis.gsfc.nasa.gov/linux/misc/NWay.html
http://cesdis.gsfc.nasa.gov/linux/misc/100mbps.html

IVc. Errata

See Packet Engines confidential appendix (prototype chips only).
*/




enum capability_flags {
	HasMII=1, FullTxStatus=2, IsGigabit=4, HasMulticastBug=8, FullRxStatus=16,
	HasMACAddrBug=32,			/* Only on early revs.  */
};

#ifdef TSILL_TODO
/* The PCI I/O space extent. */
#define YELLOWFIN_SIZE 0x100
#ifdef USE_IO_OPS
#define PCI_IOTYPE (PCI_USES_MASTER | PCI_USES_IO  | PCI_ADDR0)
#else
#define PCI_IOTYPE (PCI_USES_MASTER | PCI_USES_MEM | PCI_ADDR1)
#endif
#endif

struct pci_id_info {
	char *name;
	unsigned short	vendor, device;
	int  drv_flags;
};

/* don't change the order of this table: table index is used as "chip_id" */
static struct pci_id_info pci_id_tbl[] = {
	{"Yellowfin G-NIC Gigabit Ethernet",
	 0x1000, 0x0702,
	 FullTxStatus | IsGigabit | HasMulticastBug | HasMACAddrBug},
	{"Symbios SYM83C885",
	 0x1000, 0x0701,
	 HasMII },
	{0,},
};

/* Offsets to the Yellowfin registers.  Various sizes and alignments. */
enum yellowfin_offsets {
	TxCtrl=0x00, TxStatus=0x04, TxPtr=0x0C,
	TxIntrSel=0x10, TxBranchSel=0x14, TxWaitSel=0x18,
	RxCtrl=0x40, RxStatus=0x44, RxPtr=0x4C,
	RxIntrSel=0x50, RxBranchSel=0x54, RxWaitSel=0x58,
	EventStatus=0x80, IntrEnb=0x82, IntrClear=0x84, IntrStatus=0x86,
	ChipRev=0x8C, DMACtrl=0x90, TxThreshold=0x94,
	Cnfg=0xA0, FrameGap0=0xA2, FrameGap1=0xA4,
	MII_Cmd=0xA6, MII_Addr=0xA8, MII_Wr_Data=0xAA, MII_Rd_Data=0xAC,
	MII_Status=0xAE,
	RxDepth=0xB8, FlowCtrl=0xBC,
	AddrMode=0xD0, StnAddr=0xD2, HashTbl=0xD8, FIFOcfg=0xF8,
	EEStatus=0xF0, EECtrl=0xF1, EEAddr=0xF2, EERead=0xF3, EEWrite=0xF4,
	EEFeature=0xF5,
};

/* The Yellowfin Rx and Tx buffer descriptors.
   Elements are written as 32 bit for endian portability. */
struct yellowfin_desc {
	u32 dbdma_cmd;
	u32 addr;
	u32 branch_addr;
	u32 result_status;
};

struct tx_status_words {
#if defined(__powerpc__)
	u16 tx_errs;
	u16 tx_cnt;
	u16 paused;
	u16 total_tx_cnt;
#else  /* Little endian chips. */
	u16 tx_cnt;
	u16 tx_errs;
	u16 total_tx_cnt;
	u16 paused;
#endif
};

/* Bits in yellowfin_desc.cmd */
enum desc_cmd_bits {
	CMD_TX_PKT=0x10000000, CMD_RX_BUF=0x20000000, CMD_TXSTATUS=0x30000000,
	CMD_NOP=0x60000000, CMD_STOP=0x70000000,
	BRANCH_ALWAYS=0x0C0000, INTR_ALWAYS=0x300000, WAIT_ALWAYS=0x030000,
	BRANCH_IFTRUE=0x040000,
};

/* Bits in yellowfin_desc.status */
enum desc_status_bits { RX_EOP=0x0040, };

/* Bits in the interrupt status/mask registers. */
enum intr_status_bits {
	IntrRxDone=0x01, IntrRxInvalid=0x02, IntrRxPCIFault=0x04,IntrRxPCIErr=0x08,
	IntrTxDone=0x10, IntrTxInvalid=0x20, IntrTxPCIFault=0x40,IntrTxPCIErr=0x80,
	IntrEarlyRx=0x100, IntrWakeup=0x200, };

#define PRIV_ALIGN	31 	/* Required alignment mask */
struct yellowfin_private {
	/* Descriptor rings first for alignment.
	   Tx requires a second descriptor for status. */
	struct yellowfin_desc rx_ring[RX_RING_SIZE];
	struct yellowfin_desc tx_ring[TX_RING_SIZE*2];
	struct yellowfin_private *next_module;
	void *priv_addr;					/* Unaligned address for kfree */
	/* The addresses of receive-in-place skbuffs. */
	struct mbuf* rx_mbuf[RX_RING_SIZE];
	/* The saved address of a sent-in-place packet/buffer, for later free(). */
	struct mbuf* tx_mbuf[TX_RING_SIZE];
	struct tx_status_words tx_status[TX_RING_SIZE];
#ifdef TSILL_TODO
	struct timer_list timer;	/* Media selection timer. */
	struct net_device_stats stats;
#endif
	/* Frequently used and paired value: keep adjacent for cache effect. */
	int chip_id, drv_flags;
	long in_interrupt;
	struct yellowfin_desc *rx_head_desc;
	unsigned int cur_rx, dirty_rx;		/* Producer/consumer ring indices */
	unsigned int rx_buf_sz;				/* Based on MTU+slack. */
	struct tx_status_words *tx_tail_desc;
	unsigned int cur_tx, dirty_tx;
	int tx_threshold;
	unsigned int tx_free:5;				/* free Tx queue slots. */
	unsigned int full_duplex:1;			/* Full-duplex operation requested. */
	unsigned int duplex_lock:1;
	unsigned int medialock:1;			/* Do not sense media. */
	/* MII transceiver section. */
	int mii_cnt;						/* MII device addresses. */
	u16 advertising;					/* NWay media advertisement */
	unsigned char phys[2];				/* MII device addresses. */
	long	base_addr;
	int	irq;
	struct	arpcom	arpcom;				/* rtems if structure, contains ifnet */
	rtems_id	daemonTid;
	rtems_id	daemonSync;				/* synchronization with the daemon */

	long 		 intr_status;			/* this field is modified by the ISR */
	unsigned long old_iff;				/* cache the interface flags because
										 * bsdnet updates them before calling ioctl
										 */
	/* statistics */
	struct {
		unsigned long	txReassembled;
		unsigned long 	length_errors;
		unsigned long	frame_errors;
		unsigned long	crc_errors;
		unsigned long	nTxIrqs, nRxIrqs;
	} stats;
};


#ifndef BSP_YELLOWFIN_SUPPLY_HWADDR
static int read_eeprom(long ioaddr, int location);
#endif
static void yellowfin_stats(struct yellowfin_private *yp);
static int mdio_read(long ioaddr, int phy_id, int location);
static void mdio_write(long ioaddr, int phy_id, int location, int value);
static int yellowfin_ioctl(struct ifnet *ifp, int cmd, caddr_t data);
static int yellowfin_init_hw(struct yellowfin_private *yp);
static void yellowfin_timer(struct ifnet *ifp);
#ifdef TSILL_TODO
static void yellowfin_tx_timeout(struct net_device *dev);
#endif
static void yellowfin_init_ring(struct yellowfin_private *yp);
static int yellowfin_sendpacket(struct mbuf *skb, struct yellowfin_private *dev);
static int yellowfin_rx(struct yellowfin_private *dev);
static void yellowfin_error(struct yellowfin_private *dev);
static void yellowfin_stop(struct yellowfin_private *yp);
static int yellowfin_stop_hw(struct yellowfin_private *yp);
static void yellowfin_start(struct ifnet * ifp);
static rtems_isr yellowfin_isr(void);
static void yellowfin_irq_on(const rtems_irq_connect_data *);
static void yellowfin_irq_off(const rtems_irq_connect_data *);
static int yellowfin_irq_is_on(const rtems_irq_connect_data *);
static void yellowfin_init(void *arg);
static void yellowfin_daemon(void *arg);
static int set_rx_mode(struct yellowfin_private *yp);


/* A list of installed Yellowfin devices */
static struct yellowfin_private *root_yellowfin_dev = NULL;

/* attach parameter tells us whether to attach or to detach the driver */
int
rtems_yellowfin_driver_attach(struct rtems_bsdnet_ifconfig *config, int attach)
{
	struct yellowfin_private *np;
	void		*priv_mem;
	int		i, option,unit;
	int		drv_flags;
	int		pciB,pciD,pciF,chip_idx;
	unsigned long 	ioaddr;
	unsigned char	irq, hwaddr[ETHER_ADDR_LEN];
	char			*name;
	struct ifnet	*ifp;

	unit = rtems_bsdnet_parse_driver_name(config, &name);
	if (unit < 0)
		return 0;

	if (root_yellowfin_dev && 1!=unit) {
		rtems_panic("yellowfin: multiple devices not implemented; only unit 1 supported");
		/* support for multiple devices would have to:
		 *  - search PCI bus(es) omitting yellowfins alreay
		 *    in use
		 *  - ISR would have to figure out IRQ source. This
		 *    is complicated by the fact that RTEMS doesn't allow
		 *    for passing an ISR argument :-( :-( :-(
		 *    Most likely, you would have to install a _different_
		 *    isr for each device instance, so the isr can 
		 *    calculate the instance number from its own address
		 *    (uaarrgh...)
		 */
		return 0;
	}

	printk("Yellowfin: A Packet Engines G-NIC ethernet driver for RTEMS\n\n");
	printk("Copyright 1997-2001, Donald Becker <becker@scyld.com>\n");
	printk("          2001-2002, Till Straumann <strauman@slac.stanford.edu> (RTEMS port)\n");
	printk("LICENSE:   This driver is released under the terms of the GPL,\n");
	printk("           consult http://www.gnu.org for details\n");

	/* scan the PCI bus */
	for (i=0; pci_id_tbl[i].name; i++) {
		if (0==BSP_pciFindDevice(pci_id_tbl[i].vendor,
					   pci_id_tbl[i].device,
					   unit - 1, /* instance - NOTE: currently unimplemented */
					   &pciB,
					   &pciD,
					   &pciF))
			break;
	}

	if ( !pci_id_tbl[i].name) {
		printk("yellowfin_probe: NO DEVICE FOUND, giving up...\n");
		return 0;
	}
	chip_idx=i;

	drv_flags = pci_id_tbl[i].drv_flags;

	/* get base address + interrupt */
	pci_read_config_dword(pciB, pciD, pciF, PCI_BASE_ADDRESS_0, (void*)&ioaddr);
	ioaddr &= PCI_BASE_ADDRESS_IO_MASK;

	pci_read_config_byte(pciB, pciD, pciF, PCI_INTERRUPT_LINE, &irq);

	if (yellowfin_debug>0)
		printk("%s: %s type %8x at 0x%lx, ",
		   config->name, pci_id_tbl[i].name, inl(ioaddr + ChipRev), ioaddr);

	/* try to read HW address from the device if not overridden
	 * by config
	 */
	if (config->hardware_address) {
		memcpy(hwaddr, config->hardware_address, ETHER_ADDR_LEN);
		if (yellowfin_debug>0)
			printk(" using MAC addr from config:");
	} else {
#ifndef BSP_YELLOWFIN_SUPPLY_HWADDR
		if (drv_flags & IsGigabit)
			for (i = 0; i < 6; i++)
				hwaddr[i] = inb(ioaddr + StnAddr + i);
		else {
			int ee_offset = (read_eeprom(ioaddr, 6) == 0xff ? 0x100 : 0);
			for (i = 0; i < 6; i++)
				hwaddr[i] = read_eeprom(ioaddr, ee_offset + i);
		}
		if (yellowfin_debug>0)
			printk(" using MAC addr from device:");
#else
		BSP_YELLOWFIN_SUPPLY_HWADDR(hwaddr,unit-1);
#endif
	}
	for (i = 0; i < ETHER_ADDR_LEN-1; i++)
		if (yellowfin_debug>0)
			printk("%02x:", hwaddr[i]);
	if (yellowfin_debug>0)
		printk("%02x, IRQ %d.\n", hwaddr[i], irq);

	/* Reset the chip. */
	outl(0x80000000, ioaddr + DMACtrl);

	/* Make certain elements e.g. descriptor lists are aligned. */
	priv_mem = rtems_bsdnet_malloc(sizeof(*np) + PRIV_ALIGN, M_FREE, M_NOWAIT);
	/* Check for the very unlikely case of no memory. */
	if (priv_mem == NULL)
		rtems_panic("yellowfin: OUT OF MEMORY");

	np = (void *)(((long)priv_mem + PRIV_ALIGN) & ~PRIV_ALIGN);
	memset(np, 0, sizeof(*np));
	np->priv_addr = priv_mem;

	np->base_addr = ioaddr;
	np->irq = irq;
	memcpy(np->arpcom.ac_enaddr, hwaddr, ETHER_ADDR_LEN);

	np->next_module = root_yellowfin_dev;
	root_yellowfin_dev = np;

	np->chip_id = chip_idx;
	np->drv_flags = drv_flags;

	ifp = &np->arpcom.ac_if;

 	option = chip_idx < MAX_UNITS ? options[chip_idx] : 0;
	/* The lower four bits are the media type. */
	if (option > 0) {
		if (option & 0x200)
			np->full_duplex = 1;
	}
	np->medialock = 0;
	if (chip_idx < MAX_UNITS  &&  full_duplex[chip_idx] > 0)
		np->full_duplex = 1;

	if (np->full_duplex)
		np->duplex_lock = 1;

	ifp->if_softc = np;

	/* set this interface's name and unit */
	ifp->if_unit = unit;
	ifp->if_name = name;

	ifp->if_mtu = config->mtu ? config->mtu : ETHERMTU;

	/* The Yellowfin-specific entries in the device structure. */
	ifp->if_init     = yellowfin_init;
	ifp->if_ioctl    = yellowfin_ioctl;
	ifp->if_start    = yellowfin_start;
	ifp->if_output   = ether_output;
	ifp->if_watchdog = yellowfin_timer;

	np->old_iff = ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX;
	if (ifp->if_snd.ifq_maxlen == 0)
		ifp->if_snd.ifq_maxlen = ifqmaxlen;

	if (np->drv_flags & HasMII) {
		int phy, phy_idx = 0;
		for (phy = 0; phy < 32 && phy_idx < 4; phy++) {
			int mii_status = mdio_read(ioaddr, phy, 1);
			if (mii_status != 0xffff  &&  mii_status != 0x0000) {
				np->phys[phy_idx++] = phy;
				np->advertising = mdio_read(ioaddr, phy, 4);
				if (yellowfin_debug>0)
					printk("%s: MII PHY found at address %d, status "
					   "0x%04x advertising 0x%04x.\n",
					   ifp->if_name, phy, mii_status, np->advertising);
			}
		}
		np->mii_cnt = phy_idx;
	}

	/* create the synchronization semaphore */
	if (RTEMS_SUCCESSFUL !=
	    rtems_semaphore_create(
				rtems_build_name('y','f','i','n'),
				0,0,0,&np->daemonSync))
		rtems_panic("Yellowfin: semaphore creation failed");


	/* Actually attach the interface */
	if_attach(ifp);
	ether_ifattach(ifp);

	if (yellowfin_debug>1)
		printk("Yellowfin: Ethernet driver has been attached (handle 0x%08x, ifp 0x%08x)\n",
			np, ifp);

	return 1;
}

#ifndef BSP_YELLOWFIN_SUPPLY_HWADDR
static int read_eeprom(long ioaddr, int location)
{
	int bogus_cnt = 10000;		/* Typical 33Mhz: 1050 ticks */

	outb(location, ioaddr + EEAddr);
	outb(0x30 | ((location >> 8) & 7), ioaddr + EECtrl);
	while ((inb(ioaddr + EEStatus) & 0x80)  &&  --bogus_cnt > 0)
		;
	return inb(ioaddr + EERead);
}
#endif

/* MII Managemen Data I/O accesses.
   These routines assume the MDIO controller is idle, and do not exit until
   the command is finished. */

static int mdio_read(long ioaddr, int phy_id, int location)
{
	int i;

	outw((phy_id<<8) + location, ioaddr + MII_Addr);
	outw(1, ioaddr + MII_Cmd);
	for (i = 10000; i >= 0; i--)
		if ((inw(ioaddr + MII_Status) & 1) == 0)
			break;
	return inw(ioaddr + MII_Rd_Data);
}

static void mdio_write(long ioaddr, int phy_id, int location, int value)
{
	int i;

	outw((phy_id<<8) + location, ioaddr + MII_Addr);
	outw(value, ioaddr + MII_Wr_Data);

	/* Wait for the command to finish. */
	for (i = 10000; i >= 0; i--)
		if ((inw(ioaddr + MII_Status) & 1) == 0)
			break;
	return;
}


/* stupid rtems IRQ interface doesn't pass a user
 * pointer in rtems_irq_connect_data.
 * Nor does rtems support IRQ sharing...
 * we cannot even share 1 IRQ among several instances of
 * the same driver :-(
 */

static void yellowfin_irq_on(const rtems_irq_connect_data *c)
{
	struct yellowfin_private *yp;
	for (yp = root_yellowfin_dev; yp; yp=yp->next_module) {
		if (c->name != BSP_PCIIRQ2NAME(yp->irq))
			continue; /* another instance on another line */
		/* see enum intr_status_bits */
		outw(0x81ff, yp->base_addr + IntrEnb);
		return;
	}
	rtems_panic("yellowfin_irq_on: should never get here");
}

static void yellowfin_irq_off(const rtems_irq_connect_data *c)
{
	struct yellowfin_private *yp;
	for (yp = root_yellowfin_dev; yp; yp=yp->next_module) {
		if (c->name != BSP_PCIIRQ2NAME(yp->irq))
			continue; /* another instance on another line */
		/* see enum intr_status_bits */
		outw(0x0000, yp->base_addr + IntrEnb);
		return;
	}
	rtems_panic("yellowfin_irq_off: should never get here");
}

static int yellowfin_irq_is_on(const rtems_irq_connect_data *c)
{
	struct yellowfin_private *yp;
	/* stupid rtems IRQ interface doesn't pass a user
	 * pointer in rtems_irq_connect_data.
	 * Nor does rtems support IRQ sharing :-(
	 */
	for (yp = root_yellowfin_dev; yp; yp=yp->next_module) {
		if (c->name != BSP_PCIIRQ2NAME(yp->irq))
			continue; /* another instance on another line */
		/* see enum intr_status_bits */
		return 0x81ff & inw(yp->base_addr + IntrEnb);
	}
	rtems_panic("yellowfin_irq_is_on: should never get here");
	return 0; /* keep compiler happy */
}


/* Oh well, this is really, really, STUPID
 * --> only ONE instance supported
 */
static void yellowfin_isr(void)
{
struct yellowfin_private *yp = root_yellowfin_dev;
u16				intr_status = inw(yp->base_addr + IntrClear);
rtems_event_set	evs=0;
	/* wakeup the networking task and let it do
	 * all of the work...
	 */
	if ((0x2ee & intr_status) || 0==intr_status) {
			yp->intr_status = intr_status;
			evs |= ERR_EVENT;
	}
	if (IntrTxDone & intr_status) {
			yp->stats.nTxIrqs++;
			evs |= TX_EVENT;
	}
	if ((IntrRxDone|IntrEarlyRx)&intr_status) {
			yp->stats.nRxIrqs++;
			evs |= RX_EVENT;
	}
	rtems_event_send(yp->daemonTid, evs);
}

rtems_irq_connect_data yellowfin_irq_info={
name:	0,
hdl:	yellowfin_isr,
on:		yellowfin_irq_on,
off:	yellowfin_irq_off,
isOn:	yellowfin_irq_is_on,
};


static int yellowfin_init_hw(struct yellowfin_private *yp)
{
	long ioaddr = yp->base_addr;
	struct ifnet *ifp = &yp->arpcom.ac_if;
	int i;

	if (yellowfin_debug > 1)
		printk("Yellowfin: entering init_hw...\n");

	/* Reset the chip. */
	outl(0x80000000, ioaddr + DMACtrl);

	if (yellowfin_debug > 2)
		printk("%s%d: yellowfin_init_hw() irq %d.\n",
			   ifp->if_name, ifp->if_unit, yp->irq);

	yellowfin_init_ring(yp);

	outl((unsigned long)virt_to_bus(yp->rx_ring), ioaddr + RxPtr);
	outl((unsigned long)virt_to_bus(yp->tx_ring), ioaddr + TxPtr);

	for (i = 0; i < 6; i++)
		outb(yp->arpcom.ac_enaddr[i], ioaddr + StnAddr + i);

	/* Set up various condition 'select' registers.
	   There are no options here. */
	outl(0x00800080, ioaddr + TxIntrSel); 	/* Interrupt on Tx abort */
	outl(0x00800080, ioaddr + TxBranchSel);	/* Branch on Tx abort */
	outl(0x00400040, ioaddr + TxWaitSel); 	/* Wait on Tx status */
	outl(0x00400040, ioaddr + RxIntrSel);	/* Interrupt on Rx done */
	outl(0x00400040, ioaddr + RxBranchSel);	/* Branch on Rx error */
	outl(0x00400040, ioaddr + RxWaitSel);	/* Wait on Rx done */

	/* Initialize other registers: with so many this eventually this will
	   converted to an offset/value list. */
	outl(dma_ctrl, ioaddr + DMACtrl);
	outw(fifo_cfg, ioaddr + FIFOcfg);
	/* Enable automatic generation of flow control frames, period 0xffff. */
	outl(0x0030FFFF, ioaddr + FlowCtrl);

	yp->tx_threshold = 32;
	outl(yp->tx_threshold, ioaddr + TxThreshold);

	yp->in_interrupt = 0;

	/* Setting the Rx mode will start the Rx process. */
	if (yp->drv_flags & IsGigabit) {
		/* We are always in full-duplex mode with gigabit! */
		yp->full_duplex = 1;
		outw(0x01CF, ioaddr + Cnfg);
	} else {
		outw(0x0018, ioaddr + FrameGap0); /* 0060/4060 for non-MII 10baseT */
		outw(0x1018, ioaddr + FrameGap1);
		outw(0x101C | (yp->full_duplex ? 2 : 0), ioaddr + Cnfg);
	}

	set_rx_mode(yp);
	/* connect the interrupt handler which should
	 * take care of enabling interrupts
	 */
	yellowfin_irq_info.name = BSP_PCIIRQ2NAME(yp->irq);
	if (!BSP_install_rtems_irq_handler(&yellowfin_irq_info))
		rtems_panic("yellowfin: unable to install ISR");

	outw(0x0000, ioaddr + EventStatus);		/* Clear non-interrupting events */
	outl(0x80008000, ioaddr + RxCtrl);		/* Start Rx and Tx channels. */
	outl(0x80008000, ioaddr + TxCtrl);

	/* Set the timer to check for link beat. */
	ifp->if_timer = 3*IFNET_SLOWHZ; /* seconds */

	if (yellowfin_debug > 1) {
		printk("%s%d: Done yellowfin_init_hw().\n",
			   ifp->if_name,ifp->if_unit);
	}

	return 0;
}

static void yellowfin_init(void *arg)
{
	struct yellowfin_private *yp = arg;
	if (yellowfin_debug > 1 )
		printk("yellowfin_init(): entering... (YP: 0x%08x, daemon ID: 0x%08x)\n",
				yp, yp->daemonTid);

	if (yp->daemonTid) {
		if (yellowfin_debug>1)
			printk("Yellowfin: daemon already up, doing nothing\n");
		return;
	}
	/* initialize the hardware (we are holding the network semaphore at this point) */
	(void)yellowfin_init_hw(yp);
	/* launch network daemon */

	/* NOTE:
	 * in ss-20011025 (and later) any task created by 'bsdnet_newproc' is
 	 * wrapped by code which acquires the network semaphore...
	 */
	yp->daemonTid = rtems_bsdnet_newproc(YELLOWFIN_TASK_NAME,4096,yellowfin_daemon,arg);

	/* I guess that we own the bsdnet_semaphore at this point.
	 * Hence, we must set IFF_RUNNING in our context rather than
	 * having the daemon do it [although it is probably safe, too].
	 */
	yp->arpcom.ac_if.if_flags |= IFF_RUNNING;

	if (yellowfin_debug > 1 )
		printk("yellowfin_init(): leaving.\n");
}

static void yellowfin_timer(struct ifnet *ifp)
{
	struct yellowfin_private *yp = (struct yellowfin_private *)ifp->if_softc;
	long ioaddr = yp->base_addr;
	int next_tick = 60*IFNET_SLOWHZ;

	if (yellowfin_debug > 3) {
		printk("%s: Yellowfin timer tick, status %08x.\n",
			   ifp->if_name, inw(ioaddr + IntrStatus));
	}

#ifdef TSILL_TODO
	if (jiffies - dev->trans_start > TX_TIMEOUT
		&& yp->cur_tx - yp->dirty_tx > 1
		&& netif_queue_paused(dev))
		yellowfin_tx_timeout(dev);
#endif

	if (yp->mii_cnt) {
		int mii_reg1 = mdio_read(ioaddr, yp->phys[0], 1);
		int mii_reg5 = mdio_read(ioaddr, yp->phys[0], 5);
		int negotiated = mii_reg5 & yp->advertising;
		if (yellowfin_debug > 1)
			printk("%s: MII #%d status register is %04x, "
				   "link partner capability %04x.\n",
				   ifp->if_name, yp->phys[0], mii_reg1, mii_reg5);

		if ( ! yp->duplex_lock &&
			 ((negotiated & 0x0300) == 0x0100
			  || (negotiated & 0x00C0) == 0x0040)) {
			yp->full_duplex = 1;
		}
		outw(0x101C | (yp->full_duplex ? 2 : 0), ioaddr + Cnfg);

		if (mii_reg1 & 0x0004)
			next_tick = 60*IFNET_SLOWHZ;
		else
			next_tick = 3*IFNET_SLOWHZ;
	}

	ifp->if_timer = next_tick;
}

#ifdef TSILL_TODO
static void yellowfin_tx_timeout(struct net_device *dev)
{
	struct yellowfin_private *yp = (struct yellowfin_private *)dev->priv;
	long ioaddr = dev->base_addr;

	printk(KERN_WARNING "%s: Yellowfin transmit timed out at %d/%d Tx "
		   "status %4.4x, Rx status %4.4x, resetting...\n",
		   dev->name, yp->cur_tx, yp->dirty_tx,
		   inl(ioaddr + TxStatus), inl(ioaddr + RxStatus));

	/* Note: these should be KERN_DEBUG. */
	if (yellowfin_debug) {
		int i;
		printk(KERN_WARNING "  Rx ring %p: ", yp->rx_ring);
		for (i = 0; i < RX_RING_SIZE; i++)
			printk(" %8.8x", yp->rx_ring[i].result_status);
		printk("\n"KERN_WARNING"  Tx ring %p: ", yp->tx_ring);
		for (i = 0; i < TX_RING_SIZE; i++)
			printk(" %4.4x /%8.8x", yp->tx_status[i].tx_errs,
				   yp->tx_ring[i].result_status);
		printk("\n");
	}

	/* If the hardware is found to hang regularly, we will update the code
	   to reinitialize the chip here. */
	dev->if_port = 0;

	/* Wake the potentially-idle transmit channel. */
	outl(0x10001000, dev->base_addr + TxCtrl);
	if (yp->cur_tx - yp->dirty_tx < TX_QUEUE_SIZE)
		netif_unpause_tx_queue(dev);

	dev->trans_start = jiffies;
	yp->stats.tx_errors++;
	return;
}
#endif

/* Initialize the Rx and Tx rings, along with various 'dev' bits. */
static void yellowfin_init_ring(struct yellowfin_private *yp)
{
	int i;

	yp->tx_free = TX_RING_SIZE;
	yp->cur_rx = yp->cur_tx = 0;
	yp->dirty_tx = 0;

	yp->rx_buf_sz = (yp->arpcom.ac_if.if_mtu <= 1500 ? PKT_BUF_SZ : yp->arpcom.ac_if.if_mtu + 32);
	yp->rx_head_desc = &yp->rx_ring[0];

	for (i = 0; i < RX_RING_SIZE; i++) {
		st_le32((volatile unsigned32 *)
			&yp->rx_ring[i].dbdma_cmd,
			CMD_RX_BUF | INTR_ALWAYS | yp->rx_buf_sz);
		st_le32((volatile unsigned32 *)
			&yp->rx_ring[i].branch_addr,
		       	(unsigned long)virt_to_bus(&yp->rx_ring[i+1]));
	}
	/* Mark the last entry as wrapping the ring. */
	st_le32((volatile unsigned32 *)
		&yp->rx_ring[i-1].branch_addr,
	        (unsigned long)virt_to_bus(&yp->rx_ring[0]));

	for (i = 0; i < RX_RING_SIZE; i++) {
		struct mbuf *m;
		MGETHDR(m, M_WAIT, MT_DATA);
		MCLGET(m, M_WAIT);
		m->m_pkthdr.rcvif =  &yp->arpcom.ac_if;

		yp->rx_mbuf[i] = m;
		st_le32((volatile unsigned32 *)
			&yp->rx_ring[i].addr,
		       	(unsigned long)virt_to_bus(mtod(m, void*)));
	}
	st_le32((volatile unsigned32 *)
		&yp->rx_ring[i-1].dbdma_cmd,
	       	CMD_STOP);
	yp->dirty_rx = (unsigned int)(i - RX_RING_SIZE);

	/* TODO: T.S. machines with a non-snooping copyback cache
	 *       enabled on the RX buffers should flush/invalidate
	 *       the cache lines overlapping the buffer here.
	 *       (flush lines which overlap with adjacent memory,
	 *       invalidate others)
	 */

#define NO_TXSTATS
#ifdef NO_TXSTATS
	/* In this mode the Tx ring needs only a single descriptor. */
	for (i = 0; i < TX_RING_SIZE; i++) {
		yp->tx_mbuf[i] = 0;
		st_le32((volatile unsigned32 *)
			& yp->tx_ring[i].dbdma_cmd,
		       	CMD_STOP);
		st_le32((volatile unsigned32 *)
			&yp->tx_ring[i].branch_addr,
		       	(unsigned long)virt_to_bus(&yp->tx_ring[i+1]));
	}
	/* Wrap ring */
	st_le32((volatile unsigned32 *)
		&yp->tx_ring[--i].dbdma_cmd,
	       	CMD_STOP | BRANCH_ALWAYS);
	st_le32((volatile unsigned32 *)
		&yp->tx_ring[i].branch_addr,
	       	(unsigned long)virt_to_bus(&yp->tx_ring[0]));
#else
	/* Tx ring needs a pair of descriptors, the second for the status. */
	for (i = 0; i < TX_RING_SIZE*2; i++) {
		yp->tx_skbuff[i/2] = 0;
		/* Branch on Tx error. */
		yp->tx_ring[i].dbdma_cmd = cpu_to_le32(CMD_STOP);
		yp->tx_ring[i].branch_addr = virt_to_le32desc(&yp->tx_ring[i+1]);
		i++;
		if (yp->flags & FullTxStatus) {
			yp->tx_ring[i].dbdma_cmd =
				cpu_to_le32(CMD_TXSTATUS | sizeof(yp->tx_status[i]));
			yp->tx_ring[i].request_cnt = sizeof(yp->tx_status[i]);
			yp->tx_ring[i].addr = virt_to_le32desc(&yp->tx_status[i/2]);
		} else {				/* Symbios chips write only tx_errs word. */
			yp->tx_ring[i].dbdma_cmd =
				cpu_to_le32(CMD_TXSTATUS | INTR_ALWAYS | 2);
			yp->tx_ring[i].request_cnt = 2;
			yp->tx_ring[i].addr = virt_to_le32desc(&yp->tx_status[i/2].tx_errs);
		}
		yp->tx_ring[i].branch_addr = virt_to_le32desc(&yp->tx_ring[i+1]);
	}
	/* Wrap ring */
	yp->tx_ring[--i].dbdma_cmd |= cpu_to_le32(BRANCH_ALWAYS | INTR_ALWAYS);
	yp->tx_ring[i].branch_addr = virt_to_le32desc(&yp->tx_ring[0]);
#endif
	yp->tx_tail_desc = &yp->tx_status[0];
	return;
}

static int yellowfin_sendpacket(struct mbuf *m, struct yellowfin_private *yp)
{
	unsigned entry;

#if LINUX_VERSION_CODE < 0x20323 && TSILL_TODO > 0xdeadbeef /* TSILL_TODO */
	/* Block a timer-based transmit from overlapping.  This could better be
	   done with atomic_swap(1, dev->tbusy), but set_bit() works as well. */
	if (netif_pause_tx_queue(dev) != 0) {
		/* This watchdog code is redundant with the media monitor timer. */
		if (jiffies - dev->trans_start > TX_TIMEOUT)
			yellowfin_tx_timeout(dev);
		return 1;
	}
#endif

	/* Note: Ordering is important here, set the field with the
	   "ownership" bit last, and only then increment cur_tx. */

	/* Calculate the next Tx descriptor entry. */
	entry = yp->cur_tx % TX_RING_SIZE;

#ifndef TSILL_FINAL
	if (yp->tx_mbuf[entry])
		rtems_panic("yellowfin: mbuf leak");
#endif
	yp->tx_mbuf[entry] = m;

	if (gx_fix) {		/* Note: only works for paddable protocols e.g. IP. */
		int cacheline_end = (virt_to_bus(mtod(m,int)) + m->m_len) % 32;
		/* Fix GX chipset errata. */
		if (cacheline_end > 24  || cacheline_end == 0)
			m->m_len += 32 - cacheline_end + 1;
	}
#ifdef NO_TXSTATS
	/* T.S. Unfortunately, I have no datasheet of the 53C885 :-(
	 * therefore, I cannot properly implement letting the hardware
	 * "gather" mbufs into to one packet
	 */
	if (m->m_next) {
		/* TODO: implement letting the hardware do this */
		struct mbuf	*mtmp, *mdst;
		unsigned char	*chpt;
		int		len;
		/* multiple mbufs in this packet, we have to
		 * reassemble :-(
		 */
		/* alloc single destination buffer - we don't really
		 * need a header but it makes things simpler
		 */
		MGETHDR(mdst, M_WAIT, MT_DATA);
		MCLGET(mdst, M_WAIT);
		for (mtmp=m, len=0, chpt=mtod(mdst,unsigned char*);
		     mtmp;
		     mtmp=mtmp->m_next) {
			if (len+mtmp->m_len > sizeof(union mcluster))
				rtems_panic("yellowfin: packet waaayyy too large");
			memcpy((void*)chpt, mtod(mtmp, char*), mtmp->m_len);
			chpt+=mtmp->m_len;
			len+=mtmp->m_len;
		}
		mdst->m_len=len;
		/* free old mbuf chain */
		m_freem(m);
		yp->tx_mbuf[entry] = m = mdst;
		/* statistics */
		yp->stats.txReassembled++;
	} else {
		/* single mbuf packet; just pass it on */
	}
	st_le32((volatile unsigned32 *)
		&yp->tx_ring[entry].addr,
	        (unsigned long)virt_to_bus(mtod(m, void *)));
	yp->tx_ring[entry].result_status = 0;
	if (entry >= TX_RING_SIZE-1) {
		/* New stop command. */
		st_le32((volatile unsigned32 *)
			&yp->tx_ring[0].dbdma_cmd,
		        CMD_STOP);
		st_le32((volatile unsigned32 *)
			&yp->tx_ring[TX_RING_SIZE-1].dbdma_cmd,
			(CMD_TX_PKT|BRANCH_ALWAYS | m->m_len));
	} else {
		st_le32((volatile unsigned32 *)
			&yp->tx_ring[entry+1].dbdma_cmd,
			CMD_STOP);
		st_le32((volatile unsigned32 *)
			&yp->tx_ring[entry].dbdma_cmd,
			(CMD_TX_PKT | BRANCH_IFTRUE | m->m_len));
	}
	yp->cur_tx++;
#else
#error "TODO: RTEMS implementation"
	yp->tx_ring[entry<<1].request_cnt = skb->len;
	yp->tx_ring[entry<<1].addr = virt_to_le32desc(skb->data);
	/* The input_last (status-write) command is constant, but we must rewrite
	   the subsequent 'stop' command. */

	yp->cur_tx++;
	{
		unsigned next_entry = yp->cur_tx % TX_RING_SIZE;
		yp->tx_ring[next_entry<<1].dbdma_cmd = cpu_to_le32(CMD_STOP);
	}
	/* Final step -- overwrite the old 'stop' command. */

	yp->tx_ring[entry<<1].dbdma_cmd =
		cpu_to_le32( ((entry % 6) == 0 ? CMD_TX_PKT|INTR_ALWAYS|BRANCH_IFTRUE :
					  CMD_TX_PKT | BRANCH_IFTRUE) | skb->len);
#endif

	/* Non-x86 Todo: explicitly flush cache lines here.
	 * T.S: my guess would be that the cache should be flushed
	 * _before_ handing the descriptor over - however, I have
	 * no datasheet and therefore, I don't know when this
	 * happens exactly: when waking the TX or (as I _guess_)
	 * when setting dbdma_cmd???
	 * Luckily, The 604 (and up) PowerPC has a _snooping_cache_
	 * much nicer than the 860...
	 */

	/* Wake the potentially-idle transmit channel. */
	outl(0x10001000, yp->base_addr + TxCtrl);

#ifdef TSILL_TODO
	if (yp->cur_tx - yp->dirty_tx >= TX_QUEUE_SIZE) {
		netif_stop_tx_queue(dev);
		yp->tx_full = 1;
		if (yp->cur_tx - (volatile int)yp->dirty_tx < TX_QUEUE_SIZE) {
			netif_unpause_tx_queue(dev);
			yp->tx_full = 0;
		} else
			netif_stop_tx_queue(dev);
	} else
		netif_unpause_tx_queue(dev);		/* Typical path */
	dev->trans_start = jiffies;
#endif
	yp->tx_free--;

	if (yellowfin_debug > 4) {
		printk("%s: Yellowfin transmit frame #%d queued in slot %d.\n",
			   yp->arpcom.ac_if.if_name, yp->cur_tx, entry);
	}
	return 0;
}

/* The daemon does all of the work; RX, TX and cleaning up buffers/descriptors */
static void yellowfin_daemon(void *arg)
{
struct yellowfin_private *yp = (struct yellowfin_private*)arg;
long		ioaddr = yp->base_addr;
int		boguscnt = max_interrupt_work;
rtems_event_set	events;
struct mbuf	*m;
struct ifnet	*ifp=&yp->arpcom.ac_if;

	if (yellowfin_debug > 1)
		printk("Yellowfin daemon: starting...\n");

#if 0	/* see comment in yellowfin_init(); in newer versions of
	   rtems, we old the network semaphore at this point
	 */
	rtems_semaphore_release(yp->daemonSync);
#endif

	/* NOTE: our creator possibly holds the bsdnet_semaphore.
	 *       since that has PRIORITY_INVERSION enabled, our
	 *       subsequent call to bsdnet_event_receive() will
	 *       _not_ release it. It's still in posession of our
	 *       owner.
	 *       This is different from how killing this task
	 *       is handled.
	 */

	for (;;) {
		/* sleep until there's work to be done */
		/* Note: bsdnet_event_receive() acquires
		 *       the global bsdnet semaphore for
		 *       mutual exclusion.
		 */
		rtems_bsdnet_event_receive(ALL_EVENTS,
				RTEMS_WAIT | RTEMS_EVENT_ANY,
				RTEMS_NO_TIMEOUT,
				&events);

		if (KILL_EVENT & events) {
			break;
		}

		boguscnt = max_interrupt_work;

		if (events & RX_EVENT) {
			boguscnt-=yellowfin_rx(yp);
			outl(0x10001000, ioaddr + RxCtrl);		/* Wake Rx engine. */
		}

#ifdef NO_TXSTATS
		/* clean up and try sending packets */
		do {
			m=0;
			/* try to cleanup TX buffers */
			for (; yp->cur_tx - yp->dirty_tx > 0; yp->dirty_tx++) {
				int entry = yp->dirty_tx % TX_RING_SIZE;
				if (yp->tx_ring[entry].result_status == 0)
					break;
				ifp->if_opackets++;
				ifp->if_obytes += yp->tx_mbuf[entry]->m_len;
				/* Free the original mbuf chain */
				m_freem(yp->tx_mbuf[entry]);
				yp->tx_mbuf[entry] = 0;
				yp->tx_free++;
			}
			while (yp->tx_free) {
				IF_DEQUEUE(&ifp->if_snd,m);
				if (m) {
					yellowfin_sendpacket(m, yp);
					boguscnt--;
				} else
					break; /* nothing to send */
			}
			/* we leave this loop
			 *  - either because there's no free buffer
			 *    (m=0 initializer && !yp->tx_free)
			 *  - or there's nothing to send (IF_DEQUEUE
			 *    returned 0
			 */
		} while (m && boguscnt>=0);
#else
#error "TODO: RTEMS not implemented"
		if (intr_status & IntrTxDone
			|| yp->tx_tail_desc->tx_errs) {
			unsigned dirty_tx = yp->dirty_tx;

			for (dirty_tx = yp->dirty_tx; yp->cur_tx - dirty_tx > 0;
				 dirty_tx++) {
				/* Todo: optimize this. */
				int entry = dirty_tx % TX_RING_SIZE;
				u16 tx_errs = yp->tx_status[entry].tx_errs;

#ifndef final_version
				if (yellowfin_debug > 5)
					printk(KERN_DEBUG "%s: Tx queue %d check, Tx status "
						   "%4.4x %4.4x %4.4x %4.4x.\n",
						   dev->name, entry,
						   yp->tx_status[entry].tx_cnt,
						   yp->tx_status[entry].tx_errs,
						   yp->tx_status[entry].total_tx_cnt,
						   yp->tx_status[entry].paused);
#endif
				if (tx_errs == 0)
					break;			/* It still hasn't been Txed */
				if (tx_errs & 0xF810) {
					/* There was an major error, log it. */
#ifndef final_version
					if (yellowfin_debug > 1)
						printk(KERN_DEBUG "%s: Transmit error, Tx status %4.4x.\n",
							   dev->name, tx_errs);
#endif
					yp->stats.tx_errors++;
					if (tx_errs & 0xF800) yp->stats.tx_aborted_errors++;
					if (tx_errs & 0x0800) yp->stats.tx_carrier_errors++;
					if (tx_errs & 0x2000) yp->stats.tx_window_errors++;
					if (tx_errs & 0x8000) yp->stats.tx_fifo_errors++;
#ifdef ETHER_STATS
					if (tx_errs & 0x1000) yp->stats.collisions16++;
#endif
				} else {
#ifndef final_version
					if (yellowfin_debug > 4)
						printk(KERN_DEBUG "%s: Normal transmit, Tx status %4.4x.\n",
							   dev->name, tx_errs);
#endif
#ifdef ETHER_STATS
					if (tx_errs & 0x0400) yp->stats.tx_deferred++;
#endif
#if LINUX_VERSION_CODE > 0x20127
					yp->stats.tx_bytes += yp->tx_skbuff[entry]->len;
#endif
					yp->stats.collisions += tx_errs & 15;
					yp->stats.tx_packets++;
				}
				/* Free the original skb. */
				dev_free_skb_irq(yp->tx_skbuff[entry]);
				yp->tx_skbuff[entry] = 0;
				/* Mark status as empty. */
				yp->tx_status[entry].tx_errs = 0;
			}

#ifndef final_version
			if (yp->cur_tx - dirty_tx > TX_RING_SIZE) {
				printk("%s: Out-of-sync dirty pointer, %d vs. %d, full=%d.\n",
					   dev->name, dirty_tx, yp->cur_tx, yp->tx_full);
				dirty_tx += TX_RING_SIZE;
			}
#endif

			if (yp->tx_full
				&& yp->cur_tx - dirty_tx < TX_QUEUE_SIZE - 2) {
				/* The ring is no longer full, clear tbusy. */
				yp->tx_full = 0;
				netif_resume_tx_queue(dev);
			}

			yp->dirty_tx = dirty_tx;
			yp->tx_tail_desc = &yp->tx_status[dirty_tx % TX_RING_SIZE];
		}
#endif
		ifp->if_flags &= ~IFF_OACTIVE;

		/* wrap around the ring indices */
		if (yp->dirty_tx >= TX_RING_SIZE) {
				yp->dirty_tx -= TX_RING_SIZE;
				yp->cur_tx -= TX_RING_SIZE;
		}

		/* Log errors and other uncommon events. */
		if (ERR_EVENT & events) {
			/* Abnormal error summary. */
			yellowfin_error(yp);
		}

		if (boguscnt < 0) {
			printk("Warning %s%d: Too much work for daemon\n",
				   ifp->if_name, ifp->if_unit);
			rtems_task_wake_after(10); /* TODO: chose time */
		}
	}
	ifp->if_flags &= ~(IFF_RUNNING|IFF_OACTIVE);

	/* shut down the hardware */
	yellowfin_stop_hw(yp);
	/* flush the output queue */
	for (;;) {
			IF_DEQUEUE(&ifp->if_snd,m);
			if (!m) 
					break;
			m_freem(m);
	}
	/* as of 'rtems_bsdnet_event_receive()' we own the
	 * networking semaphore
	 */
	rtems_bsdnet_semaphore_release();
	rtems_semaphore_release(yp->daemonSync);
	if (yellowfin_debug > 1)
		printk("Yellowfin daemon exiting...\n");
	/* Note that I dont use yp->daemonTid here - 
	 * theoretically, that variable could already
	 * hold a newly created TID
	 */
	rtems_task_delete(RTEMS_SELF);
}

/* T.Straumann:
 * TODO: maybe, the algorithm should be redesigned to something
 *       like this:
 *          while (yp->rx_head_desc->result_status) {
 *                   ether_input_and_statistics();
 *                   alloc_new_buffer();
 *                   wake_up_rx_engine();
 *                   move_head_desc();
 *          }
 *
 *       instead of
 *
 *          while (yp->rx_head_desc->result_status) {
 *          		ether_input_and_statistics(packet);
 *          		move_head_desc();
 *          }
 *          refill_all_empty_descs();
 *          wake_up_rx_engine();
 *
 * RETURNS: number of packets received;
 */

static int yellowfin_rx(struct yellowfin_private *yp)
{
	int entry = yp->cur_rx % RX_RING_SIZE;
	int nloops = 0;
	int boguscnt = yp->dirty_rx + RX_RING_SIZE - yp->cur_rx;
	struct ifnet *ifp = & yp->arpcom.ac_if;

	if (yellowfin_debug > 4) {
		printk(" In yellowfin_rx(), entry %d status 0x%08x.\n",
			   entry, yp->rx_ring[entry].result_status);
		printk("   #%d desc. 0x%08x 0x%08x 0x%08x.\n",
			   entry, yp->rx_ring[entry].dbdma_cmd, yp->rx_ring[entry].addr,
			   yp->rx_ring[entry].result_status);
	}

	/* If EOP is set on the next entry, it's a new packet. Send it up. */
	while (yp->rx_head_desc->result_status) {
		struct yellowfin_desc *desc = yp->rx_head_desc;
		u16 desc_status = le32_to_cpu(desc->result_status) >> 16;
		int data_size =
			(le32_to_cpu(desc->dbdma_cmd) - le32_to_cpu(desc->result_status))
			& 0xffff;
		u8 *buf_addr = le32desc_to_virt(desc->addr);
		s16 frame_status = get_unaligned((s16*)&(buf_addr[data_size - 2]));
		nloops++;

		if (yellowfin_debug > 4)
			printk("  yellowfin_rx() status was 0x%04x.\n",
				   frame_status);
		if (--boguscnt < 0)
			break;
		if ( ! (desc_status & RX_EOP)) {
			printk("%s%d: Oversized Ethernet frame spanned multiple buffers,"
				   " status 0x%04x!\n", ifp->if_name, ifp->if_unit, desc_status);
			yp->stats.length_errors++;
		} else if ((yp->drv_flags & IsGigabit)  &&  (frame_status & 0x0038)) {
			/* There was a error. */
			if (yellowfin_debug > 3)
				printk("  yellowfin_rx() Rx error was 0x%04x.\n",
					   frame_status);
			ifp->if_ierrors++;
			if (frame_status & 0x0060) yp->stats.length_errors++;
			if (frame_status & 0x0008) yp->stats.frame_errors++;
			if (frame_status & 0x0010) yp->stats.crc_errors++;
			if (frame_status < 0) ifp->if_iqdrops++;
		} else if ( !(yp->drv_flags & IsGigabit)  &&
				   ((buf_addr[data_size-1] & 0x85) || buf_addr[data_size-2] & 0xC0)) {
			u8 status1 = buf_addr[data_size-2];
			u8 status2 = buf_addr[data_size-1];
			ifp->if_ierrors++;
			if (status1 & 0xC0) yp->stats.length_errors++;
			if (status2 & 0x03) yp->stats.frame_errors++;
			if (status2 & 0x04) yp->stats.crc_errors++;
			if (status2 & 0x80) ifp->if_iqdrops++;
#ifdef YF_PROTOTYPE			/* Support for prototype hardware errata. */
		} else if ((yp->flags & HasMACAddrBug)  &&
				   memcmp(le32desc_to_virt(yp->rx_ring[entry].addr),
						  dev->dev_addr, 6) != 0
				   && memcmp(le32desc_to_virt(yp->rx_ring[entry].addr),
							 "\377\377\377\377\377\377", 6) != 0) {
			if (bogus_rx++ == 0)
				printk("%s%d: Bad frame to %2.2x:%2.2x:%2.2x:%2.2x:"
					   "%2.2x:%2.2x.\n",
					   ifp->if_name, ifp->if_unit,
					   buf_addr[0], buf_addr[1], buf_addr[2],
					   buf_addr[3], buf_addr[4], buf_addr[5]);
#endif
		} else {
			struct mbuf *m;
			int pkt_len = data_size -
				(yp->chip_id ? 7 : 8 + buf_addr[data_size - 8]);
			/* To verify: Yellowfin Length should omit the CRC! */

#ifndef final_version
			if (yellowfin_debug > 4)
				printk("  yellowfin_rx() normal Rx pkt length %d"
					   " of %d, bogus_cnt %d.\n",
					   pkt_len, data_size, boguscnt);
#endif
			/* Check if the packet is long enough to just pass up the mbuf
			   without copying to a properly sized mbuf. */
			if (pkt_len > rx_copybreak) {
				m = yp->rx_mbuf[entry];
				yp->rx_mbuf[entry] = NULL;
#ifndef final_version				/* Remove after testing. */
				if (le32desc_to_virt(yp->rx_ring[entry].addr) != mtod(m,void*))
					printk("%s: Internal fault: The skbuff addresses "
						   "do not match in yellowfin_rx: %p vs. %p.\n",
						   ifp->if_name,
						   le32desc_to_virt(yp->rx_ring[entry].addr),
						   mtod(m,void*));
#endif
			} else {
	        		MGETHDR(m, M_WAIT, MT_DATA);
				if (pkt_len >= MINCLSIZE) {
					MCLGET(m, M_WAIT);
				}
				m->m_pkthdr.rcvif = ifp;
#ifdef TSILL_TODO
				skb_reserve(skb, 2);	/* 16 byte align the IP header */
#if HAS_IP_COPYSUM
				eth_copy_and_sum(skb, yp->rx_skbuff[entry]->tail, pkt_len, 0);
				skb_put(skb, pkt_len);
#else
				memcpy(skb_put(skb, pkt_len), yp->rx_skbuff[entry]->tail,
					   pkt_len);
#endif
#endif
				memcpy(mtod(m,char*), mtod(yp->rx_mbuf[entry],char*), pkt_len);
			}
			/* fill in length and pass up */
			m->m_len = m->m_pkthdr.len = pkt_len - sizeof(struct ether_header);
			{
				struct ether_header *eh = mtod(m, struct ether_header*);
				m->m_data += sizeof(*eh);
				ether_input(ifp,eh,m);
			}
#ifdef TSILL_TODO
			skb->protocol = eth_type_trans(skb, dev);
			netif_rx(skb);
			dev->last_rx = jiffies;
			yp->stats.rx_packets++;
#if LINUX_VERSION_CODE > 0x20127
			yp->stats.rx_bytes += pkt_len;
#endif
#endif
			ifp->if_ipackets++;
			ifp->if_ibytes+=pkt_len;
		}
		entry = (++yp->cur_rx) % RX_RING_SIZE;
		yp->rx_head_desc = &yp->rx_ring[entry];
	}

	/* Refill the Rx ring buffers. */
	for (; yp->cur_rx - yp->dirty_rx > 0; yp->dirty_rx++) {
		entry = yp->dirty_rx % RX_RING_SIZE;
		if (yp->rx_mbuf[entry] == NULL) {
			struct mbuf *m;
	        	MGETHDR(m, M_WAIT, MT_DATA);
			MCLGET(m, M_WAIT);
			yp->rx_mbuf[entry] = m;
			m->m_pkthdr.rcvif = ifp; /* Mark as being used by this device. */
#ifdef TSILL_TODO
			skb_reserve(skb, 2);	 /* Align IP on 16 byte boundaries */
#endif
			st_le32((volatile unsigned32 *)
				&yp->rx_ring[entry].addr,
		       		(unsigned long)virt_to_bus(mtod(m, void*)));
		}
		
		/* TODO: T.S. machines with a non-snooping copyback cache
		 *       enabled on the RX buffers should flush/invalidate
		 *       the cache lines overlapping the buffer here.
		 *       (flush lines which overlap with adjacent memory,
	 	 *       invalidate others)
		 *       Actually, this should be optimized: reallocation
		 *       of buffers and cache invalidation should be handled
		 *       above since copying and invalidating small buffers
		 *       can be done more efficiently in one single step...
		 * Yet, I leave it here for the moment, so I can re-use
		 * Don's code.
		 */
		st_le32((volatile unsigned32 *)
			&yp->rx_ring[entry].dbdma_cmd,
			CMD_STOP);
		yp->rx_ring[entry].result_status = 0;	/* Clear complete bit. */
		if (entry != 0)
			st_le32((volatile unsigned32 *)
				&yp->rx_ring[entry - 1].dbdma_cmd,
				(CMD_RX_BUF | INTR_ALWAYS | yp->rx_buf_sz));
		else
			st_le32((volatile unsigned32 *)
				&yp->rx_ring[RX_RING_SIZE - 1].dbdma_cmd,
				(CMD_RX_BUF | INTR_ALWAYS | BRANCH_ALWAYS
							| yp->rx_buf_sz));
	}
	/* wrap around ring indices */
	if (yp->dirty_rx >= RX_RING_SIZE) {
			yp->dirty_rx -= RX_RING_SIZE;
			yp->cur_rx -= RX_RING_SIZE;
			/* both should be 0 now */
	}

	return nloops;
}

static void yellowfin_error(struct yellowfin_private *yp)
{
	struct ifnet			*ifp = &yp->arpcom.ac_if;
	unsigned long			intr_status;
	rtems_interrupt_level	l;

	/* read and reset the status; because this is written
	 * by the ISR, we must disable interrupts here
	 */
	rtems_interrupt_disable(l);
		intr_status = yp->intr_status;
		yp->intr_status=0;
	rtems_interrupt_enable(l);

	if (intr_status) {
			printk("%s%d: Something Wicked happened! 0x%04x.\n",
							ifp->if_name, ifp->if_unit, intr_status);
			/* Hmmmmm, it's not clear what to do here. */
			if (intr_status & (IntrTxPCIErr | IntrTxPCIFault))
					ifp->if_oerrors++;
			if (intr_status & (IntrRxPCIErr | IntrRxPCIFault))
					ifp->if_ierrors++;
	} else {
			printk("%s%d: Hmmm, got interrupt but nothing to do?\n",
					ifp->if_name, ifp->if_unit);
	}
}

static void yellowfin_start(struct ifnet * ifp)
{
	struct yellowfin_private *yp=ifp->if_softc;
	ifp->if_flags |= IFF_OACTIVE;
	rtems_event_send (yp->daemonTid, RESTART_EVENT);
}

static void yellowfin_stop(struct yellowfin_private *yp)
{
	/* kill the daemon. We also must release the networking
	 * semaphore or there'll be a deadlock...
	 */
	rtems_event_send(yp->daemonTid, KILL_EVENT);
	rtems_bsdnet_semaphore_release();

	yp->daemonTid=0;
	/* now wait for it to die */
	rtems_semaphore_obtain(yp->daemonSync,RTEMS_WAIT,RTEMS_NO_TIMEOUT);

	/* reacquire the bsdnet sema */
	rtems_bsdnet_semaphore_obtain();
}


static int yellowfin_stop_hw(struct  yellowfin_private *yp)
{
	struct ifnet	*ifp = &yp->arpcom.ac_if;
	long		ioaddr = yp->base_addr;
	int		i;

	if (yellowfin_debug > 1)
		printk("Yellowfin: entering stop_hw...\n");

	/* remove our interrupt handler which will also
	 * disable interrupts at the MPIC and the device
	 * itself
	 */
	if (!BSP_remove_rtems_irq_handler(&yellowfin_irq_info))
		rtems_panic("Yellowfin: unable to remove IRQ handler!");

#if 0	/* done already by remove_rtems_irq_handler */
	/* Disable interrupts by clearing the interrupt mask. */
	outw(0x0000, ioaddr + IntrEnb);
#endif

	/* Stop the chip's Tx and Rx processes. */
	outl(0x80000000, ioaddr + RxCtrl);
	outl(0x80000000, ioaddr + TxCtrl);

	if (yellowfin_debug > 1) {
		printk("%s%d: Shutting down ethercard, status was Tx 0x%04x "
			   "Rx 0x%04x Int 0x%02x.\n",
			   ifp->if_name, ifp->if_unit,
			   inw(ioaddr + TxStatus),
			   inw(ioaddr + RxStatus), inw(ioaddr + IntrStatus));
		printk("%s%d: Queue pointers were Tx %d / %d,  Rx %d / %d.\n",
			   ifp->if_name, ifp->if_unit,
			   yp->cur_tx, yp->dirty_tx, yp->cur_rx, yp->dirty_rx);
	}

	/* stop the timer, just in case */
	ifp->if_timer = 0;

#if defined(__i386__)
#error "TODO: i386 support not implemented"
	if (yellowfin_debug > 2) {
		printk("\n"KERN_DEBUG"  Tx ring at %8.8x:\n",
			   (int)virt_to_bus(yp->tx_ring));
		for (i = 0; i < TX_RING_SIZE*2; i++)
			printk(" %c #%d desc. %8.8x %8.8x %8.8x %8.8x.\n",
				   inl(ioaddr + TxPtr) == (long)&yp->tx_ring[i] ? '>' : ' ',
				   i, yp->tx_ring[i].dbdma_cmd, yp->tx_ring[i].addr,
				   yp->tx_ring[i].branch_addr, yp->tx_ring[i].result_status);
		printk(KERN_DEBUG "  Tx status %p:\n", yp->tx_status);
		for (i = 0; i < TX_RING_SIZE; i++)
			printk("   #%d status %4.4x %4.4x %4.4x %4.4x.\n",
				   i, yp->tx_status[i].tx_cnt, yp->tx_status[i].tx_errs,
				   yp->tx_status[i].total_tx_cnt, yp->tx_status[i].paused);

		printk("\n"KERN_DEBUG "  Rx ring %8.8x:\n",
			   (int)virt_to_bus(yp->rx_ring));
		for (i = 0; i < RX_RING_SIZE; i++) {
			printk(KERN_DEBUG " %c #%d desc. %8.8x %8.8x %8.8x\n",
				   inl(ioaddr + RxPtr) == (long)&yp->rx_ring[i] ? '>' : ' ',
				   i, yp->rx_ring[i].dbdma_cmd, yp->rx_ring[i].addr,
				   yp->rx_ring[i].result_status);
			if (yellowfin_debug > 6) {
				if (get_unaligned((u8*)yp->rx_ring[i].addr) != 0x69) {
					int j;
					for (j = 0; j < 0x50; j++)
						printk(" %4.4x",
							   get_unaligned(((u16*)yp->rx_ring[i].addr) + j));
					printk("\n");
				}
			}
		}
	}
#endif /* __i386__ debugging only */

	/* Free all the skbuffs in the Rx queue. */
	for (i = 0; i < RX_RING_SIZE; i++) {
		st_le32((volatile unsigned32 *)
			&yp->rx_ring[i].dbdma_cmd,
			CMD_STOP);
		yp->rx_ring[i].addr = 0xBADF00D0; /* An invalid address. */
		if (yp->rx_mbuf[i]) {
			m_freem(yp->rx_mbuf[i]);
			yp->rx_mbuf[i] = 0;
		}
	}
	for (i = 0; i < TX_RING_SIZE; i++) {
		if (yp->tx_mbuf[i])
			m_freem(yp->rx_mbuf[i]);
		yp->tx_mbuf[i] = 0;
	}

#ifdef YF_PROTOTYPE			/* Support for prototype hardware errata. */
	if (yellowfin_debug > 0) {
		printk("%s%d: Received %d frames that we should not have.\n",
			   ifp->if_name, ifp->if_unit, bogus_rx);
	}
#endif

	return 0;
}


/* Set or clear the multicast filter for this adaptor. */

/* The little-endian AUTODIN32 ethernet CRC calculation.
   N.B. Do not use for bulk data, use a table-based routine instead.
   This is common code and should be moved to net/core/crc.c */
static unsigned const ethernet_polynomial_le = 0xedb88320U;

static inline unsigned ether_crc_le(int length, unsigned char *data)
{
	unsigned int crc = 0xffffffff;	/* Initial value. */
	while(--length >= 0) {
		unsigned char current_octet = *data++;
		int bit;
		for (bit = 8; --bit >= 0; current_octet >>= 1) {
			if ((crc ^ current_octet) & 1) {
				crc >>= 1;
				crc ^= ethernet_polynomial_le;
			} else
				crc >>= 1;
		}
	}
	return crc;
}

static inline void
set_bit(int n, void *d)
{
unsigned char *ptr=d;
		*(ptr+(n>>3)) |= 1<<(n&7);
}


static int set_rx_mode(struct yellowfin_private *yp)
{
	int rval=0;
	struct arpcom *ac = &yp->arpcom;
	struct ifnet  *ifp = &ac->ac_if;
	long ioaddr = yp->base_addr;
	u16 cfg_value = inw(ioaddr + Cnfg);

	/* Stop the Rx process to change any value. */
	outw(cfg_value & ~0x1000, ioaddr + Cnfg);
	if (ifp->if_flags & IFF_PROMISC) {			/* Set promiscuous. */
		if (yellowfin_debug>0)
			printk("%s: Promiscuous mode enabled.\n", ifp->if_name);
		outw(0x000F, ioaddr + AddrMode);
	} else if ((ac->ac_multicnt > 64)  ||  (ifp->if_flags & IFF_ALLMULTI)) {
		/* Too many to filter well, or accept all multicasts. */
		outw(0x000B, ioaddr + AddrMode);
	} else if (ac->ac_multicnt > 0) { /* Must use the multicast hash table. */
		struct ether_multi *enm;
		u16 hash_table[4];
		int i;
		memset(hash_table, 0, sizeof(hash_table));
		/* TODO: counting i seems to be redundant with enm list traversal
		 *       (remember: this code originated from linux/skb...)
		 */
		for (i = 0, enm = ac->ac_multiaddrs; enm && i < ac->ac_multicnt;
			 i++, enm = enm->enm_next) {
			unsigned char *enaddr=enm->enm_addrlo;
			/* HMMM, how to deal with address ranges??? */
			if (memcmp(enaddr, enm->enm_addrhi, ETHER_ADDR_LEN)) {
					rval=EAFNOSUPPORT;
					printk("WARNING: Yellowfin: multicast address ranges not supported\n");
			}
			/* Due to a bug in the early chip versions, multiple filter
			   slots must be set for each address. */

			/* NOTE: the original (linux) set_bit is probably _not_
			 *       endian safe!
			 */
			if (yp->drv_flags & HasMulticastBug) {
				set_bit((ether_crc_le(3, enaddr) >> 3) & 0x3f,
						hash_table);
				set_bit((ether_crc_le(4, enaddr) >> 3) & 0x3f,
						hash_table);
				set_bit((ether_crc_le(5, enaddr) >> 3) & 0x3f,
						hash_table);
			}
			set_bit((ether_crc_le(6, enaddr) >> 3) & 0x3f,
					hash_table);
		}
		/* Copy the hash table to the chip. */
		for (i = 0; i < 4; i++)
			outw(hash_table[i], ioaddr + HashTbl + i*2);

		/* TODO: remove this, once this feature has been tested */
		printk("Yellowfin: WARNING: setting multicast hash table has not been tested yet\n");
		outw(0x0003, ioaddr + AddrMode);
	} else {					/* Normal, unicast/broadcast-only mode. */
		outw(0x0001, ioaddr + AddrMode);
	}
	/* update the cached copy */
	yp->old_iff = ifp->if_flags;
	/* Restart the Rx process. */
	outw(cfg_value | 0x1000, ioaddr + Cnfg);
	return rval;
}


/* TODO: the SIOxMIIxx constants are not defined yet */
#if defined(SIOCGMIIPHY) && defined(SIOCGMIIREG) && defined(SIOCSMIIREG)
#define USE_MII_IOCTLS
#warning "YELLOWFIN: SIOCxMIIxx seem to be defined, cleanup yellowfin.c..."
#endif

static int yellowfin_ioctl(struct ifnet *ifp, int cmd, caddr_t arg)
{
	int		rval=0;
	struct yellowfin_private *np = ifp->if_softc;
#ifdef USE_MII_IOCTLS
	long ioaddr = np->base_addr;
#endif
	struct ifreq *ifr=(struct ifreq*)arg;

	if (yellowfin_debug>1)
		printk("Yellowfin: entering ioctl...\n");

	switch(cmd) {
	default:
		if (yellowfin_debug>1)
			printk("Yellowfin: etherioctl...\n");
		/* handles SIOxIFADDR, SIOCSIFMTU */
		return ether_ioctl(ifp, cmd, arg);

#ifdef USE_MII_IOCTLS
	case SIOCGMIIPHY:		/* Get the address of the PHY in use. */
		data[0] = np->phys[0] & 0x1f;
		/* Fall Through */
	case SIOCGMIIREG:		/* Read the specified MII register. */
		data[3] = mdio_read(ioaddr, data[0] & 0x1f, data[1] & 0x1f);
		return 0;
	case SIOCSMIIREG:		/* Write the specified MII register */
		if (data[0] == np->phys[0]) {
			u16 value = data[2];
			switch (data[1]) {
			case 0:
				/* Check for autonegotiation on or reset. */
				np->medialock = (value & 0x9000) ? 0 : 1;
				if (np->medialock)
					np->full_duplex = (value & 0x0100) ? 1 : 0;
				break;
			case 4: np->advertising = value; break;
			}
			/* Perhaps check_duplex(dev), depending on chip semantics. */
		}
		mdio_write(ioaddr, data[0] & 0x1f, data[1] & 0x1f, data[2]);
		return 0;
#endif

	case SIOCSIFFLAGS:
		switch (ifp->if_flags & (IFF_UP | IFF_RUNNING)) {
		case IFF_RUNNING:
			yellowfin_stop(np);
			break;

		case IFF_UP:
			yellowfin_init(np);
			break;

#if 0
		/* I prefer to do nothing in this case.
		 * if they want to restart the interface, 
		 * they should bring it down first and then
		 * bring it up again.
		 */
		case IFF_UP | IFF_RUNNING:
			yellowfin_stop(np);
			yellowfin_init(np);
			break;
#endif

		default:
			break;
		}

		if ((ifp->if_flags & IFF_PROMISC) != (np->old_iff & IFF_PROMISC))
				set_rx_mode(np);
		break;

	case SIOCADDMULTI:
		if (ENETRESET == (rval=ether_addmulti(ifr, &np->arpcom))) {
				set_rx_mode(np);
				rval=0;
		}
		break;

	case SIOCDELMULTI:
		if (ENETRESET == (rval=ether_delmulti(ifr, &np->arpcom))) {
				set_rx_mode(np);
				rval=0;
		}
		break;

	case SIO_RTEMS_SHOW_STATS:
		yellowfin_stats (np);
		break;

	}
	if (yellowfin_debug>1)
		printk("Yellowfin: leaving ioctl...\n");
	return rval;
}

static void yellowfin_stats(struct yellowfin_private *yp)
{
		printf("      Rx Interrupts:%-8lu\n", yp->stats.nRxIrqs);
		printf("     Framing Errors:%-8lu",   yp->stats.frame_errors);
		printf("         Crc Errors:%-8lu",   yp->stats.crc_errors);
		printf("   Oversized Frames:%-8lu\n", yp->stats.length_errors);

		printf("      Tx Interrupts:%-8lu\n", yp->stats.nTxIrqs);
		printf(" Reassembled Frames:%-8lu\n", yp->stats.txReassembled);
}


/*
 * Local variables:
 *  compile-command: "gcc -DMODULE -Wall -Wstrict-prototypes -O6 -c yellowfin.c"
 *  compile-command-alphaLX: "gcc -DMODULE -Wall -Wstrict-prototypes -O2 -c yellowfin.c -fomit-frame-pointer -fno-strength-reduce -mno-fp-regs -Wa,-m21164a -DBWX_USABLE -DBWIO_ENABLED"
 *  simple-compile-command: "gcc -DMODULE -O6 -c yellowfin.c"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */

