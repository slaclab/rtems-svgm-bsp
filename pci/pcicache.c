/* $Id$ */

/* Maintain a cache of PCI device/vendor bus scan.
 * Rationale: config read access to an empty slot
 *            triggers a master abort. Since we want
 *            to run the VGM board in 'paranoid' mode
 *            with MCP enabled on hostbridge errors,
 *            this would be fatal.
 *            Therefore, we scan the bus once, at init
 *            time and later use cached info
 *
 * Author:    Till Straumann <strauman@slac.stanford.edu> 5/2002
 * License:   GPL
 */

#include <bsp.h>
#include <bsp/pci.h>

#include <stdio.h>

typedef struct PCICacheDevNodeRec_ *PCICacheDevNode;

/* structure describing a device function */
typedef struct PCICacheDevNodeRec_ {
		unsigned long	vendorid;		/* the vendor / device id */
		unsigned char	bus,dev,fun;
} PCICacheDevNodeRec;

#define	MAX_NUM_PCI_DEVICES	10

/* se have no malloc or workspace yet when this is called
 * hence we need static memory :-(
 */
static PCICacheDevNodeRec	devList[MAX_NUM_PCI_DEVICES];
static int					numPCIDevs=0;

#define PCI_INVALID_VENDORDEVICEID	(-1)
#define PCI_MULTI_FUNCTION			0x80

/* build a linked list of all devices */
void
_BSP_pciCacheInit(void)
{
int				bus,dev,fun;
PCICacheDevNode	node;
unsigned 		d;
unsigned char	hd;

	node=devList;
	for (bus=0; bus<BusCountPCI(); bus++) {
      for (dev=0; dev<PCI_MAX_DEVICES; dev++) {
		/* Is this a multi function device? */
		pci_read_config_byte(bus, dev, 0, PCI_HEADER_TYPE, &hd);
		hd = (hd&PCI_MULTI_FUNCTION ? PCI_MAX_FUNCTIONS : 1);
		for (fun=0; fun<hd; fun++) {
			/* 
			 * The last devfun id/slot is special; must skip it
			 */
			if (PCI_MAX_DEVICES-1==dev && PCI_MAX_FUNCTIONS-1 == fun)
				break;
			(void)pci_read_config_dword(bus,dev,fun,PCI_VENDOR_ID,&d);
			if (PCI_INVALID_VENDORDEVICEID == d)
				continue;
#if defined(PCI_DEBUG)
			printk("_BSP_pciCacheInit: found 0x%08x at %i/%i/%i\n",d,bus,dev,fun);
#endif
			if (++numPCIDevs > MAX_NUM_PCI_DEVICES) {
				BSP_panic("Too many PCI devices found; increase MAX_NUM_PCI_DEVICES in pcicache.c\n");
			}
			/* allocate a new node */
			memset(node,0,sizeof(*node));
			node->bus=bus;
			node->dev=dev;
			node->fun=fun;
			node->vendorid=d;
			/* append to the list */
			node++;
		 }
      }
	}
}

void
BSP_pciCacheDump(void)
{
PCICacheDevNode n;
int				i;
	printf("PCI Devices:\n");
	printf("Bus Dev Fn  Vendor   ID\n");
	for (n=devList,i=0; i<numPCIDevs; n++,i++) {
		printf(" %2i  %2i  %1i 0x%04x 0x%04x\n",
			n->bus,
			n->dev,
			n->fun,
			(unsigned)(n->vendorid & 0xffff),
			(unsigned)((n->vendorid>>16)&0xffff));
	}
}

int
BSP_pciFindDevice(unsigned short vendor, unsigned short id, int instance,
				  int *pbus, int *pdev, int *pfun)
{
PCICacheDevNode n;
int				i;
unsigned		v=((id&0xffff)<<16) | (vendor&0xffff);
	for (n=devList,i=0; i<numPCIDevs; n++,i++) {
		if (n->vendorid==v && !instance--) {
				/* found it */
				*pbus=n->bus;
				*pdev=n->dev;
				*pfun=n->fun;
				return 0;
		}
	}
	return -1;
}
