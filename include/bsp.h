/*
 *  bsp.h  -- contain BSP API definition.
 *
 *  Copyright (C) 1999 Eric Valette. valette@crf.canon.fr
 *
 *  The license and distribution terms for this file may be
 *  found in found in the file LICENSE in this distribution or at
 *  http://www.OARcorp.com/rtems/license.html.
 *
 * $Id$
 */
#ifndef LIBBSP_POWERPC_SVGM_BSP_H
#define LIBBSP_POWERPC_SVGM_BSP_H

#include <rtems.h>
#include <console.h>
#include <clockdrv.h>
#include <bsp/vectors.h>

#include <libcpu/io.h>
/* these should not be defined in libcpu/io.h! */
#warning "TODO: fix _IO_BASE & friends hack..."
/* change definitions for CHRP / SVGM */
#undef _IO_BASE
#define _IO_BASE 0xfe000000
#undef _ISA_MEM_BASE
#define _ISA_MEM_BASE 0
#undef PCI_DRAM_OFFSET
#define PCI_DRAM_OFFSET 0	/* start of our ram seen from the PCI bus */

/* 
 * The BSP maps VME address ranges into
 * one BAT.
 * NOTE: the BSP (startup/bspstart.c) uses
 * hardcoded window lengths that match this
 * layout:
 */
#define _VME_A32_WIN0_ON_PCI	0xc0000000
#define _VME_A24_ON_PCI		0xcf000000
#define _VME_A16_ON_PCI		0xcfff0000

/* start of the A32 window on the VME bus
 * TODO: this should perhaps be a configuration option
 */
#define _VME_A32_WIN0_ON_VME	0x20000000

/* if _VME_DRAM_OFFSET is defined, the BSP
 * will map our RAM onto the VME bus, starting
 * at _VME_DRAM_OFFSET
 */
#define _VME_DRAM_OFFSET	0xc0000000

/* VME related declarations */
/* how to map a VME address to the CPU local bus.
 * Note that this traverses two bridges:
 * the grackle and the universe. For the
 * Universe, there is a lookup procedure while
 * we assume a 1:1 mapping for the grackle...
 */

/* NOTE about the fast mapping macros:
 * using these macros is only safe if the user app
 * does _NOT_ change the universe mappings!
 * While changing the PCI windows probably doesn't
 * make much sense (involves changing the MMU/DBATs as well),
 * The user might wish to change the VME address
 * layout, i.e. by remapping _VME_A32_WIN0_ON_VME
 * and _VME_DRAM_OFFSET...
 * Hence, using the A24 and A16 macros is probably safe.
 */

#define BSP_vme2local_adrs(am, vmeaddr, plocaladdr) \
	vmeUniverseBusToLocalAdrs(am, vmeaddr, plocaladdr)

/* when using this macro, the universe setup MUST NOT BE
 * CHANGED by the application...
 */
#define BSP_vme2local_A32_fast(vmeaddr) \
	((vmeaddr)-_VME_A32_WIN0_ON_VME + _VME_A32_WIN0_ON_PCI)
#define BSP_vme2local_A24_fast(vmeaddr) \
	(((vmeaddr)&0x7ffffff)+_VME_A24_ON_PCI)
#define BSP_vme2local_A16_fast(vmeaddr) \
	(((vmeaddr)&0xffff)+_VME_A16_ON_PCI)

/* how a CPU address is mapped to the VME bus (if at all)
 * note that we assume a 1:1 mapping of CPU : PCI bus
 * here.
 */
#define BSP_local2vme_adrs(am, localaddr, pvmeaddr) \
	vmeUniverseLocalToBusAdrs(am,localaddr,pvmeaddr)

#define BSP_localdram2vme_fast(localaddr) \
	((localaddr)+_VME_DRAM_OFFSET)

/* interrupt handlers and levels */
typedef void (*BSP_VME_ISR_t)(void *usrArg, unsigned long vector);

#define BSP_installVME_isr(vector, handler, arg) \
	vmeUniverseInstallISR(vector, handler, arg)

#define BSP_removeVME_isr(vector, handler, arg) \
	vmeUniverseRemoveISR(vector, handler, arg)

/* retrieve the currently installed ISR for a given vector */
#define BSP_getVME_isr(vector, parg) \
    vmeUniverseISRGet(vector, parg)

#define BSP_enableVME_int_lvl(level) \
	vmeUniverseIntEnable(level)

#define BSP_disableVME_int_lvl(level) \
	vmeUniverseIntDisable(level)

/*
 *  confdefs.h overrides for this BSP:
 *   - termios serial ports (defaults to 1)
 *   - Interrupt stack space is not minimum if defined.
 */

/* #define CONFIGURE_NUMBER_OF_TERMIOS_PORTS 2 */
#define CONFIGURE_INTERRUPT_STACK_MEMORY  (16 * 1024)

/* private definitions, not visible to apps */
#define BSP_UART_IOBASE_COM1	(0xffeffb08)
#define BSP_UART_IOBASE_COM2	(0xffeffb00)

#define BSP_CONSOLE_PORT	BSP_UART_COM1
#define BSP_UART_BAUD_BASE	115200

#include <bsp/openpic.h>

#define BSP_PIC_DO_EOI		openpic_eoi(0)


#ifndef ASM
#define outport_byte(port,value)
#define outport_word(port,value)
#define outport_long(port,value)

#define inport_byte(port,value)
#define inport_word(port,value)
#define inport_long(port,value)

/*
 * Total memory read from board register
 */
extern unsigned int BSP_mem_size;
/*
 * PCI Bus Frequency
 */
extern unsigned int BSP_bus_frequency;
/*
 * processor clock frequency
 */
extern unsigned int BSP_processor_frequency;
/*
 * Time base divisior (how many tick for 1 second).
 */
extern unsigned int BSP_time_base_divisor;

#define BSP_Convert_decrementer( _value ) \
  ((unsigned long long) ((((unsigned long long)BSP_time_base_divisor) * 1000000ULL) /((unsigned long long) BSP_bus_frequency)) * ((unsigned long long) (_value)))

extern rtems_configuration_table  BSP_Configuration;
extern void BSP_panic(char *s);
extern void rtemsReboot(void);
extern int BSP_disconnect_clock_handler (void);
extern int BSP_connect_clock_handler (void);

/* this is misleading, it's actually the first interface's name */
#define RTEMS_BSP_NETWORK_DRIVER_NAME	"es0"
#define RTEMS_BSP_NETWORK_DRIVER_ATTACH	rtems_yellowfin_driver_attach
/* don't use a full prototype here - it produces warnings when this file
 * is included from files that don't include networking headers...
 */
extern int
RTEMS_BSP_NETWORK_DRIVER_ATTACH(/* struct rtems_bsdnet_ifconfig * */);

/* set user LED pattern */
extern void
BSP_setLED(unsigned char val);

#endif

#endif
