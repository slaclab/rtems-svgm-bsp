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
#include <rtems/console.h>
#include <rtems/clockdrv.h>
#include <bsp/vectors.h>

#include <libcpu/io.h>

/* fundamental addresses for this BSP (PREPxxx are from libcpu/io.h) */
#define _IO_BASE			CHRP_ISA_IO_BASE
#define	_ISA_MEM_BASE		CHRP_ISA_MEM_BASE
/* start of our ram seen from the PCI bus */
#define PCI_DRAM_OFFSET		CHRP_PCI_DRAM_OFFSET
/* SMON has no ISA bus and configures PCI devices' memory space with respect
 * to 0x00000000 (_not_ as an offset from CHRP_ISA_MEM_BASE)
 *
 * PCI memory space as seen from the CPU
 */
#define PCI_MEM_BASE		0x00000000
#define PCI_MEM_BASE_ADJUSTMENT	0x00000000

/*
 *  confdefs.h overrides for this BSP:
 *   - termios serial ports (defaults to 1)
 *   - Interrupt stack space is not minimum if defined.
 */

#define CONFIGURE_NUMBER_OF_TERMIOS_PORTS 2
#define BSP_INTERRUPT_STACK_SIZE          (16 * 1024)

#define BSP_NVRAM_BOOTPARMS_START	(0xffe9f000)
#define BSP_NVRAM_BOOTPARMS_END		(0xffe9f4ff)

/* private definitions, not visible to apps */
#define BSP_UART_IOBASE_COM1	(0xffeffb08)
#define BSP_UART_IOBASE_COM2	(0xffeffb00)

#define BSP_CONSOLE_PORT		BSP_UART_COM1
#define BSP_UART_BAUD_BASE		115200

#include <bsp/openpic.h>

/* BSP_PIC_DO_EOI is optionally used by the 'vmeUniverse' driver
 * to implement VME IRQ priorities in software.
 * Note that this requires support by the interrupt controller
 * driver (cf. libbsp/shared/powerpc/irq/openpic_i8259_irq.c)
 * and the BSP-specific universe initialization/configuration
 * (cf. libbsp/shared/powerpc/vme/VMEConfig.h vme_universe.c)
 *
 * ********* IMPORTANT NOTE ********
 * When deriving from this file (new BSPs)
 * DO NOT define "BSP_PIC_DO_EOI" if you don't know what
 * you are doing i.e., w/o implementing the required pieces
 * mentioned above.
 * ********* IMPORTANT NOTE ********
 */

#define BSP_PIC_DO_EOI		openpic_eoi(0)

/* Use a task notepad to attach user exception handler info;
 * may be changed by application startup code (EPICS uses 11)
 */
#define BSP_EXCEPTION_NOTEPAD		14
 
#ifndef ASM
/* drivers should not use these anyway */
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
 * Start of the heap
 */
extern unsigned int BSP_heap_start;
/*
 * PPC Bus Frequency
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

/*
 * The commandline as passed from the bootloader.
 * (Note that this is the same string available
 * at rtems_bdnet_bootp_cmdline if networking
 * is linked)
 */
extern char *BSP_commandline_string;

#define BSP_Convert_decrementer( _value ) \
  ((unsigned long long) ((((unsigned long long)BSP_time_base_divisor) * 1000000ULL) /((unsigned long long) BSP_bus_frequency)) * ((unsigned long long) (_value)))

extern rtems_configuration_table  BSP_Configuration;
extern void BSP_panic(char *s);
extern void bsp_reset(void);
extern int BSP_disconnect_clock_handler (void);
extern int BSP_connect_clock_handler (void);

/* clear hostbridge errors
 *
 * clear all error flags adhering to the algorithm
 * recommended by motorola - MPC106 user's manual
 * sect. 9.3.3.3
 *
 * enableMCP: whether to enable MCP checkstop / machine check interrupts
 *            on the hostbridge and in HID0.
 *
 *            NOTE: HID0 and MEREN are left alone if this flag is 0
 * 
 * quiet    : be silent
 *
 * Returns: (for diagnostic purposes)
 *          original settings (i.e. before applying the clearing
 *          sequence) of
 *          (errdr2<<24) | (errdr1<<16) | pci_status
 */

extern unsigned long _BSP_clear_hostbridge_errors(int enableMCP, int quiet);



extern char *BSP_build_date;

/* this is misleading, it's actually the first interface's name */
#define RTEMS_BSP_NETWORK_DRIVER_NAME	"es1"
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
