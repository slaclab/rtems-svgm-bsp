#ifndef RTEMS_SVGM_BSP_VME_H
#define RTEMS_SVGM_BSP_VME_H
/* $Id$ */

/* SVGM BSP's VME support */
/* Author: Till Straumann, <strauman@slac.stanford.edu> */

/* pull in bsp.h */
#include <bsp.h>
/* our VME bridge */
#include <bsp/vmeUniverse.h>

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

#define BSP_vme_init() \
	vmeUniverseInit

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

#endif
