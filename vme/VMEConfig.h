#ifndef RTEMS_BSP_VME_CONFIG_H
#define RTEMS_BSP_VME_CONFIG_H
/* $Id$ */

/* BSP specific address space configuration parameters */

/* 
 * The BSP maps VME address ranges into
 * one BAT.
 * NOTE: the BSP (startup/bspstart.c) uses
 * hardcoded window lengths that match this
 * layout:
 */
#define _VME_A32_WIN0_ON_PCI	0xc0000000
#define _VME_A24_ON_PCI			0xcf000000
#define _VME_A16_ON_PCI			0xcfff0000

/* start of the A32 window on the VME bus
 * TODO: this should perhaps be a configuration option
 */
#define _VME_A32_WIN0_ON_VME	0x20000000

/* if _VME_DRAM_OFFSET is defined, the BSP
 * will map our RAM onto the VME bus, starting
 * at _VME_DRAM_OFFSET
 */
#define _VME_DRAM_OFFSET		0xc0000000

/* Tell the interrupt manager that the universe driver
 * already called openpic_eoi() and that this step hence
 * must be omitted.
 */
#define BSP_PCI_VME_DRIVER_DOES_EOI
/* don't reference vmeUniverse0PciIrqLine directly here - leave it up to
 * bspstart() to set BSP_vme_bridge_irq. That way, we can generate variants
 * of the BSP with / without the universe driver...
 */
extern int _BSP_vme_bridge_irq;

extern int BSP_VMEInit();
extern int BSP_VMEIrqMgrInstall();

#define BSP_VME_UNIVERSE_INSTALL_IRQ_MGR(err)	\
	do {										\
		err = vmeUniverseInstallIrqMgr(			\
			4, BSP_PCI_IRQ_LOWEST_OFFSET + 1,	\
			5, BSP_PCI_IRQ_LOWEST_OFFSET + 8);	\
	} while (0)

/* setup bat2 for VME */
#define BSP_VME_BAT_IDX	2

#endif
