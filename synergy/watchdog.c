/* $Id$ */

/* Support for the watchdog timer present on some
 * Synergy VGM boards
 *
 * Author: Till Straumann <strauman@slac.stanford.edu>
 *         3/2002
 */

/* This routine arms the watchdog
 * NOTE: supplying 0 usecs disables the WD.
 */

#include <bsp/bspVGM.h>
#include "synergyregs.h"
#include <bsp/openpic.h>
#include <libcpu/io.h>

/* On VGM5, cpu3 is wired to the system reset line */
#define VGM_WD_FAKE_CPU		3
/* VGM uses the 3rd timer as a watchdog */
#define VGM_WD_PIC_TIMER	3

long
BSP_watchdogArm(unsigned long usecs)
{	
unsigned long count,val;
unsigned char boardRev;

	/* check the board revision */
	boardRev=*SYN_VGM_REG_INFO_BOARD_REV;
	switch ( SYN_VGM_REG_INFO_BOARD_TYPE_MSK & boardRev ) {
		case SYN_VGM_REG_INFO_BOARD_TYPE_VGMD:
			if ((SYN_VGM_REG_INFO_BOARD_REV_MSK & boardRev) >= 0x2)
				break; /* Revision C or newer */
			return -1; /* older has no watchdog */

		case SYN_VGM_REG_INFO_BOARD_TYPE_VGM5:
			if ((SYN_VGM_REG_INFO_BOARD_REV_MSK & boardRev) >= 0x4)
				break; /* Revision E or newer */
			/* older, fall thru */
		default:
			return -1; /* feature not implemented */
	}
	/* make sure timer IRQ source is disabled */
	openpic_inittimer(VGM_WD_PIC_TIMER, 0x0, 0);
	
	/* make sure timer is stopped */
	out_le32(&OpenPIC->Global.Timer[VGM_WD_PIC_TIMER].Base_Count,
			OPENPIC_TIMER_COUNT_INHIBIT);

	/* disable WD interrupt on dest. CPU */
	openpic_set_priority(VGM_WD_FAKE_CPU,0xf);

	if (usecs) {
		/* use highest priority and a dummy vector */
		openpic_inittimer(VGM_WD_PIC_TIMER, 0xf, 7);
		openpic_maptimer(VGM_WD_PIC_TIMER, 1<<VGM_WD_FAKE_CPU);
		/* compute count from usecs using integer arithmetics;
		 * algorithm from the VGM5 manual...
		 */
		count = usecs*33/8 + usecs*3/80 + usecs*3/800 + usecs*3/8000;
		/* enable IRQ on destination CPU */
		openpic_set_priority(VGM_WD_FAKE_CPU,0);
		/* enable timer IRQ source */
		while ( (val=in_le32(&OpenPIC->Global.Timer[VGM_WD_PIC_TIMER].Vector_Priority)) & OPENPIC_ACTIVITY )
			/* wait */;
		out_le32(&OpenPIC->Global.Timer[VGM_WD_PIC_TIMER].Vector_Priority,
				 val & ~OPENPIC_MASK);
		
		/* go and arm the timer */
		out_le32(&OpenPIC->Global.Timer[VGM_WD_PIC_TIMER].Base_Count,
			count);
	}
	return 0;
}

void
BSP_watchdogPet(void)
{
unsigned long count = in_le32(&OpenPIC->Global.Timer[VGM_WD_PIC_TIMER].Base_Count);
	out_le32(&OpenPIC->Global.Timer[VGM_WD_PIC_TIMER].Base_Count,
			OPENPIC_TIMER_COUNT_INHIBIT);
	out_le32(&OpenPIC->Global.Timer[VGM_WD_PIC_TIMER].Base_Count,
			count);
}
