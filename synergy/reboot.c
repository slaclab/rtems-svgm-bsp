/* $Id$ */

/* VGM implementation of reboot */

#include <libchip/vmeUniverse.h>
#include <bspVGM.h>

void
rtemsReboot(void)
{
	/* try a watchdog reset (the board's response
	 * to its own VME SYSRESET may be disabled
	 * in the respective board control register
	 */
	if ( BSP_watchdogArm(10) ) {
		/* watchdog unsupported; try VME */
		vmeUniverseResetBus();
	}
}
