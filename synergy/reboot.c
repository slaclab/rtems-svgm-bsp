/* $Id$ */

/* VGM implementation of reboot */

#include <bsp/vmeUniverse.h>
#include <rtems/bspIo.h>
#include <libcpu/stackTrace.h>
#include <bspVGM.h>

/* vector through a pointer for trying
 * a VME reset.
 * REASON: give applications the option
 *         _not_ to use VME and _not_
 *         linking against the vmeUniverse
 *         driver.
 *         This variable should be set by
 *         BSP_vme_config().
 */
void (*__BSP_alternate_reset)(void)=0;

void
rtemsReboot(void)
{
	printk("Printing a stack trace for your convenience :-)\n");
	CPU_print_stack();
	/* try a watchdog reset (the board's response
	 * to its own VME SYSRESET may be disabled
	 * in the respective board control register
	 */
	if ( BSP_watchdogArm(10) && __BSP_alternate_reset ) {
		/* watchdog unsupported; try VME */
		__BSP_alternate_reset();
	}
}
