#ifndef BSP_SYNERGY_VGM_INTERFACE_H
#define BSP_SYNERGY_VGM_INTERFACE_H
/* $Id$ */

/* Interface for Synergy VGM specific features of the BSP */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 3/2002 */

/* get type description string for this board ("VGM5", "VGMD", ...)
 * RETURNS: constant string or 0 for unsupported board types
 */
const unsigned char *
BSP_boardType(void);

/* LED Support */

/* Set the 8 user LEDs to reflect 'val'
 * This routine is 'dumb' and has no
 * protection against race conditions...
 */
void
BSP_setLEDs(unsigned char val);

/* Read back current LED status
 * Again, no protection from concurrent access
 * by multiple threads...
 */
unsigned char
BSP_getLEDs(void);

/* Support for the watchdog timer */

/* NOTE: Not all board revisions support this feature */

/* Arm the watchdog to reset the board
 * if not 'petted' or re-armed within
 * 'usecs' microseconds.
 *
 * Arming the watchdog with a 0 argument
 * disables it.
 *
 * RETURNS: 0 on success, nonzero if
 *          the board doesn't support
 *          MPIC timer watchdog reset.
 */
long
BSP_watchdogArm(unsigned long usecs);

void
BSP_watchdogPet(void);

#endif
