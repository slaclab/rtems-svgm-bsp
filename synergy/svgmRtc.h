#ifndef SVGM_TIMEKEEPER_H
#define SVGM_TIMEKEEPER_H
/* $Id$ */

#include <time.h>

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2004 */

/* Start the RTC oscillator. */
void BSP_rtcOscillatorStart();

/* Stop the RTC oscillator -- this saves battery if the RTC is unused */
void BSP_rtcOscillatorStop();

/* Return nonzero/zero if the oscillator is currently running / not running */ 
int BSP_rtcOscillatorIsRunning();

/* Read the RTC into a struct tm (to be provided by the user)
 * NOTE: tm_isdst  and tm_yday are not supported and set to -1.
 *
 * RETURNS: 0 on success (always).
 */
int BSP_rtcRead(struct tm *ptm);

/* Set the RTC from a struct tm. If it the oscillator is
 * currently stopped, this call does *not* start it.
 *
 * RETURNS: zero on success, -1 on failure (invalid tm field
 *          values).
 */

int BSP_rtcSet(const struct tm *ptm);
#define CALIBRATION_UNIT	(32768*15)	/* calibration is in units of 1/(32768*15) s/s */
#define CALIBRATION_MAX		CTRL_CLB_MSK

/* Calibrate the RTC oscillator speed.
 *
 * INPUT: 'diffSecs': drift (in seconds) of the RTC against a
 *        reference clock ( positive if the RTC is faster, i.e.,
 * 
 *               diffSecs = rtcTime - trueTime
 *
 *        'period': time period (in seconds) over which the drift
 *                  accumulated
 *
 * RETURNS: zero on success.
 *          -1   on parameter error (diffSecs == 0) or period
 *               obviously too short to detect anything.
 *           1   the required calibration exceeded the range
 *               supported by the hardware. Calibration is still
 *               set, but clamped to the supported max.
 *
 * NOTE:   calibration range is +-0..31 units of 1/(32768*15) seconds/second
 */
int BSP_rtcCalibrate(long diffSecs, unsigned long period);

/* Determine the RTC/NVRAM battery status
 * RETURNS: 0 if battery is OK, nonzero if it is low.
 */
int BSP_rtcBattIsLow();

#endif
