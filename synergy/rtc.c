/* $Id$ */

#include <bsp/svgmRtc.h>
#include <libcpu/io.h>

/* Register definitions for the M48T201Y Clock/Timekeeper */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2004 */

typedef struct M48T201YRegsRec_ {
	volatile unsigned char	flags;
	volatile unsigned char	century;
	volatile unsigned char	alarmSeconds;
	volatile unsigned char	alarmMinutes;
	volatile unsigned char	alarmHours;
	volatile unsigned char	alarmDate;
	volatile unsigned char	alarmMonth;
	volatile unsigned char	watchdog;
	volatile unsigned char	control;
	volatile unsigned char	seconds;
	volatile unsigned char	minutes;
	volatile unsigned char	hour;
	volatile unsigned char	dayOfWeek;
	volatile unsigned char	dayOfMonth;
	volatile unsigned char	month;
	volatile unsigned char	year;
} M48T201YRegsRec, *M48T201YRegs;

static M48T201YRegs	svgmRtc = (M48T201YRegs)0xffe9fff0;

#define FLAG_WDF		(1<<7)	/* Watchdog Flag         */
#define FLAG_AF			(1<<6)	/* Alarm Flag            */
#define FLAG_BL			(1<<4)	/* Battery Low           */
#define FLAG_RS_MSK		(0xf)	/* SQW Frequency Bits    */
#define FLAG_RS_(val)	((val)&FLAG_RS_MSK)

#define CTRL_W			(1<<7)	/* Write Access Bit      */
#define CTRL_R			(1<<6)	/* Read  Access Bit      */
#define	CTRL_S			(1<<5)  /* Calibration Sign Bit  */
#define CTRL_CLB_MSK	(0x1f)  /* Calibration Bits      */

#define SECS_STOP		(1<<7)	/* Stop Oscillator       */

#define WDAY_FT			(1<<6)	/* Frequency Test Bit    */
void
BSP_rtcOscillatorStart()
{
unsigned flags;
unsigned char v;
	rtems_interrupt_disable(flags);
		v = in_8(&svgmRtc->seconds);
		out_8(&svgmRtc->seconds, (v & ~SECS_STOP));
	rtems_interrupt_enable(flags);
}


void
BSP_rtcOscillatorStop()
{
unsigned flags;
unsigned char v;
	rtems_interrupt_disable(flags);
		v = in_8(&svgmRtc->seconds);
		out_8(&svgmRtc->seconds, (v | SECS_STOP));
	rtems_interrupt_enable(flags);
}

#if 0
/* avoid pulling in C-library header */

struct tm {
	int tm_sec;
	int tm_min;
	int tm_hour;
	int tm_mday;
	int tm_mon;
	int tm_year;
	int tm_wday;
	int tm_yday;
	int tm_isdst;
};
#endif

static unsigned char frombcd(unsigned char n)
{
	return 10*(n>>4)+(n&0xf);
}

static unsigned char tobcd(unsigned char n)
{
unsigned char tmp = n/10;
	return (tmp<<4) + (n - 10*tmp);
}

int
BSP_rtcRead(struct tm *ptm)
{
M48T201YRegsRec cpy;
unsigned 		flags;
unsigned char	ctrl;

	rtems_interrupt_disable(flags);
		ctrl = in_8(&svgmRtc->control);
		out_8(&svgmRtc->control, (ctrl | CTRL_R) );
		cpy = *svgmRtc;
		asm volatile("eieio");
		out_8(&svgmRtc->control, ctrl);
	rtems_interrupt_enable(flags);

	ptm->tm_sec     = frombcd(cpy.seconds & ~SECS_STOP);
	ptm->tm_min     = frombcd(cpy.minutes);
	ptm->tm_hour    = frombcd(cpy.hour);
	ptm->tm_mday    = frombcd(cpy.dayOfMonth);
	ptm->tm_mon     = frombcd(cpy.month) - 1;
	ptm->tm_year    = 100*frombcd(cpy.century) + frombcd(cpy.year) - 1900;

	/* adjust for differences between struct tm and hardware regs */
	ptm->tm_isdst = -1; /* dont' know */
	/* newlib mktime() ignores these */
	ptm->tm_yday  = -1;
	ptm->tm_wday  = -1;
	return 0;
}

int
BSP_rtcSet(const struct tm *ptm)
{
unsigned 		flags;
unsigned char	ctrl;
unsigned char	sec,min,hour,mday,mon,wday,year,cent;

	if ( ptm->tm_sec  < 0 || ptm->tm_sec  > 59 ) return -1;
	if ( ptm->tm_min  < 0 || ptm->tm_min  > 59 ) return -1;
	if ( ptm->tm_hour < 0 || ptm->tm_hour > 23 ) return -1;
	if ( ptm->tm_mday < 1 || ptm->tm_mday > 31 ) return -1;
	if ( ptm->tm_mon  < 0 || ptm->tm_mon  > 11 ) return -1;
	if ( ptm->tm_year < 0 ) return -1;
	if ( ptm->tm_wday < 0 || ptm->tm_wday >  6 ) return -1;

	sec  = tobcd(ptm->tm_sec);
	min  = tobcd(ptm->tm_min);
	hour = tobcd(ptm->tm_hour);
	mday = tobcd(ptm->tm_mday);
	mon  = tobcd(ptm->tm_mon + 1);
	wday = tobcd(ptm->tm_wday);
	year = tobcd((ptm->tm_year + 1900) % 100);
	cent = tobcd((ptm->tm_year + 1900) / 100);

	rtems_interrupt_disable(flags);
		ctrl = in_8(&svgmRtc->control);
		out_8(&svgmRtc->control, (ctrl | CTRL_W) );
		svgmRtc->seconds = sec | (svgmRtc->seconds & SECS_STOP);
		svgmRtc->minutes = min;
		svgmRtc->hour    = hour;
		svgmRtc->dayOfMonth = mday;
		svgmRtc->month   = mon;
		svgmRtc->dayOfWeek  = wday | ( svgmRtc->dayOfWeek & WDAY_FT );
		svgmRtc->year    = year;
		svgmRtc->century = cent;
		asm volatile("eieio");
		out_8(&svgmRtc->control, ctrl);
	rtems_interrupt_enable(flags);
	return 0;
}

#define CALIBRATION_UNIT	(32768*15)	/* calibration is in units of 1/(32768*15) s/s */
#define CALIBRATION_MAX		CTRL_CLB_MSK

int
BSP_rtcCalibrate(long diffSecs, unsigned long period)
{
int					rval = 0;
int					sign = 0;
unsigned long long	cal;
unsigned			flags, ctrl;

	if (  0 == diffSecs || period < CALIBRATION_MAX )
		return -1;

	/* diffSecs = rtcTime - trueTime; diff > 0 --> slow down */
	if ( diffSecs < 0 ) {
		sign = 1;
		diffSecs = -diffSecs;
	}
	cal = ((unsigned long long )diffSecs * CALIBRATION_UNIT) / (unsigned long long)period;
	if ( cal > CALIBRATION_MAX ) {
		cal = CALIBRATION_MAX;
		rval = 1;
	}

	if (sign)
		cal |= CTRL_S;

	rtems_interrupt_disable(flags);
		ctrl = in_8(&svgmRtc->control);
		ctrl &= ~(CTRL_CLB_MSK | CTRL_S);
		ctrl |= cal;
		out_8(&svgmRtc->control, ctrl);
	rtems_interrupt_enable(flags);
	
	return rval;
}

int
BSP_rtcBattIsLow()
{
	return 0 != (in_8(&svgmRtc->flags) & FLAG_BL);
}

int
BSP_rtcOscillatorIsRunning()
{
	return !(in_8(&svgmRtc->seconds) & SECS_STOP);
}
