/*
 *  Support for the VGM LEDs
 *
 *  $Id$
 *
 *  Author: Till Straumann <strauman@slac.stanford.edu>
 *          3/2002
 */


#include <synergyregs.h>
#include <bspVGM.h>

void
BSP_setLEDs(unsigned char val)
{
int i;
SynergyVGMBoardReg ledreg = SYN_VGM_REG_CTRL_USR_LED_0;
	for (i=0; i<8; i++,ledreg+=8,val>>=1)
		*ledreg = val&1 ? SYN_VGM_REG_CTRL_USR_LED_ON : 0;	
	__asm__ __volatile__("eieio");
}

unsigned char
BSP_getLEDs(void)
{
int i;
unsigned char rval;
SynergyVGMBoardReg ledreg = SYN_VGM_REG_CTRL_USR_LED_7;
	for (i=0,rval=0; i<8; i++,ledreg-=8) {
		rval<<=1;
		if (*ledreg & SYN_VGM_REG_CTRL_USR_LED_ON)
			rval++;
	}
	return rval;
}
