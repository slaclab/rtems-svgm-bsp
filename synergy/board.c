/* $Id$ */
#include <bsp/bspVGM.h>
#include "synergyregs.h"
#include <rtems/bspIo.h>
#include <libcpu/spr.h>
#include <libcpu/cpuIdent.h>

SPR_RO(HID1)

const char *
BSP_boardType(void)
{
unsigned char boardRev=*SYN_VGM_REG_INFO_BOARD_REV;
	switch ( SYN_VGM_REG_INFO_BOARD_TYPE_MSK & boardRev ) {
		case SYN_VGM_REG_INFO_BOARD_TYPE_VGM5:
			return "VGM5";
		case SYN_VGM_REG_INFO_BOARD_TYPE_VGMD:
			return "VGMD";
		default:
			break;
	}
	return 0;	/* unsupported board */
}

/* The necessary information for detecting the CPU clock frequency
 * has been supplied by Synergy Microsystems - thanks :-)
 */

static unsigned pllBitsTable[3][16] = {
	/* SVGM5, SVGMD, KGM5, SVSS4, KSS4 */
	{  0, 15, 14,  0,  4, 13,  5,  9,
	   6, 11,  8, 10, 16, 12,  7,  0},
	/* SVGM2_04 */
	{  0,  0, 14,  0,  0, 13,  5,  9,
	   6, 11,  8, 10,  0, 12,  7,  0},
	/* SVGM1_04 */
	{  0,  0, 14,  0,  4, 13,  5,  9,
	   6, 11,  8, 10,  0, 12,  7,  0},
};

unsigned long
BSP_getCpuClock(unsigned long busFreq)
{
int				type = -1;
unsigned char	boardRev=*SYN_VGM_REG_INFO_BOARD_REV;
long long		rval = 0;
unsigned long	idx;

	switch ( SYN_VGM_REG_INFO_BOARD_TYPE_MSK & boardRev ) {
		case SYN_VGM_REG_INFO_BOARD_TYPE_VGM5:
		case SYN_VGM_REG_INFO_BOARD_TYPE_VGMD:
			type = 0;
		default:
			break;
	}
	if ( type < 0 ) {
		printk("WARNING; BSP_getCpuClock(): unknown board, add to 'svgm/synergy/board.c'\n");
		return -1;
	} else {
		idx = _read_HID1();
		switch (get_ppc_cpu_type()) {
#if 0 	/* 745x not recognized yet */
			case PPC_7450:
			case PPC_7455:
			case PPC_7457:
				idx >>= 13;
			break;
#endif
			default: 
				idx>>=28;
			break;
		}
		/* this overflows if the CPU clock is > ~ 4GHz */
		rval = ((long long)busFreq * (long long)(pllBitsTable[type][idx&0xf]));
		rval /= 2;
	}
	return (unsigned long)rval;
}
