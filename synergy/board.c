/* $Id$ */
#include <bspVGM.h>
#include <synergyregs.h>

const unsigned char *
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
