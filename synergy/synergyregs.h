#ifndef SYNERGYREG_H
#define SYNERGYREG_H
/* $Id$ */

/* Author: Till Straumann <strauman@slac.stanford.edu>
 *         3/2002
 */

/*
 * Synergy VGM board register support
 */

typedef volatile unsigned char * SynergyVGMBoardReg;

/* Board Info Registers */
#define SYN_VGM_REG_INFO_BOARD_REV			((SynergyVGMBoardReg)0xffeffe00)
#if 0 /* not tested */
#  define SYN_VGM_REG_INFO_BOARD_TYPE_VGM1			(1<<4)
#  define SYN_VGM_REG_INFO_BOARD_TYPE_VGM2			(2<<4)
#  define SYN_VGM_REG_INFO_BOARD_TYPE_VSS4			(4<<4)
#endif
#  define SYN_VGM_REG_INFO_BOARD_TYPE_VGM5			(5<<4)
#if 0 /* not tested */
#  define SYN_VGM_REG_INFO_BOARD_TYPE_VGMC			(0xc<<4)
#endif
#  define SYN_VGM_REG_INFO_BOARD_TYPE_VGMD			(0xd<<4)
#  define SYN_VGM_REG_INFO_BOARD_TYPE_MSK			(0xf<<4)
#  define SYN_VGM_REG_INFO_BOARD_REV_MSK			0xf

#define SYN_VGM_REG_INFO_SPC_MOD			((SynergyVGMBoardReg)0xffeffe08)

#define SYN_VGM_REG_INFO_BOARD_FAM			((SynergyVGMBoardReg)0xffeffe10)
#  define SYN_VGM_REG_INFO_BOARD_FAM_VGM			(0<<4)
#if 0 /* not tested */
#  define SYN_VGM_REG_INFO_BOARD_FAM_VSS			(1<<4)
#  define SYN_VGM_REG_INFO_BOARD_FAM_KGM			(2<<4)
#  define SYN_VGM_REG_INFO_BOARD_FAM_VGR			(3<<4)
#endif

#define SYN_VGM_REG_INFO_L2_CACHE			((SynergyVGMBoardReg)0xffeffe30)

#define SYN_VGM_REG_INFO_MEMORY				((SynergyVGMBoardReg)0xffeffe38)
#  define SYN_VGM_REG_INFO_MEMORY_BANKS(reg)		((1 << (((int)((reg)&0x7))-1)) & 0xf)
#  define SYN_VGM_REG_INFO_MEMORY_BANK_SIZE(reg)	(8*1024*1024 << (((reg)>>3)&7))

#define SYN_VGM_REG_INFO_2ND_PCI_SLT		((SynergyVGMBoardReg)0xffeffe48)

#define SYN_VGM_REG_INFO_VME64_SLT			((SynergyVGMBoardReg)0xffefff30)
#  define SYN_VGM_REG_INFO_VME64_SLT_NUM(reg)		((reg)&0x1f)
#  define SYN_VGM_REG_INFO_VME64_SLT_VALID			(1<<5)
#  define SYN_VGM_REG_INFO_VME64_SLT_AUTO			(1<<6)

/* Status Registers */
#define SYN_VGM_REG_STAT_USR_SWITCH			((SynergyVGMBoardReg)0xffeffd00)

#define SYN_VGM_REG_STAT_BOARD				((SynergyVGMBoardReg)0xffeffe18)
#  define SYN_VGM_REG_STAT_BOARD_FAILED				1
#  define SYN_VGM_REG_STAT_BOARD_EJECTED			2
#  define SYN_VGM_REG_STAT_CPU_BUS_SPEED_66			(0<<2)
#  define SYN_VGM_REG_STAT_CPU_BUS_SPEED_83			(1<<2)
#  define SYN_VGM_REG_STAT_CPU_BUS_SPEED_100		(2<<2)
#  define SYN_VGM_REG_STAT_CPU_BUS_SPEED_MSK		(3<<2)

#define SYN_VGM_REG_STAT_CPU				((SynergyVGMBoardReg)0xffeffe20)

/* Control Mode Registers */
#define SYN_VGM_REG_CTRL_CPU_TIMEBASE		((SynergyVGMBoardReg)0xffeffe28)
#  define SYN_VGM_REG_CTRL_CPUX_TIMEBASE_EN			1
#  define SYN_VGM_REG_CTRL_CPUY_TIMEBASE_EN			2

#define SYN_VGM_REG_CTRL_FLASH				((SynergyVGMBoardReg)0xffeffe40)
#  define SYN_VGM_REG_CTRL_FLASH_PROTECT_SW			1
#  define SYN_VGM_REG_CTRL_FLASH_PROTECTED_HW		(1<<6)
#  define SYN_VGM_REG_CTRL_FLASH_BOOT_EPROM			(1<<7)

/* This is write only */
#define SYN_VGM_REG_CTRL_FLASH_WIN			((SynergyVGMBoardReg)0xffeffe50)
#  define SYN_VGM_REG_CTRL_FLASH_WIN_BANK(reg)		((reg)&0x7f)
#  define SYN_VGM_REG_CTRL_FLASH_WIN_USER			(1<<7)

#define SYN_VGM_REG_CTRL_P0_PCI				((SynergyVGMBoardReg)0xffeffe68)

#define SYN_VGM_REG_CTRL_USR_LED_0			((SynergyVGMBoardReg)0xffeffe80)
#  define SYN_VGM_REG_CTRL_USR_LED_ON					1
#define SYN_VGM_REG_CTRL_USR_LED_1			((SynergyVGMBoardReg)0xffeffe88)
#define SYN_VGM_REG_CTRL_USR_LED_2			((SynergyVGMBoardReg)0xffeffe90)
#define SYN_VGM_REG_CTRL_USR_LED_3			((SynergyVGMBoardReg)0xffeffe98)
#define SYN_VGM_REG_CTRL_USR_LED_4			((SynergyVGMBoardReg)0xffeffea0)
#define SYN_VGM_REG_CTRL_USR_LED_5			((SynergyVGMBoardReg)0xffeffea8)
#define SYN_VGM_REG_CTRL_USR_LED_6			((SynergyVGMBoardReg)0xffeffeb0)
#define SYN_VGM_REG_CTRL_USR_LED_7			((SynergyVGMBoardReg)0xffeffeb8)

/* The Direct PowerPC - VME Bridge is not present on VGMD */
#define SYN_VGM_REG_CTRL_VME64_SLV_MSK		((SynergyVGMBoardReg)0xffefff00)
#  define SYN_VGM_REG_CTRL_VME64_SLV_MSK_WIN_16MB		0
#  define SYN_VGM_REG_CTRL_VME64_SLV_MSK_WIN_32MB		1
#  define SYN_VGM_REG_CTRL_VME64_SLV_MSK_WIN_64MB		3
#  define SYN_VGM_REG_CTRL_VME64_SLV_MSK_WIN_128MB		7
#  define SYN_VGM_REG_CTRL_VME64_SLV_MSK_WIN_256MB		0xf
#  define SYN_VGM_REG_CTRL_VME64_SLV_MSK_BCST_WAIT_B	(1<<4)
#  define SYN_VGM_REG_CTRL_VME64_SLV_MSK_BCST_DTA_ENA	(1<<5)
#  define SYN_VGM_REG_CTRL_VME64_SLV_MSK_BCST_ENA		(1<<6)
#  define SYN_VGM_REG_CTRL_VME64_SLV_MSK_ENABLE			(1<<7)

#define SYN_VGM_REG_CTRL_VME64_SLV_ADDR		((SynergyVGMBoardReg)0xffefff08)

#define SYN_VGM_REG_CTRL_VME64_BCST_SLV_ADDR	((SynergyVGMBoardReg)0xffefff10)

/* The Direct PowerPC - VME Bridge is not present on VGMD */
#define SYN_VGM_REG_CTRL_VME64_MAS_ADDR		((SynergyVGMBoardReg)0xffefff18)

#define SYN_VGM_REG_CTRL_VME64_MODE			((SynergyVGMBoardReg)0xffefff20)
#  define SYN_VGM_REG_CTRL_VME64_MODE_BUS_REQ_LVL(reg)	((reg)&3)
#  define SYN_VGM_REG_CTRL_VME64_MODE_FAIR				(1<<2)
#  define SYN_VGM_REG_CTRL_VME64_MODE_BUS_REL_ROR		(1<<3)
#  define SYN_VGM_REG_CTRL_VME64_MODE_BLT_FAST			(1<<4)
#  define SYN_VGM_REG_CTRL_VME64_MODE_BCST_SLV_DIRECT	(1<<7)

#define SYN_VGM_REG_CTRL_VME64_RMW			((SynergyVGMBoardReg)0xffefff28)
#  define SYN_VGM_REG_CTRL_VME64_RMW_RMW				1	

#define SYN_VGM_REG_CTRL_VME64_SYS_RESET	((SynergyVGMBoardReg)0xffefff38)
#  define SYN_VGM_REG_CTRL_VME64_SYS_RESET_RST			1

#endif
