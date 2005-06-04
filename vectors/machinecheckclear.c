/* $Id$ */

/* SVGM-specific machine check clearing */

#include <bsp.h>
#include <bsp/vectors.h>
#include <libcpu/spr.h>
#include <bsp/pci.h>
#include <rtems/bspIo.h>

#include <bsp/bspException.h>

void 
BSP_machineCheckClearException(BSP_Exception_frame *excPtr, int quiet)
{
	if (excPtr->EXC_SRR1 & (SRR1_MCP_EXC)) {
		unsigned char c1,c2;
		unsigned int  l;
		unsigned long gerr;

		/* it's MCP; gather info from the host bridge */

		/* read latched status prior to resetting a potential error condition */
		pci_read_config_byte(0,0,0,0xc3,&c1);
		pci_read_config_byte(0,0,0,0xc7,&c2);
		pci_read_config_dword(0,0,0,0xc8,&l);

		gerr=_BSP_clear_hostbridge_errors(0,0);
		if (0x80 != gerr) {
			if (!quiet) {
				printk("Grackle Error Registers: PCI CSR: %04x, ERRDR1: %02x, ERRDR2: %02x\n",
						gerr & 0xffff, (gerr>>16) & 0xff, (gerr>>24)&0xff);
				printk("                           60x Bus Error Status: %02x\n",c1);
				printk("                           PCI Bus Error Status: %02x\n",c2);
				printk("                           Bus Error Address   : %08x\n",l);
			}
		} else {
			if (!quiet)
				printk("Grackle seems OK\n");
		}
	} else {
		if (!quiet)
			printk("\n");
	}
}
