/*
 * $Id$
 *
 *  Copyright (C) 2001,2003 Till Straumann <strauman@slac.stanford.edu>
 *
 */

#include <libcpu/io.h>
#include <libcpu/spr.h>
#include <bsp/pci.h>
#include <bsp.h>
#include <rtems/bspIo.h> /* printk */

#ifndef _GRACKLE_PCI_CONFIG_ADDR
#define _GRACKLE_PCI_CONFIG_ADDR 0xfec00000
#endif
#ifndef _GRACKLE_PCI_CONFIG_DATA
#define _GRACKLE_PCI_CONFIG_DATA 0xfee00000
#endif

/* this can also be used on the return value of
 * BSP_clear_hostbridge_errors()
 */
#define PCI_STATUS_FAST_BTB		0x80	/* Grackle is fast back to back capable as a target */
#define PCI_STATUS_GRCKL_OK(s) (PCI_STATUS_FAST_BTB==(s))

#define PICR1				0xa8	/* LONG register */
#define PICR1_MCP_EN			(1<<11)
#define PICR1_TEA_EN			(1<<10)

#define ERRENR1				0xc0	/* BYTE register */
#define ERRENR1_60X_BUS_ERR		(1<<0)
#define ERRENR1_PCI_MAS_ABT		(1<<1)
#define ERRENR1_MEM_PAR_ERR		(1<<2)
#define ERRENR1_PCI_MAS_PERR	(1<<3)
#define ERRENR1_MEM_RFSH_OFL	(1<<4)
#define ERRENR1_MEM_SEL_ERR		(1<<5)
#define ERRENR1_PCI_TGT_PERR	(1<<6)
#define ERRENR1_PCI_TGT_ABT		(1<<7)

#define ERRDR1				0xc1	/* error reporting register 1 (BYTE)	*/
#define ERRENR2				0xc4	/* error enable register    2 (BYTE)    */
#define ERRDR2				0xc5	/* error reporting register 2 (BYTE)	*/
#define ERRDR_CLR_ALL		0xff	/* write 1 to clear error bits	*/

/* Values Synergy uses in their BSP;
 *
 * My VGMD froze with more paranoid settings
 * while the VGM5 worked fine.
 *
 * Note that the ERRENR bits only affect MCP
 * but not TEA and most (if not all - I was
 * unable to test anything other than the
 * PCI_TGT_ABT and PCI_MAS_ABT which are
 * crucial for probing) of the error conditions
 * seem to pull TEA anyways (unfortunately,
 * this is not precisely documented in the
 * Grackle manual)...
 *
 * The right thing to do would probably be
 * enabling only stuff that is not pulling TEA
 * but that's not documented and some cases
 * are difficult to test :-(
 */
#define ERRENR1_VAL				0x79
#define ERRENR2_VAL				0xa1

extern pci_config_access_functions pci_direct_functions;
extern pci_config_access_functions pci_indirect_functions;

/* clear all error flags adhering to the algorithm
 * recommended by motorola - MPC106 user's manual
 * sect. 9.3.3.3
 *
 * Returns: (for diagnostic purposes)
 *          original settings (i.e. before applying the clearing
 *          sequence) of
 *          (errdr2<<24) | (errdr1<<16) | pci_status
 */
unsigned long
_BSP_clear_hostbridge_errors(int enableMCP, int quiet)
{
unsigned long	rval, status;
unsigned int	picr1;
unsigned char	errenR1,errdr1,errdr2;
unsigned short	pcistat,pcistat_orig;
int				count;
	/* disable MCP interrupt generation PICR1[MCP_EN]=0 */
	pci_read_config_dword(0,0,0,PICR1,  &picr1);
	pci_write_config_dword(0,0,0,PICR1, picr1 & ~(PICR1_MCP_EN|PICR1_TEA_EN));

	pci_read_config_byte(0,0,0,ERRENR1,  &errenR1);
	pci_write_config_byte(0,0,0,ERRENR1, errenR1 & ~ERRENR1_PCI_MAS_ABT);

	/* read error status for info return */
	pci_read_config_word(0,0,0,PCI_STATUS,&pcistat_orig);
	pci_read_config_byte(0,0,0,ERRDR1,&errdr1);
	pci_read_config_byte(0,0,0,ERRDR2,&errdr2);

	count=10;
	do {
		/* clear error reporting registers */

		pci_write_config_byte(0,0,0,ERRDR1,ERRDR_CLR_ALL);
		pci_write_config_byte(0,0,0,ERRDR2,ERRDR_CLR_ALL);

		/* clear PCI status register */
		pci_write_config_word(0,0,0,PCI_STATUS, 0xffff);

		/* read  new status */
		pci_read_config_word(0,0,0,PCI_STATUS, &pcistat);

		/* this seems to be necessary to reliably
		 * clear the master abort flag - mysterious...
		 */
		__asm__ __volatile__("sync");
		rtems_bsp_delay(2);
		__asm__ __volatile__("sync");
	} while ( ! PCI_STATUS_GRCKL_OK(pcistat) && count-- );

	/* we also read 4 words off the machine check vector
	 * location to make sure the Grackle has seen
	 * the CPU reading the vector which it needs for
	 * acknowledging an earlier checkstop interrupt.
	 * Otherwise, a new one might not get through...
	 */
	__asm__ __volatile__(
			"dcbf 0, %0        \n"	/* make sure we are not reading the cache */
			"sync              \n"
			"lwz  %%r0, 0(%0)  \n"
			"lwzu %%r0, 4(%0)  \n"
			"lwzu %%r0, 4(%0)  \n"
			"lwzu %%r0, 4(%0)  \n"
			"sync              \n"
		:
		:"r"(0x200)	/* machine check vector is 0x200 */
		:"r0");

	rval = (errdr2<<24) | (errdr1<<16) | pcistat_orig;

	pci_read_config_byte(0,0,0,ERRDR1,&errdr1);
	pci_read_config_byte(0,0,0,ERRDR2,&errdr2);

	status = (errdr2<<24) | (errdr1<<16) | pcistat;

	if ( !PCI_STATUS_GRCKL_OK(rval) && !quiet) {
		printk("Cleared Grackle errors: pci_stat was 0x%04x errdr1 0x%02x errdr2 0x%02x\n",
					pcistat_orig, errdr1, errdr2);
  	}

	if ( PCI_STATUS_GRCKL_OK(status) && enableMCP) {
		/* re-enable error/MCP generation */
		if (!quiet)
			printk("Enabling MCP and TEA generation on hostbridge errors\n");
#if 0 /* restore original settings */
		pci_write_config_byte(0,0,0,ERRENR1, errenR1 | ERRENR1_60X_BUS_ERR);
#else /* enable MCP on all errors - paranoia setting */
		pci_write_config_byte(0,0,0,ERRENR1,ERRENR1_VAL);
		pci_write_config_byte(0,0,0,ERRENR2,ERRENR2_VAL);
#endif
		pci_write_config_dword(0,0,0,PICR1,picr1 | (PICR1_MCP_EN|PICR1_TEA_EN));
	} else {
		if (!quiet && enableMCP) {
			printk("leaving MCP and TEA interrupts disabled\n");
		}
	}
	return rval & ~PCI_STATUS_FAST_BTB;
}

void detect_host_bridge()
{
  unsigned int id0;

  /* setup the correct address configuration */
  BSP_pci_configuration.pci_config_addr = (void*)_GRACKLE_PCI_CONFIG_ADDR;
  BSP_pci_configuration.pci_config_data = (void*)_GRACKLE_PCI_CONFIG_DATA;

  /* TS: left this comment from the raven-bridge code; I am not
   *     sure if it applies, though...
   * This code assumes that the host bridge is located at
   * bus 0, dev 0, func 0 AND that the old pre PCI 2.1
   * standart devices detection mechanism that was used on PC
   * (still used in BSD source code) works.
   */
  {
	/* T. Straumann; Grackle needs indirect_functions */
    BSP_pci_configuration.pci_functions = &pci_indirect_functions; 

    /* On all direct bridges I know the host bridge itself
     * appears as device 0 function 0. 
	 * Should we use pciFindDevice instead??
     */
    pci_read_config_dword(0, 0, 0, PCI_VENDOR_ID, &id0);
    if (id0==~0U) {
	    BSP_panic("Unable to detect host bridge");
    }
  }
  if( ! id0 == PCI_VENDOR_ID_MOTOROLA +
     (PCI_DEVICE_ID_MOTOROLA_MPC106<<16)) {
	    BSP_panic("Host Bridge is not a MPC106/Grackle?????");
  }
  printk("Motorola MPC106/Grackle hostbridge detected\n");
}
