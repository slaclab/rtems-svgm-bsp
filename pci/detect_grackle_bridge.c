/*
 * $Id$
 *
 *  Copyright (C) 2001 Till Straumann <strauman@slac.stanford.edu>
 *
 */

#include <libcpu/io.h>
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
#define ERR_STATUS_GRCKL_OK(s) (0x0080==(s))

#define PICR1				0xa8	/* LONG register */
#define PICR1_MCP_EN			(1<<11)

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
_BSP_clear_hostbridge_errors(void)
{
unsigned long	picr1;
unsigned char	errenR1,errdr1,errdr2;
unsigned short	pcistat,pcistat_orig;
int				count;
	/* disable MCP interrupt generation PICR1[MCP_EN]=0 */
	pci_read_config_dword(0,0,0,PICR1,  &picr1);
	pci_write_config_dword(0,0,0,PICR1, picr1 & ~PICR1_MCP_EN);

	pci_read_config_byte(0,0,0,ERRENR1,  &errenR1);
	pci_write_config_byte(0,0,0,ERRENR1, errenR1 & ~ERRENR1_60X_BUS_ERR);

	/* read error status for info return */
	pci_read_config_word(0,0,0,PCI_STATUS,&pcistat_orig);
	pci_read_config_byte(0,0,0,ERRDR1,&errdr1);
	pci_read_config_byte(0,0,0,ERRDR2,&errdr2);

	count=50;
	do {
		/* clear error reporting registers */
		pci_write_config_byte(0,0,0,ERRDR1,ERRDR_CLR_ALL);
		pci_write_config_byte(0,0,0,ERRDR2,ERRDR_CLR_ALL);

		/* clear PCI status register */
		pci_write_config_word(0,0,0,PCI_STATUS, 0xffff);
		pci_read_config_word(0,0,0,PCI_STATUS, &pcistat);
	} while ( ! ERR_STATUS_GRCKL_OK(pcistat) && --count);

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

	if (ERR_STATUS_GRCKL_OK(pcistat)) {
		/* re-enable error/MCP generation */
		pci_write_config_byte(0,0,0,ERRENR1, errenR1 | 1);
		pci_write_config_dword(0,0,0,PICR1,picr1 | PICR1_MCP_EN);
	} else {
		printk("I have problems clearing Grackle PCI status register (still 0x%04x);\n",
				pcistat);
		printk("leaving MCP interrupt disabled\n");
	}
	return (errdr2<<24) | (errdr1<<16) | pcistat_orig;
}

static inline unsigned long
MFHID0(void)
{
unsigned long rval;
__asm__ __volatile__("mfspr %0, %1":"=r"(rval):"i"(HID0));
return rval;
}

static inline void
MTHID0(unsigned long val)
{
__asm__ __volatile__("mtspr %0, %1"::"i"(HID0),"r"(val));
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
   * standart devices detection mecahnism that was used on PC
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
  /* clear possible error flags, so MCP can be enabled */
  {
    unsigned long errs=_BSP_clear_hostbridge_errors();
	if ( ! ERR_STATUS_GRCKL_OK(errs)) {
		printk("Cleared Grackle errors: pci_stat was 0x%04x errdr1 0x%02x errdr2 0x%02x\n",
				errs & 0xffff, (errs>>16) & 0xff, (errs>>24) &0xff);
	} else {
		unsigned long hid0;
		pci_write_config_byte(0,0,0,ERRENR1,0xff);
		pci_write_config_byte(0,0,0,ERRENR2,0x81);
		/* enable MCP interrupt */
		MTHID0(MFHID0()|0x80000000);
	}
  }
}
