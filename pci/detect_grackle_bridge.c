/*
 * $Id$
 *
 *  Copyright (C) 2001 Till Straumann <strauman@slac.stanford.edu>
 *
 */

#include <libcpu/io.h>
#include <bsp/pci.h>
#include <bsp.h>
#include <bspIo.h> /* printk */

#ifndef _GRACKLE_PCI_CONFIG_ADDR
#define _GRACKLE_PCI_CONFIG_ADDR 0xfec00000
#endif
#ifndef _GRACKLE_PCI_CONFIG_DATA
#define _GRACKLE_PCI_CONFIG_DATA 0xfee00000
#endif

void detect_host_bridge()
{
  unsigned int id0;

  /* setup the correct address configuration */
  BSP_pci_configuration.pci_config_addr = (void*)_GRACKLE_PCI_CONFIG_ADDR;
  BSP_pci_configuration.pci_config_data = (void*)_GRACKLE_PCI_CONFIG_DATA;

  /*
   * This code assumes that the host bridge is located at
   * bus 0, dev 0, func 0 AND that the old pre PCI 2.1
   * standart devices detection mecahnism that was used on PC
   * (still used in BSD source code) works.
   */
  {
#if 0 /* T. Straumann, seems to need indirect_functions
       * I dont really understand this however...
       */
    BSP_pci_configuration.pci_functions = &pci_direct_functions; 
#endif
    /* On all direct bridges I know the host bridge itself
     * appears as device 0 function 0. 
     */
    pci_read_config_dword(0, 0, 0, PCI_VENDOR_ID, &id0);
    if (id0==~0U) {
	    BSP_panic("Unable to detect host bridge");
    }
  }
  pci_read_config_dword(0, 0, 0, 0, &id0);
  if( ! id0 == PCI_VENDOR_ID_MOTOROLA +
     (PCI_DEVICE_ID_MOTOROLA_MPC106<<16)) {
	    BSP_panic("Host Bridge is not a MPC106/Grackle?????");
  }
  printk("Motorola MPC106/Grackle hostbridge detected\n");
}
