/*
 * pci.c :  this file contains basic PCI Io functions.
 *
 *  CopyRight (C) 1999 valette@crf.canon.fr
 *
 *  This code is heavilly inspired by the public specification of STREAM V2
 *  that can be found at :
 *
 *      <http://www.chorus.com/Documentation/index.html> by following
 *  the STREAM API Specification Document link.
 *
 *  The license and distribution terms for this file may be
 *  found in found in the file LICENSE in this distribution or at
 *  http://www.OARcorp.com/rtems/license.html.
 *
 *  $Id$
 */

#include <bsp/consoleIo.h>
#include <libcpu/io.h>
#include <bsp/pci.h>
#include <bsp.h>

/* SVGM values */
#ifndef PCI_CONFIG_ADDR
#define PCI_CONFIG_ADDR			0xfec00000
#endif
#ifndef PCI_CONFIG_DATA
#define PCI_CONFIG_DATA			0xfee00000
#endif
#define PCI_INVALID_VENDORDEVICEID	0xffffffff
#define PCI_MULTI_FUNCTION		0x80

#define SHOW_GRACKLE_SETTINGS

/*
 * Bit encode for PCI_CONFIG_HEADER_TYPE register
 */
unsigned char ucMaxPCIBus;

static int
indirect_pci_read_config_byte(unsigned char bus, unsigned char slot,
			      unsigned char function, 
			      unsigned char offset, unsigned char *val) {
	out_be32((unsigned int*) pci.pci_config_addr, 
		 0x80|(bus<<8)|(PCI_DEVFN(slot,function)<<16)|((offset&~3)<<24));
	*val = in_8(pci.pci_config_data + (offset&3));
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_read_config_word(unsigned char bus, unsigned char slot,
			      unsigned char function, 
			      unsigned char offset, unsigned short *val) {
	*val = 0xffff; 
	if (offset&1) return PCIBIOS_BAD_REGISTER_NUMBER;
	out_be32((unsigned int*) pci.pci_config_addr, 
		 0x80|(bus<<8)|(PCI_DEVFN(slot,function)<<16)|((offset&~3)<<24));
	*val = in_le16((volatile unsigned short *)(pci.pci_config_data + (offset&3)));
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_read_config_dword(unsigned char bus, unsigned char slot,
			      unsigned char function, 
			      unsigned char offset, unsigned int *val) {
	*val = 0xffffffff; 
	if (offset&3) return PCIBIOS_BAD_REGISTER_NUMBER;
	out_be32((unsigned int*) pci.pci_config_addr, 
		 0x80|(bus<<8)|(PCI_DEVFN(slot,function)<<16)|(offset<<24));
	*val = in_le32((volatile unsigned int *)pci.pci_config_data);
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_write_config_byte(unsigned char bus, unsigned char slot,
			       unsigned char function, 
			       unsigned char offset, unsigned char val) {
	out_be32((unsigned int*) pci.pci_config_addr, 
		 0x80|(bus<<8)|(PCI_DEVFN(slot,function)<<16)|((offset&~3)<<24));
	out_8(pci.pci_config_data + (offset&3), val);
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_write_config_word(unsigned char bus, unsigned char slot,
			       unsigned char function, 
			       unsigned char offset, unsigned short val) {
	if (offset&1) return PCIBIOS_BAD_REGISTER_NUMBER;
	out_be32((unsigned int*) pci.pci_config_addr, 
		 0x80|(bus<<8)|(PCI_DEVFN(slot,function)<<16)|((offset&~3)<<24));
	out_le16((volatile unsigned short *)(pci.pci_config_data + (offset&3)), val);
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_write_config_dword(unsigned char bus, unsigned char slot,
				unsigned char function, 
				unsigned char offset, unsigned int val) {
	if (offset&3) return PCIBIOS_BAD_REGISTER_NUMBER;
	out_be32((unsigned int*) pci.pci_config_addr, 
		 0x80|(bus<<8)|(PCI_DEVFN(slot,function)<<16)|(offset<<24));
	out_le32((volatile unsigned int *)pci.pci_config_data, val);
	return PCIBIOS_SUCCESSFUL;
}

static const pci_config_access_functions indirect_functions = {
  	indirect_pci_read_config_byte,
  	indirect_pci_read_config_word,
  	indirect_pci_read_config_dword,
  	indirect_pci_write_config_byte,
  	indirect_pci_write_config_word,
  	indirect_pci_write_config_dword
};

pci_config pci = {(volatile unsigned char*)PCI_CONFIG_ADDR,
			 (volatile unsigned char*)PCI_CONFIG_DATA,
			 &indirect_functions};

static int
direct_pci_read_config_byte(unsigned char bus, unsigned char slot,
			    unsigned char function, 
			    unsigned char offset, unsigned char *val) {
	if (bus != 0 || (1<<slot & 0xff8007fe)) {
		*val=0xff;
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	*val=in_8(pci.pci_config_data + ((1<<slot)&~1) 
		  + (function<<8) + offset);
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_read_config_word(unsigned char bus, unsigned char slot,
			    unsigned char function, 
			    unsigned char offset, unsigned short *val) {
	*val = 0xffff; 
	if (offset&1) return PCIBIOS_BAD_REGISTER_NUMBER;
	if (bus != 0 || (1<<slot & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	*val=in_le16((volatile unsigned short *)
		     (pci.pci_config_data + ((1<<slot)&~1)
		      + (function<<8) + offset));
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_read_config_dword(unsigned char bus, unsigned char slot,
			     unsigned char function, 
			     unsigned char offset, unsigned int *val) {
	*val = 0xffffffff; 
	if (offset&3) return PCIBIOS_BAD_REGISTER_NUMBER;
	if (bus != 0 || (1<<slot & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	*val=in_le32((volatile unsigned int *)
		     (pci.pci_config_data + ((1<<slot)&~1)
		      + (function<<8) + offset));
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_write_config_byte(unsigned char bus, unsigned char slot,
			     unsigned char function, 
			     unsigned char offset, unsigned char val) {
	if (bus != 0 || (1<<slot & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	out_8(pci.pci_config_data + ((1<<slot)&~1) 
	      + (function<<8) + offset, 
	      val);
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_write_config_word(unsigned char bus, unsigned char slot,
			     unsigned char function, 
			     unsigned char offset, unsigned short val) {
	if (offset&1) return PCIBIOS_BAD_REGISTER_NUMBER;
	if (bus != 0 || (1<<slot & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	out_le16((volatile unsigned short *)
		 (pci.pci_config_data + ((1<<slot)&~1)
		  + (function<<8) + offset),
		 val);
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_write_config_dword(unsigned char bus, unsigned char slot,
			      unsigned char function, 
			      unsigned char offset, unsigned int val) {
	if (offset&3) return PCIBIOS_BAD_REGISTER_NUMBER;
	if (bus != 0 || (1<<slot & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	out_le32((volatile unsigned int *)
		 (pci.pci_config_data + ((1<<slot)&~1)
		  + (function<<8) + offset),
		 val);
	return PCIBIOS_SUCCESSFUL;
}

static const pci_config_access_functions direct_functions = {
  	direct_pci_read_config_byte,
  	direct_pci_read_config_word,
  	direct_pci_read_config_dword,
  	direct_pci_write_config_byte,
  	direct_pci_write_config_word,
  	direct_pci_write_config_dword
};


void detect_host_bridge()
{
  unsigned int id0;

  /*
   * This code assumes that the host bridge is located at
   * bus 0, dev 0, func 0 AND that the old pre PCI 2.1
   * standart devices detection mecahnism that was used on PC
   * (still used in BSD source code) works.
   */
  {
    /* Let us try by experimentation at our own risk! */
#if 0 /* T. Straumann, seems to need indirect_functions
       * I dont really understand this however...
       */
    pci.pci_functions = &direct_functions; 
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
	    BSP_panic("Host Bridget is not a MPC106/Grackle?????");
  }
  printk("Motorola MPC106/Grackle hostbridge detected\n");
}

/*
 * This routine determines the maximum bus number in the system
 */
void InitializePCI()
{
  unsigned char ucSlotNumber, ucFnNumber, ucNumFuncs;
  unsigned char ucHeader;
  unsigned char ucMaxSubordinate;
  unsigned int  ulClass, ulDeviceID;

  detect_host_bridge();
  /*
   * Scan PCI bus 0 looking for PCI-PCI bridges
   */
  for(ucSlotNumber=0;ucSlotNumber<PCI_MAX_DEVICES;ucSlotNumber++) {
    (void)pci_read_config_dword(0,
				ucSlotNumber,
				0,
				PCI_VENDOR_ID,
				&ulDeviceID);
    if(ulDeviceID==PCI_INVALID_VENDORDEVICEID) {
      /*
       * This slot is empty
       */
      continue;
    }
    (void)pci_read_config_byte(0,
			       ucSlotNumber,
			       0,
			       PCI_HEADER_TYPE,
			       &ucHeader);
    if(ucHeader&PCI_MULTI_FUNCTION)	{
      ucNumFuncs=PCI_MAX_FUNCTIONS;
    }
    else {
      ucNumFuncs=1;
    }
    for(ucFnNumber=0;ucFnNumber<ucNumFuncs;ucFnNumber++) {
      (void)pci_read_config_dword(0,
				  ucSlotNumber,
				  ucFnNumber,
				  PCI_VENDOR_ID,
				  &ulDeviceID);
      if(ulDeviceID==PCI_INVALID_VENDORDEVICEID) {
				/*
				 * This slot/function is empty
				 */
	continue;
      }

      /*
       * This slot/function has a device fitted.
       */
      (void)pci_read_config_dword(0,
				  ucSlotNumber,
				  ucFnNumber,
				  PCI_CLASS_REVISION,
				  &ulClass);
      ulClass >>= 16;
      if (ulClass == PCI_CLASS_BRIDGE_PCI) {
				/*
				 * We have found a PCI-PCI bridge
				 */
	(void)pci_read_config_byte(0,
				   ucSlotNumber,
				   ucFnNumber,
				   PCI_SUBORDINATE_BUS,
				   &ucMaxSubordinate);
	if(ucMaxSubordinate>ucMaxPCIBus) {
	  ucMaxPCIBus=ucMaxSubordinate;
	}
      }
    }
  }
}

/*
 * Return the number of PCI busses in the system
 */
unsigned char BusCountPCI()
{
  return(ucMaxPCIBus+1);
}
