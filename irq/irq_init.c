/* irq_init.c
 *
 *  This file contains the implementation of rtems initialization
 *  related to interrupt handling.
 *
 *  CopyRight (C) 1999 valette@crf.canon.fr
 *
 * Enhanced by Jay Kulpinski <jskulpin@eng01.gdds.com>
 * to make it valid for MVME2300 Motorola boards.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.OARcorp.com/rtems/license.html.
 *
 *  $Id$
 */
#include <bsp/consoleIo.h>
#include <libcpu/io.h>
#include <libcpu/spr.h>
#include <bsp/pci.h>
#include <bsp/openpic.h>
#include <bsp/irq.h>
#include <bsp.h>
#include <libcpu/raw_exception.h>

/*
#define SHOW_ISA_PCI_BRIDGE_SETTINGS
*/
#define TRACE_IRQ_INIT  
#undef  TRACE_IRQ_INIT  

extern unsigned int external_exception_vector_prolog_code_size[];
extern void external_exception_vector_prolog_code();
extern unsigned int decrementer_exception_vector_prolog_code_size[];
extern void decrementer_exception_vector_prolog_code();

/*
 * default on/off function
 */
static void nop_func(){}
/*
 * default isOn function
 */
static int not_connected() {return 0;}
/*
 * default possible isOn function
 */
static int connected() {return 1;}

static rtems_irq_connect_data     	rtemsIrq[BSP_IRQ_NUMBER];
static rtems_irq_global_settings     	initial_config;
static rtems_irq_connect_data     	defaultIrq = {
  /* vectorIdex,	 hdl		, on		, off		, isOn */
  0, 			 nop_func	, nop_func	, nop_func	, not_connected
};
static rtems_irq_prio irqPrioTable[BSP_IRQ_NUMBER]={
  /*
   * actual rpiorities for interrupt :
   *	0   means that only current interrupt is masked
   *	255 means all other interrupts are masked
   */
  /*
   * PCI Interrupts
   */
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, /* for raven prio 0 means unactive... */
  /*
   * Processor exceptions handled as interrupts
   */
  0
};


  /*
   * This code assumes the exceptions management setup has already
   * been done. We just need to replace the exceptions that will
   * be handled like interrupt. On mcp750/mpc750 and many PPC processors
   * this means the decrementer exception and the external exception.
   */
void BSP_rtems_irq_mng_init(unsigned cpuId)
{
  rtems_raw_except_connect_data vectorDesc;
  int i,bus,dev,fn;
#ifdef TRACE_IRQ_INIT  
  printk("Going to scan the PCI bus for the IBM MPIC\n");
#endif       
  if ( BSP_pciFindDevice(PCI_VENDOR_ID_IBM,
	  		PCI_DEVICE_ID_IBM_MPIC,
			0,
			&bus,
			&dev,
			&fn) ||
	pci_read_config_dword(bus,dev,fn,PCI_BASE_ADDRESS_0,(unsigned int*)&OpenPIC) ||
        ! OpenPIC ) {
	  BSP_panic("Unable to find MPIC");
  }

  /* On the VGM series, the OpenPIC timer frequency is hardcoded to the
   * PCI bus speed (33MHz) / 8
   * Set it up now, so openpic_init() reports this value.
   */
  out_le32(&OpenPIC->Global.Timer_Frequency, 33333333/8);

  
  /*
   * First initialize the Interrupt management hardware
   */
#ifdef TRACE_IRQ_INIT  
  printk("Going to initialize interrupt controller (openpic compliant)\n");
#endif       
  /* use default polarity (0) and senses (1==level) */
  openpic_init(1,0,0);

#ifdef TRACE_IRQ_INIT  
  printk("Going to re-initialize the rtemsIrq table\n");
#endif       
  /*
   * Initialize Rtems management interrupt table
   */
    /*
     * re-init the rtemsIrq table
     */
    for (i = 0; i < BSP_IRQ_NUMBER; i++) {
      rtemsIrq[i]      = defaultIrq;
      rtemsIrq[i].name = i;
    }
    /*
     * Init initial Interrupt management config
     */
    initial_config.irqNb 	= BSP_IRQ_NUMBER;
    initial_config.defaultEntry = defaultIrq;
    initial_config.irqHdlTbl	= rtemsIrq;
    initial_config.irqBase	= BSP_ASM_IRQ_VECTOR_BASE;
    initial_config.irqPrioTbl	= irqPrioTable;

#ifdef TRACE_IRQ_INIT  
  printk("Going to setup irq mngt configuration\n");
#endif       

    if (!BSP_rtems_irq_mngt_set(&initial_config)) {
      /*
       * put something here that will show the failure...
       */
      BSP_panic("Unable to initialize RTEMS interrupt Management!!! System locked\n");
    }
    /* is a normal PCI irq on the SVGM;
     * irq 0 had been enabled by BSP_rtems_irq_mngt_set()
     * assuming that an ISA irq controller is hooked up there...
     */
    openpic_disable_irq(0);
  
  /*
   * We must connect the raw irq handler for the two
   * expected interrupt sources : decrementer and external interrupts.
   */
    vectorDesc.exceptIndex 	=	ASM_DEC_VECTOR;
    vectorDesc.hdl.vector	=	ASM_DEC_VECTOR;
    vectorDesc.hdl.raw_hdl	=	decrementer_exception_vector_prolog_code;
    vectorDesc.hdl.raw_hdl_size	=	(unsigned) decrementer_exception_vector_prolog_code_size;
    vectorDesc.on		=	nop_func;
    vectorDesc.off		=	nop_func;
    vectorDesc.isOn		=	connected;
    if (!mpc60x_set_exception (&vectorDesc)) {
      BSP_panic("Unable to initialize RTEMS decrementer raw exception\n");
    }
    vectorDesc.exceptIndex	=	ASM_EXT_VECTOR;
    vectorDesc.hdl.vector	=	ASM_EXT_VECTOR;
    vectorDesc.hdl.raw_hdl	=	external_exception_vector_prolog_code;
    vectorDesc.hdl.raw_hdl_size	=	(unsigned) external_exception_vector_prolog_code_size;
    if (!mpc60x_set_exception (&vectorDesc)) {
      BSP_panic("Unable to initialize RTEMS external raw exception\n");
    }
#ifdef TRACE_IRQ_INIT  
    printk("RTEMS IRQ management is now operationnal\n");
#endif
}
