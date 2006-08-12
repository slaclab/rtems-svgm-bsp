/* irq.h
 *
 *  This include file describe the data structure and the functions implemented
 *  by rtems to write interrupt handlers.
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
 *
 *  T. Straumann, 9/27/2001:
 *    - adapted shared/irq/irq.h for SVGM (no ISA devices/PIC)
 */

#ifndef LIBBSP_POWERPC_SVGM_IRQ_IRQ_H
#define LIBBSP_POWERPC_SVGM_IRQ_IRQ_H


#define BSP_ASM_IRQ_VECTOR_BASE 0x0

#ifndef ASM

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_SHARED_HANDLER_SUPPORT      1
#include <rtems/irq.h>


/*
 * Symbolic IRQ names and related definitions.
 */

/* leave the ISA symbols in there, so we can reuse shared/irq.c
 * Also, we start numbering PCI irqs at 16 because the OPENPIC
 * driver relies on this when mapping irq number <-> vectors
 * (OPENPIC_VEC_SOURCE in openpic.h)
 * Currently, we hack this away by redefining OPENPIC_VEC_SOURCE in bsp.h
 * which is included in openpic.c _after_ openpic.h
 */

typedef enum {
  /* Base vector for our ISA IRQ handlers. */
  BSP_ISA_IRQ_VECTOR_BASE	=	BSP_ASM_IRQ_VECTOR_BASE,
  /*
   * ISA IRQ handler related definitions
   */
  BSP_ISA_IRQ_NUMBER    	= 	0, /* SVGM has no ISA PIC */
  BSP_ISA_IRQ_LOWEST_OFFSET	= 	0,
  BSP_ISA_IRQ_MAX_OFFSET	= 	BSP_ISA_IRQ_LOWEST_OFFSET + BSP_ISA_IRQ_NUMBER - 1,
  /*
   * PCI IRQ handlers related definitions
   * CAUTION : BSP_PCI_IRQ_LOWEST_OFFSET should be equal to OPENPIC_VEC_SOURCE
   */
  BSP_PCI_IRQ_NUMBER		=	16,
  BSP_PCI_IRQ_LOWEST_OFFSET	=	16, /* _MUST_ be the same as OPENPIC_VEC_SOURCE */
  BSP_PCI_IRQ_MAX_OFFSET	= 	BSP_PCI_IRQ_LOWEST_OFFSET + BSP_PCI_IRQ_NUMBER - 1,
  /*
   * PowerPc exceptions handled as interrupt where a rtems managed interrupt
   * handler might be connected
   */
  BSP_PROCESSOR_IRQ_NUMBER	=	1,
  BSP_PROCESSOR_IRQ_LOWEST_OFFSET = 	BSP_PCI_IRQ_MAX_OFFSET + 1,
  BSP_PROCESSOR_IRQ_MAX_OFFSET	=	BSP_PROCESSOR_IRQ_LOWEST_OFFSET + BSP_PROCESSOR_IRQ_NUMBER - 1,

  /* allow a couple of vectors for VME and timer irq sources etc. */
  BSP_MISC_IRQ_NUMBER		=	30,
  BSP_MISC_IRQ_LOWEST_OFFSET	=	BSP_PROCESSOR_IRQ_MAX_OFFSET + 1,
  BSP_MISC_IRQ_MAX_OFFSET	=	BSP_MISC_IRQ_LOWEST_OFFSET + BSP_MISC_IRQ_NUMBER - 1,

  /*
   * Summary
   */
  BSP_IRQ_NUMBER		= 	BSP_MISC_IRQ_MAX_OFFSET + 1,
  BSP_LOWEST_OFFSET		=	BSP_ISA_IRQ_LOWEST_OFFSET,
  BSP_MAX_OFFSET		=	BSP_MISC_IRQ_MAX_OFFSET,
    /*
     * Some ISA IRQ symbolic name definition
     */	       

  /* on the SVGM these are actually PCI irqs */
  BSP_UART_COM2_IRQ		=	BSP_PCI_IRQ_LOWEST_OFFSET + 14,

  BSP_UART_COM1_IRQ		=	BSP_PCI_IRQ_LOWEST_OFFSET + 15,

    /*
     * Some PCI IRQ symbolic name definition
     */
  BSP_PCI_IRQ0			=	BSP_PCI_IRQ_LOWEST_OFFSET,

  /* must never detect a ISA irq on the SVGM */
  BSP_PCI_ISA_BRIDGE_IRQ	=	0xdeadbeef,
    /*
     * Some Processor execption handled as rtems IRQ symbolic name definition
     */
  BSP_DECREMENTER		=	BSP_PROCESSOR_IRQ_LOWEST_OFFSET
  
}rtems_irq_symbolic_name;

/*
 * Type definition for RTEMS managed interrupts
 */
typedef unsigned short rtems_i8259_masks;

/*-------------------------------------------------------------------------+
| Ignore some stuff on the SVGM
+--------------------------------------------------------------------------*/
static inline int BSP_irq_disable_at_i8259s(irqLine)	{ return 0; }
static inline int BSP_irq_enable_at_i8259s(irqLine)		{ return 0; }
static inline int BSP_irq_ack_at_i8259s(irqLine)		{ return 0; }
static inline int BSP_irq_enabled_at_i8259s(irqLine)	{ return 0; }
#define i8259s_cache (*(rtems_i8259_masks *)(0))

/*
 * PIC-independent function to enable/disable interrupt lines at
 * the pic.
 */
extern void BSP_enable_irq_at_pic		(const rtems_irq_number irqLine);
extern void BSP_disable_irq_at_pic		(const rtems_irq_number irqLine);

extern int BSP_setup_the_pic			(rtems_irq_global_settings* config);
extern void BSP_rtems_irq_mng_init(unsigned cpuId);

#ifdef __cplusplus
};
#endif

#endif

#endif
