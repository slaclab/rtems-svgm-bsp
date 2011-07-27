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

  /*
   * ISA IRQ handler related definitions
   */
#define  BSP_ISA_IRQ_NUMBER             0  /* SVGM has no ISA PIC */
#define  BSP_ISA_IRQ_LOWEST_OFFSET      0
#define  BSP_ISA_IRQ_MAX_OFFSET         (BSP_ISA_IRQ_LOWEST_OFFSET + BSP_ISA_IRQ_NUMBER - 1)
  /*
   * PCI IRQ handlers related definitions
   * CAUTION : BSP_PCI_IRQ_LOWEST_OFFSET should be equal to OPENPIC_VEC_SOURCE
   */
#define  BSP_PCI_IRQ_NUMBER             16
#define  BSP_PCI_IRQ_LOWEST_OFFSET      16 /* _MUST_ be the same as OPENPIC_VEC_SOURCE */
#define  BSP_PCI_IRQ_MAX_OFFSET         (BSP_PCI_IRQ_LOWEST_OFFSET + BSP_PCI_IRQ_NUMBER - 1)
  /*
   * PowerPc exceptions handled as interrupt where a rtems managed interrupt
   * handler might be connected
   */
#define  BSP_PROCESSOR_IRQ_NUMBER        1
#define  BSP_PROCESSOR_IRQ_LOWEST_OFFSET (BSP_PCI_IRQ_MAX_OFFSET + 1)
#define  BSP_PROCESSOR_IRQ_MAX_OFFSET    (BSP_PROCESSOR_IRQ_LOWEST_OFFSET + BSP_PROCESSOR_IRQ_NUMBER - 1)

  /* allow a couple of vectors for VME and timer irq sources etc. */
#define  BSP_MISC_IRQ_NUMBER            30
#define  BSP_MISC_IRQ_LOWEST_OFFSET     (BSP_PROCESSOR_IRQ_MAX_OFFSET + 1)
#define  BSP_MISC_IRQ_MAX_OFFSET        (BSP_MISC_IRQ_LOWEST_OFFSET + BSP_MISC_IRQ_NUMBER - 1)

  /*
   * Summary
   */
#define  BSP_IRQ_NUMBER     (BSP_MISC_IRQ_MAX_OFFSET + 1)
#define  BSP_LOWEST_OFFSET  (BSP_ISA_IRQ_LOWEST_OFFSET)
#define  BSP_MAX_OFFSET	    (BSP_MISC_IRQ_MAX_OFFSET)
    /*
     * Some ISA IRQ symbolic name definition
     */	       

  /* on the SVGM these are actually PCI irqs */
#define  BSP_UART_COM2_IRQ  (BSP_PCI_IRQ_LOWEST_OFFSET + 14)

#define  BSP_UART_COM1_IRQ  (BSP_PCI_IRQ_LOWEST_OFFSET + 15)

    /*
     * Some PCI IRQ symbolic name definition
     */
#define  BSP_PCI_IRQ0               BSP_PCI_IRQ_LOWEST_OFFSET

  /* must never detect a ISA irq on the SVGM */
#undef   BSP_PCI_ISA_BRIDGE_IRQ
    /*
     * Some Processor execption handled as rtems IRQ symbolic name definition
     */
#define  BSP_DECREMENTER            BSP_PROCESSOR_IRQ_LOWEST_OFFSET

#if 0
#warning remove
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
#endif

/*
 * PIC-independent function to enable/disable interrupt lines at
 * the pic.
 */
extern void BSP_rtems_irq_mng_init(unsigned cpuId);

#include <bsp/irq_supp.h>

#ifdef __cplusplus
};
#endif

#endif

#endif
