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

/*
 * Symbolic IRQ names and related definitions.
 */

/* leave the ISA symbols in there, so we can reuse shared/irq.c
 * Also, we start numbering PCI irqs at 16 because the OPENPIC
 * driver relies on this when mapping irq number <-> vectors
 * (OPENPIC_VEC_SOURCE in openpic.h)
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
  BSP_ISA_UART_COM2_IRQ		=	BSP_PCI_IRQ_LOWEST_OFFSET + 14,

  BSP_ISA_UART_COM1_IRQ		=	BSP_PCI_IRQ_LOWEST_OFFSET + 15,

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
typedef unsigned char  rtems_irq_prio;
typedef unsigned short rtems_i8259_masks;


struct 	__rtems_irq_connect_data__;	/* forward declaratiuon */

typedef void (*rtems_irq_hdl)		(void);
typedef void (*rtems_irq_enable)	(const struct __rtems_irq_connect_data__*);
typedef void (*rtems_irq_disable)	(const struct __rtems_irq_connect_data__*);
typedef int  (*rtems_irq_is_enabled)	(const struct __rtems_irq_connect_data__*);

typedef struct __rtems_irq_connect_data__ {
      /*
       * IRQ line
       */
      rtems_irq_symbolic_name 	name;
      /*
       * handler. See comment on handler properties below in function prototype.
       */
      rtems_irq_hdl	   		hdl;
      /*
       * function for enabling interrupts at device level (ONLY!).
       * The BSP code will automatically enable it at i8259s level and openpic level.
       * RATIONALE : anyway such code has to exist in current driver code.
       * It is usually called immediately AFTER connecting the interrupt handler.
       * RTEMS may well need such a function when restoring normal interrupt
       * processing after a debug session.
       * 
       */
      rtems_irq_enable		on;	
      /*
       * function for disabling interrupts at device level (ONLY!).
       * The code will disable it at i8259s level. RATIONALE : anyway
       * such code has to exist for clean shutdown. It is usually called
       * BEFORE disconnecting the interrupt. RTEMS may well need such
       * a function when disabling normal interrupt processing for
       * a debug session. May well be a NOP function.
       */
      rtems_irq_disable		off;
      /*
       * function enabling to know what interrupt may currently occur
       * if someone manipulates the i8259s interrupt mask without care...
       */
      rtems_irq_is_enabled	isOn;
      /*
       *  Set to -1 for vectors forced to have only 1 handler
       */
      void *next_handler;

}rtems_irq_connect_data;

typedef struct {
  /*
   * size of all the table fields (*Tbl) described below.
   */
  unsigned int	 		irqNb;
  /*
   * Default handler used when disconnecting interrupts.
   */
  rtems_irq_connect_data	defaultEntry;
  /*
   * Table containing initials/current value.
   */
  rtems_irq_connect_data*	irqHdlTbl;
  /*
   * actual value of BSP_ISA_IRQ_VECTOR_BASE...
   */
  rtems_irq_symbolic_name	irqBase;
  /*
   * software priorities associated with interrupts.
   * if irqPrio  [i]  >  intrPrio  [j]  it  means  that  
   * interrupt handler hdl connected for interrupt name i
   * will  not be interrupted by the handler connected for interrupt j
   * The interrupt source  will be physically masked at i8259 level.
   */
    rtems_irq_prio*		irqPrioTbl;
}rtems_irq_global_settings;




/*-------------------------------------------------------------------------+
| Ignore some stuff on the SVGM
+--------------------------------------------------------------------------*/
static inline int BSP_irq_disable_at_i8259s(irqLine)	{ return 0; }
static inline int BSP_irq_enable_at_i8259s(irqLine)		{ return 0; }
static inline int BSP_irq_ack_at_i8259s(irqLine)		{ return 0; }
static inline int BSP_irq_enabled_at_i8259s(irqLine)	{ return 0; }
#define i8259s_cache (*(rtems_i8259_masks *)(0))

/*
 * ------------------------ RTEMS Single Irq Handler Mngt Routines ----------------
 */
/*
 * function to connect a particular irq handler. This hanlder will NOT be called
 * directly as the result of the corresponding interrupt. Instead, a RTEMS
 * irq prologue will be called that will :
 *
 *	1) save the C scratch registers,
 *	2) switch to a interrupt stack if the interrupt is not nested,
 *	3) store the current i8259s' interrupt masks
 *	4) modify them to disable the current interrupt at 8259 level (and may
 *	be others depending on software priorities)
 *	5) aknowledge the i8259s',
 *	6) demask the processor,
 *	7) call the application handler
 *
 * As a result the hdl function provided
 *
 *	a) can perfectly be written is C,
 * 	b) may also well directly call the part of the RTEMS API that can be used
 *	from interrupt level,
 *	c) It only responsible for handling the jobs that need to be done at
 *	the device level including (aknowledging/re-enabling the interrupt at device,
 *	level, getting the data,...)
 *
 *	When returning from the function, the following will be performed by
 *	the RTEMS irq epilogue :
 *
 *	1) masks the interrupts again,
 *	2) restore the original i8259s' interrupt masks
 *	3) switch back on the orinal stack if needed,
 *	4) perform rescheduling when necessary,
 *	5) restore the C scratch registers...
 *	6) restore initial execution flow
 * 
 */
int BSP_install_rtems_irq_handler   	(const rtems_irq_connect_data*);
int BSP_install_rtems_shared_irq_handler  (const rtems_irq_connect_data*);

#define BSP_SHARED_HANDLER_SUPPORT      1

/*
 * function to get the current RTEMS irq handler for ptr->name. It enables to
 * define hanlder chain...
 */
int BSP_get_current_rtems_irq_handler	(rtems_irq_connect_data* ptr);
/*
 * function to get disconnect the RTEMS irq handler for ptr->name.
 * This function checks that the value given is the current one for safety reason.
 * The user can use the previous function to get it.
 */
int BSP_remove_rtems_irq_handler    	(const rtems_irq_connect_data*);

/*
 * ------------------------ RTEMS Global Irq Handler Mngt Routines ----------------
 */
/*
 * (Re) Initialize the RTEMS interrupt management.
 *
 * The result of calling this function will be the same as if each individual
 * handler (config->irqHdlTbl[i].hdl)  different from "config->defaultEntry.hdl"
 * has been individualy connected via
 *	BSP_install_rtems_irq_handler(&config->irqHdlTbl[i])
 * And each handler currently equal to config->defaultEntry.hdl
 * has been previously disconnected via
 * 	 BSP_remove_rtems_irq_handler (&config->irqHdlTbl[i])
 *
 * This is to say that all information given will be used and not just
 * only the space.
 *
 * CAUTION : the various table address contained in config will be used
 *	     directly by the interrupt mangement code in order to save
 *	     data size so they must stay valid after the call => they should
 *	     not be modified or declared on a stack.
 */

int BSP_rtems_irq_mngt_set(rtems_irq_global_settings* config);
/*
 * (Re) get info on current RTEMS interrupt management.
 */
int BSP_rtems_irq_mngt_get(rtems_irq_global_settings**);
  
extern void BSP_rtems_irq_mng_init(unsigned cpuId);
#endif

#endif
