/*
 *  This routine starts the application.  It includes application,
 *  board, and monitor specific initialization and configuration.
 *  The generic CPU dependent initialization has been performed
 *  before this routine is invoked.
 *
 *  COPYRIGHT (c) 1989-1998.
 *  On-Line Applications Research Corporation (OAR).
 *  Copyright assigned to U.S. Government, 1994.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.OARcorp.com/rtems/license.html.
 *
 *  Modified to support the MCP750.
 *  Modifications Copyright (C) 1999 Eric Valette. valette@crf.canon.fr
 *
 *  Modified to support the Synergy VGM series of boards
 *  (C) by Till Straumann, <strauman@slac.stanford.edu>, 2002
 *
 *  $Id$
 */

#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include <bsp.h>
#include <rtems/libio.h>
#include <rtems/libcsupport.h>
#include <bsp/uart.h>
#include <libcpu/spr.h>
#include <bsp/pci.h>
#include <bsp/openpic.h>
#include <bsp/irq.h>
#include <libcpu/bat.h>
#include <libcpu/page.h>
#include <libcpu/pte121.h>
#include <libcpu/cpuIdent.h>

#include <bsp/vectors.h>
#include <bsp/bspVGM.h>
#include <bsp/bspException.h>
#include <synergyregs.h>

/* for RTEMS_VERSION :-( I dont like the preassembled string */
#include <rtems/sptables.h>

#ifdef __RTEMS_APPLICATION__
#undef __RTEMS_APPLICATION__
#endif

/*
 * system init stack and soft ir stack size
 */
#define INIT_STACK_SIZE 0x1000
#define INTR_STACK_SIZE CONFIGURE_INTERRUPT_STACK_MEMORY


#define USE_BOOTP_STUFF
#undef   SHOW_MORE_INIT_SETTINGS


#include "bspstart_common.c"
#include "cmdline.c"

/* a couple of declarations we have no header for :-( */
void
_BSP_pciCacheInit();

void
_BSP_pciIRouteFixup();

BSP_output_char_function_type BSP_output_char = BSP_output_char_via_serial;

/* missing bits... */

/* Reminder: useful setdbat0 code skeleton can be found at the bottom */

/* L2CR bits */
#define L2CR_L2E		(1<<31)			/* enable */
#define L2CR_L2PE		(1<<30)			/* parity enable */
#define L2CR_L2SIZ(a)	(((a)&3)<<28)	/* size: 0=2MB, 3=1MB */
#define L2CR_L2CLK(a)	(((a)&7)<<25)	/* clock speed  1=1:1, 2=1:1.5, 4=2:1, 5=2:5, 6=3:1 */
#define L2CR_L2RAM(a)	(((a)&3)<<23)	/* 3=pipelined sync, late-write */	
#define L2CR_L2DO		(1<<22)			/* data only */
#define L2CR_L2I		(1<<21)			/* global invalidate */
#define L2CR_L2CTL		(1<<20)			/* ZZ enable */
#define L2CR_L2WT		(1<<19)			/* write through */
#define L2CR_L2TS		(1<<18)			/* test mode */
#define L2CR_L2OH(a)	(((a)&3)<<16)	/* output hold; 0=0.5ns, 1=1ns */
#define L2CR_L2SL		(1<<15)			/* slow */
#define L2CR_L2DF		(1<<14)			/* differential clock */
#define L2CR_L2BYP		(1<<13)			/* bypass */
#define L2CR_L2IP		(1<<0)			/* invalidate in progress */

SPR_RW(DABR)
SPR_RW(HID0)
SPR_RW(SPRG0)
SPR_RW(SPRG1)

extern void		L1_caches_enables();
extern unsigned get_L2CR();
extern unsigned set_L2CR(unsigned);
extern void		bsp_cleanup(void);

/*
 * Vital Board data obtained from VGM board registers
 */
/*
 * Total memory
 */
unsigned int BSP_mem_size;
/*
 * CPU/PPC Bus Frequency
 */
unsigned int BSP_bus_frequency;
/*
 * processor clock frequency
 */
unsigned int BSP_processor_frequency;
/*
 * Time base divisior (how many tick for 1 second).
 */
unsigned int BSP_time_base_divisor;

/* flags are: 4 BT (must be MSR[DR])
 *            2 break on store
 *            1 break on load
 */
void
BSP_set_DABR(void *addr, int flags)
{
	printk("BSP: setting breakpoint at 0x%08x\n",addr);
	__asm__ __volatile__("sync");
	_write_DABR((((long)addr)&~7) | (flags & 7));
	__asm__ __volatile__("isync");
}

/*
 *  The original table from the application and our copy of it with
 *  some changes.
 */

extern rtems_configuration_table Configuration;

rtems_configuration_table  BSP_Configuration;

rtems_cpu_table Cpu_table;

char *rtems_progname;

/*
 *  Use the shared implementations of the following routines
 */
 
void			bsp_postdriver_hook(void);

Triv121PgTbl	BSP_pgtbl_setup(unsigned int*);
void			BSP_pgtbl_activate(Triv121PgTbl);

/* bsp_postdriver_hook() opens the console for stdio;
 * we also want 'special' BREAK processing, so we can
 * reset the board by sending a BREAK to the console...
 */
static void		svgmPostdriverHook(void)
{
BSP_UartBreakCbRec cb;
	/* do standard init (open console) */
	bsp_postdriver_hook();
	/* stdin should be fd 0 now */
	cb.handler = (BSP_UartBreakCbProc)rtemsReboot;
	cb.private = 0;
	ioctl(0,BIOCSETBREAKCB,&cb);
}


/*
 *  bsp_start
 *
 *  This routine does the bulk of the system initialization.
 */

void bsp_start( void )
{
  unsigned char				*stack;
  unsigned long				*r1sp;
  unsigned					l2cr;
  register unsigned char*	intrStack;
  unsigned char				*work_space_start;
  ppc_cpu_id_t				myCpu;
  ppc_cpu_revision_t		myCpuRevision;
  unsigned char				reg;
  const unsigned char		*chpt;
  Triv121PgTbl				pt;

  /*
   * Get CPU identification dynamically. Note that the get_ppc_cpu_type() function
   * store the result in global variables so that it can be used latter...
   */
  myCpu 		= get_ppc_cpu_type();
  myCpuRevision = get_ppc_cpu_revision();

  /*
   * Access to board registers and PCI devices.
   *
   * Luckily, SMON configures PCI devices and maps their
   * memory areas starting at 0xf0000000 (PCI addr)
   */
  setdbat(1, 0xf0000000, 0xf0000000, 0x10000000, IO_PAGE);


  /*
   * enables L1 Cache. Note that the L1_caches_enables() codes checks for
   * relevant CPU type so that the reason why there is no use of myCpu...
   */
  L1_caches_enables();

  /*
   * Enable L2 Cache. Note that the set_L2CR(L2CR) codes checks for
   * relevant CPU type (mpc750)...
   *
   * It also takes care of flushing the cache under certain conditions:
   *   current    going to (E==enable, I==invalidate)
   *     E           E | I	-> __NOT_FLUSHED_, invalidated, stays E
   *     E               I	-> flush & disable, invalidate
   *     E           E		-> nothing, stays E
   *     0           E | I	-> not flushed, invalidated, enabled
   *     0             | I	-> not flushed, invalidated, stays off
   *     0           E      -> not flushed, _NO_INVALIDATE, enabled
   *
   * The first and the last combinations are potentially dangerous!
   *
   * NOTE: we assume the essential cache parameters (speed, size etc.)
   *       have been set correctly by the SMON firmware!
   *
   */
  l2cr = get_L2CR();
  if ( -1 != (int)l2cr ) {
	/* -1 would mean that this machine doesn't support L2 */

	l2cr &= ~L2CR_L2DO; /* clear 'data only' */
	if ( ! (l2cr & L2CR_L2E) ) {
		/* we are going to enable the L2 - hence we
		 * MUST invalidate it first; however, if
		 * it was enabled already, we MUST NOT
		 * invalidate it!!
		 */
		l2cr |= L2CR_L2E | L2CR_L2I;
	}
	l2cr=set_L2CR(l2cr);
  }

  /*
   * the initial stack  has already been set to this value in start.S
   * so there is no need to set it in r1 again... It is just for info
   * so that It can be printed without accessing R1.
   */
  stack = ((unsigned char*) __rtems_end) + INIT_STACK_SIZE - CPU_MINIMUM_STACK_FRAME_SIZE;

  /* tag the bottom, so a stack trace utility may know when to stop */
  *((unsigned32 *)stack) = 0;

  /* fill stack with pattern for debugging */
  __asm__ __volatile__("mr %0, %%r1":"=r"(r1sp));
  while (--r1sp >= (unsigned long*)__rtems_end)
	  *r1sp=0xeeeeeeee;

  /*
   * Initialize the interrupt related settings
   * SPRG1 = software managed IRQ stack
   *
   * This could be done latter (e.g in IRQ_INIT) but it helps to understand
   * some settings below...
   */
  intrStack = ((unsigned char*) __rtems_end) + INIT_STACK_SIZE + INTR_STACK_SIZE - CPU_MINIMUM_STACK_FRAME_SIZE;

  /* make sure it's properly aligned */
  (unsigned32)intrStack &= ~(CPU_STACK_ALIGNMENT-1);

  /* tag the bottom of the interrupt stack as well */
  *((unsigned32 *)intrStack) = 0;
  /* fill interrupt stack with pattern for debugging */
  r1sp=(unsigned long*)intrStack;
  while (--r1sp >= (unsigned long*)( intrStack - (INTR_STACK_SIZE - 8)))
	  *r1sp=0xeeeeeeee;

  _write_SPRG1((unsigned int)intrStack);

  /* signal them that we have fixed PR288 - eventually, this should go away */
  _write_SPRG0(PPC_BSP_HAS_FIXED_PR288);

  /*
   * Initialize default raw exception handlers. See vectors/vectors_init.c
   */
  Cpu_table.exceptions_in_RAM 	 = TRUE;
  initialize_exceptions();

  if (!(chpt=BSP_boardType())) {
		  BSP_panic("Unknown Synergy Board Type");
  }

  printk("-----------------------------------------\n");
  printk("Welcome to RTEMS RELEASE %s/svgm on %s/%s/%s\n",
		  RTEMS_VERSION, chpt, CPU_NAME, get_ppc_cpu_type_name(myCpu));
  printk("SSRL Release $Name$\n");
  printk("Build Date: %s\n",BSP_build_date);
  printk("-----------------------------------------\n");
#ifdef SHOW_MORE_INIT_SETTINGS  
  printk("Initial system stack at 0x%08x\n",stack);
  __asm__ __volatile__ ("mr %0, %%r1":"=r"(stack));
  printk("(R1 stack pointer is    0x%08x)\n", stack);
  printk("Software IRQ stack at   0x%08x\n",intrStack);
  printk("Initial L2CR value is   0x%08x\n", l2cr);
  printk("-----------------------------------------\n");
#endif

#ifdef SHOW_MORE_INIT_SETTINGS
  printk("Going to start PCI buses scanning and initialization\n");
#endif  
  /* Disable MCP interrupts at CPU level; scanning the PCI configuration space
   * will result in master-aborts.
   */

  _write_HID0(_read_HID0() & ~ HID0_EMCP);

  /* initialize pci driver. We just supply the SVGM's 
   * config_addr / config_data addresses here
   */
  InitializePCI();

  _BSP_pciIRouteFixup();

  /* Install our own exception handler (needs PCI) */
  globalExceptHdl = BSP_exceptionHandler;

  /* now build the pci device cache which supports BSP_pciFindDevice() */
  _BSP_pciCacheInit();
  /* read board info registers */
  reg = *SYN_VGM_REG_INFO_MEMORY;
  BSP_mem_size 				= 
		SYN_VGM_REG_INFO_MEMORY_BANKS(reg) * 	/* number of banks */
		SYN_VGM_REG_INFO_MEMORY_BANK_SIZE(reg); /* bank size */

  reg = *SYN_VGM_REG_STAT_BOARD;
  switch (reg & SYN_VGM_REG_STAT_CPU_BUS_SPEED_MSK) {
		  default:
		  case SYN_VGM_REG_STAT_CPU_BUS_SPEED_66:
				BSP_bus_frequency = 66666667; break;
		  case SYN_VGM_REG_STAT_CPU_BUS_SPEED_83:
				BSP_bus_frequency = 83333333; break;
		  case SYN_VGM_REG_STAT_CPU_BUS_SPEED_100:
				BSP_bus_frequency = 100000000; break;
		  case SYN_VGM_REG_STAT_CPU_BUS_SPEED_133:
				BSP_bus_frequency = 133333333; break;
  } 
  BSP_processor_frequency	= BSP_getCpuClock(BSP_bus_frequency);
  BSP_time_base_divisor		= 4000; /* 750 and 7400 clock the TB / DECR at 1/4 of the CPU speed */
  BSPBaseBaud				= 9600*156; /* TODO, found by experiment */
  
  /* and finally clear the hostbridge errors and enable MCP exception
   * generation. Note that config space access to non-existent devices
   * results in a master abort
   *
   * (call this routine only after the CPU table has been initialized;
   * it uses rtems_bsp_delay())
   *
   * Call twice; once silently to avoid printing the errors
   * caused by the PCI config space scan. The second time
   * to inform the user that MCP/TEA interrupts will be enabled.
   */
  _BSP_clear_hostbridge_errors(0/*enableMCP*/,1/*quiet*/);
  _BSP_clear_hostbridge_errors(1/*enableMCP*/,0/*quiet*/);

  /* enable MCP interrupt (TEA is always on) */
  _write_HID0(_read_HID0() | HID0_EMCP);

  /* Allocate and set up the page table mappings.
   * This is done in an extra file giving applications
   * a chance to override the default mappings :-)
   *
   * NOTE: This setup routine may modify the available memory
   *       size. It is essential to call it before
   *       calculating the workspace etc.
   */
  pt = BSP_pgtbl_setup(&BSP_mem_size);

  /*
   * Set up our hooks
   * Make sure libc_init is done before drivers initialized so that
   * they can use atexit()
   */

  Cpu_table.pretasking_hook 	 = bsp_pretasking_hook;    /* init libc, etc. */
  Cpu_table.postdriver_hook 	 = svgmPostdriverHook;
  Cpu_table.do_zero_of_workspace = TRUE;
  Cpu_table.interrupt_stack_size = CONFIGURE_INTERRUPT_STACK_MEMORY;
  /* TB is clocked by PPC bus clock / timebase divisor */
  Cpu_table.clicks_per_usec 	 = BSP_bus_frequency/(BSP_time_base_divisor * 1000);

  /* did they pass a workspace size on the commandline ? */
  {
  long size=0;
  CmdLine cmdline = (CmdLine)heapStart();
  printk("bspstart: **** GOT COMMANDLINE: >>>%s<<<\n",cmdline->buf);
  if ( (chpt = strstr(cmdline->buf, "WSPC=")) ) {
    /* strip quotes */
    for ( chpt+=5; '\''==*chpt; chpt++ )
      /* nothing else to do */;
    size = strtol(chpt, 0, 0); 
  }
  if ( size ) {
    printk("Allocating %i bytes of workspace as requested from cmdline\n", size);
    Configuration.work_space_size = BSP_Configuration.work_space_size = size;
  }
  }

  work_space_start = 
    (unsigned char *)BSP_mem_size - BSP_Configuration.work_space_size;

  if ( work_space_start <= ((unsigned char *)__rtems_end) + INIT_STACK_SIZE + INTR_STACK_SIZE) {
    printk( "bspstart: Not enough RAM!!!\n" );
    bsp_cleanup();
  }

  BSP_Configuration.work_space_start = work_space_start;

  /*
   * Initalize RTEMS IRQ system (including the openPIC)
   */ 

  BSP_rtems_irq_mng_init(0);

  /* Activate the page table mappings only after
   * initializing interrupts because the irq_mng_init()
   * routine needs to modify the text
   */
  if (pt) {
#ifdef  SHOW_MORE_INIT_SETTINGS
	printk("Page table setup finished; will activate it NOW...\n");
#endif
  	BSP_pgtbl_activate(pt);
  }
}
