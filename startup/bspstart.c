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

#include <bsp.h>
#include <rtems/libio.h>
#include <rtems/libcsupport.h>
#include <bsp/consoleIo.h>
#include <bsp/uart.h>
#include <libcpu/spr.h>
#include <bsp/pci.h>
#include <bsp/openpic.h>
#include <bsp/irq.h>
#include <libcpu/bat.h>
#include <libcpu/page.h>

/* bat valid bits */
#define  BAT_VS 2
#define  BAT_VP 1

#include <bsp/vectors.h>
#include <bsp/VME.h>
#include <bsp/bspVGM.h>
#include <synergyregs.h>
#include <pte121.h>

#ifdef __RTEMS_APPLICATION__
#undef __RTEMS_APPLICATION__
#endif
/* there is no public Workspace_Free() variant :-( */
#include <rtems/score/wkspace.h>

extern void _return_to_ppcbug();
extern unsigned long __rtems_end;
extern unsigned long _end;
extern void L1_caches_enables();
extern unsigned get_L2CR();
extern void set_L2CR(unsigned);
extern void bsp_cleanup(void);

typedef struct CmdLineRec_ {
		unsigned long	size;
		char			buf[0];
} CmdLineRec, *CmdLine;

#define  SHOW_MORE_INIT_SETTINGS

/*
 * Vital Board data Start using DATA RESIDUAL
 */
/*
 * Total memory using RESIDUAL DATA
 */
unsigned int BSP_mem_size;
/*
 * PCI Bus Frequency
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
/*
 * system init stack and soft ir stack size
 */
#define INIT_STACK_SIZE 0x1000
#define INTR_STACK_SIZE CONFIGURE_INTERRUPT_STACK_MEMORY

static unsigned long
heapStart(void)
{
unsigned long rval;
    rval = ((rtems_unsigned32) &__rtems_end) +INIT_STACK_SIZE + INTR_STACK_SIZE;
    if (rval & (CPU_ALIGNMENT-1))
        rval = (rval + CPU_ALIGNMENT) & ~(CPU_ALIGNMENT-1);
	return rval;
}

void BSP_panic(char *s)
{
  printk("%s PANIC %s\n",_RTEMS_version, s);
  __asm__ __volatile ("sc"); 
}

void _BSP_Fatal_error(unsigned int v)
{
  printk("%s PANIC ERROR %x\n",_RTEMS_version, v);
  __asm__ __volatile ("sc"); 
}

/* flags are: 4 BT (must be MSR[DR])
 *            2 break on store
 *            1 break on load
 */
void
BSPsetDABR(void *addr, int flags)
{
	printk("BSP: setting breakpoint at 0x%08x\n",addr);
	__asm__ __volatile__("sync; mtspr 1013, %0; isync"::"r"((((long)addr)&~7) | (flags & 7)));
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
 
void bsp_postdriver_hook(void);
void bsp_libc_init( void *, unsigned32, int );

void BSP_vme_config(void);

/*
 *  Function:   bsp_pretasking_hook
 *  Created:    95/03/10
 *
 *  Description:
 *      BSP pretasking hook.  Called just before drivers are initialized.
 *      Used to setup libc and install any BSP extensions.
 *
 *  NOTES:
 *      Must not use libc (to do io) from here, since drivers are
 *      not yet initialized.
 *
 */
 
void bsp_pretasking_hook(void)
{
    rtems_unsigned32        heap_start=heapStart();    
	CmdLine					cmdline=(CmdLine)heap_start;
    rtems_unsigned32        heap_size;
	char					*buf;

    heap_size = (BSP_mem_size - heap_start) - BSP_Configuration.work_space_size;

	if (cmdline->size > heap_size - (cmdline->buf-(char*)cmdline)) {
			/* line was truncated by the workspace area; */
			printk("WARNING: huge commandline overlaps with workspace, truncated\n");
			cmdline->size=heap_size - (cmdline->buf-(char*)cmdline);
			cmdline->buf[cmdline->size-1]=0;
	}
	/* allocate workspace for the commandline */
	buf=_Workspace_Allocate(cmdline->size);
	if (!buf) {
			printk("WARNING: not enough workspace for commandline, dropping it\n");
	} else {
			memcpy(buf,cmdline->buf,cmdline->size);
	}

#ifdef SHOW_MORE_INIT_SETTINGS
    printk(" HEAP start %x  size %x\n", heap_start, heap_size);
#endif    
    bsp_libc_init((void *) heap_start, heap_size, 0);

	/* put the commandline parameters into the environment */
	if (buf) {
			char *beg,*end;
			extern int putenv();
			for (beg=buf; beg; beg=end) {
					/* skip whitespace */
					while (' '==*beg) {
							if (!*++beg) {
								/* end of string reached; bail out */
								goto done;
							}
					}
					end=strchr(beg,' ');
					if (end) *(end++)=0;
					/* add string to environment */
					putenv(beg);
			}
		done:
			_Workspace_Free(buf);
	}

#ifdef RTEMS_DEBUG
    rtems_debug_enable( RTEMS_DEBUG_ALL_MASK );
#endif
}

void zero_bss()
{
  extern unsigned long __bss_start, __sbss_start, __sbss_end;
  extern unsigned long __sbss2_start, __sbss2_end;
  memset(&__sbss_start, 0, ((unsigned) (&__sbss_end)) - ((unsigned) &__sbss_start));
  memset(&__sbss2_start, 0, ((unsigned) (&__sbss2_end)) - ((unsigned) &__sbss2_start));
  memset(&__bss_start, 0, ((unsigned) (&__rtems_end)) - ((unsigned) &__bss_start));
}


/* NOTE: we cannot simply malloc the commandline string;
 * save_boot_params() is called during a very early stage when
 * libc/malloc etc. are not yet initialized!
 *
 * Here's what we do:
 *
 * initial layout setup by the loader (preload.S):
 *
 * 0..RTEMS...__rtems_end | cmdline ....... TOP
 *
 * After the save_boot_params() routine returns, the stack area will be
 * set up (start.S):
 *
 * 0..RTEMS..__rtems_end | INIT_STACK | IRQ_STACK | ..... TOP
 *
 * initialize_executive_early() [called from boot_card()]
 * will initialize the workspace:
 *
 * 0..RTEMS..__rtems_end | INIT_STACK | IRQ_STACK | ...... | workspace | TOP
 *
 * and later calls our pretasking_hook() which ends up initializing
 * libc which in turn initializes the heap
 *
 * 0..RTEMS..__rtems_end | INIT_STACK | IRQ_STACK | heap | workspace | TOP
 *
 * The idea here is to first move the commandline to the future 'heap' area
 * from where it will be picked up by our pretasking_hook().
 * pretasking_hook() then moves it either to INIT_STACK or the workspace
 * area using proper allocation, initializes libc and finally moves
 * the data to the environment...
 */

void
save_boot_params(void *r3, void *r4, void* r5, char *cmdline_start, char *cmdline_end)
{
int		i=cmdline_end-cmdline_start;
CmdLine future_heap=(CmdLine)heapStart();

 	/* get the string out of the stack area into the future heap region;
	 * assume there's enough memory...
	 */
	memmove(future_heap->buf,cmdline_start,i);
	/* make sure there's an end of string marker */
	future_heap->buf[i++]=0;
	future_heap->size=i;
}

#define BPNT 0xffd4
#undef  BPNT

/*
 *  bsp_start
 *
 *  This routine does the bulk of the system initialization.
 */

int _BSP_vme_bridge_irq=-1;

void bsp_start( void )
{
#ifdef BPNT
  int bpval = *(int*)BPNT;
#endif
  unsigned char *stack;
  unsigned long *r1sp;
  unsigned l2cr;
  register unsigned char* intrStack;
  register unsigned int intrNestingLevel = 0;
  unsigned char *work_space_start;
  ppc_cpu_id_t myCpu;
  ppc_cpu_revision_t myCpuRevision;
  unsigned char reg;
  const unsigned char *chpt;
  Triv121PgTbl	pt;
  unsigned long	ldPtSize;
  unsigned long tmp;

  /*
   * Get CPU identification dynamically. Note that the get_ppc_cpu_type() function
   * store the result in global variables so that it can be used latter...
   */
  myCpu 	= get_ppc_cpu_type();
  myCpuRevision = get_ppc_cpu_revision();

  /*
   * Access to board registers and PCI devices.
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
   */
  l2cr = get_L2CR();
#ifdef SHOW_LCR2_REGISTER
  printk("Initial L2CR value = %x\n", l2cr);
#endif  
  if ( (! (l2cr & 0x80000000)) && ((int) l2cr == -1))
    set_L2CR(0xb9A14000);

  /*
   * the initial stack  has already been set to this value in start.S
   * so there is no need to set it in r1 again... It is just for info
   * so that It can be printed without accessing R1.
   */
  stack = ((unsigned char*) &__rtems_end) + INIT_STACK_SIZE - CPU_MINIMUM_STACK_FRAME_SIZE;

 /* tag the bottom (T. Straumann 6/36/2001 <strauman@slac.stanford.edu>) */
  *((unsigned32 *)stack) = 0;

  /* fill stack with pattern for debugging */
  __asm__ __volatile__("mr %0, %%r1":"=r"(r1sp));
  while (--r1sp >= (unsigned long*)&__rtems_end)
	  *r1sp=0xeeeeeeee;

  /*
   * Initialize the interrupt related settings
   * SPRG0 = interrupt nesting level count
   * SPRG1 = software managed IRQ stack
   *
   * This could be done latter (e.g in IRQ_INIT) but it helps to understand
   * some settings below...
   */
  intrStack = ((unsigned char*) &__rtems_end) + INIT_STACK_SIZE + INTR_STACK_SIZE - CPU_MINIMUM_STACK_FRAME_SIZE;

 /* tag the bottom (T. Straumann 6/36/2001 <strauman@slac.stanford.edu>) */
  *((unsigned32 *)intrStack) = 0;
  /* fill interrupt stack with pattern for debugging */
  r1sp=(unsigned long*)intrStack;
  while (--r1sp >= (unsigned long*)( intrStack - (INTR_STACK_SIZE - 8)))
	  *r1sp=0xeeeeeeee;

  asm volatile ("mtspr	273, %0" : "=r" (intrStack) : "0" (intrStack));
  asm volatile ("mtspr	272, %0" : "=r" (intrNestingLevel) : "0" (intrNestingLevel));
  /*
   * Initialize default raw exception hanlders. See vectors/vectors_init.c
   */
  initialize_exceptions();
#ifdef BPNT
  BSPsetDABR(BPNT, 0x6);
#endif

#if 0
  /*
   * Init MMU block address translation to enable hardware
   * access
   */
  /*
   * PC legacy IO space used for inb/outb and all PC
   * compatible hardware
   */
  setdbat(3, 0x80000000, 0x80000000, 0x10000000, IO_PAGE);
  /*
   * PCI devices memory area. Needed to access OPENPIC features
   * provided by the RAVEN
   */
#endif

  select_console(CONSOLE_SERIAL);

  if (!(chpt=BSP_boardType())) {
		  BSP_panic("Unknown Synergy Board Type");
  }

  printk("-----------------------------------------\n");
  printk("Welcome to %s on %s\n", _RTEMS_version, chpt);
  printk("-----------------------------------------\n");
#ifdef SHOW_MORE_INIT_SETTINGS  
  printk("Initial system stack at %x\n",stack);
  __asm__ __volatile__ ("mr %0, %%r1":"=r"(stack));
  printk("(R1 stack pointer is 0x%08x)\n", stack);
  printk("Software IRQ stack at %x\n",intrStack);
  printk("-----------------------------------------\n");
#endif

#ifdef SHOW_MORE_INIT_SETTINGS
  printk("Going to start PCI buses scanning and initialization\n");
#endif  
  /* initialize pci driver. We just supply the SVGM's 
   * config_addr / config_data addresses here
   */
  InitializePCI();

#ifdef TEST_RAW_EXCEPTION_CODE  
  printk("Testing exception handling Part 1\n");
  /*
   * Cause a software exception
   */
  __asm__ __volatile ("sc");
  /*
   * Check we can still catch exceptions and returned coorectly.
   */
  printk("Testing exception handling Part 2\n");
  __asm__ __volatile ("sc");
#endif  

#undef DEBUG_PROTECT_TEXT
/* The idea here is to put text and readonly data
 * into a write-protected area to catch bugs.
 * However, since we have no pagetables (only BATs),
 * we must use two rather large chunks of e.g.
 * 16M+16M.
 * BAT 0 already maps everything RW which is fine
 * until the vectors are installed. Afterwards,
 * we will restrict access to the first chunk
 * by setting up another BAT (3 is still unused)
 * 
 * NOTE: enabling this requires changing the
 *       linker script:
 *       - bump up the code size to 32M
 *       - origin the data section to 16M
 *       Be warned that this results in a huge
 *       image which takes a while to download...
 */
#ifdef DEBUG_PROTECT_TEXT
  BSP_mem_size 				= 0x2000000;		/* 16M+16M */
#else
  /* read memory info register */
  reg = *SYN_VGM_REG_INFO_MEMORY;
  BSP_mem_size 				= 
		SYN_VGM_REG_INFO_MEMORY_BANKS(reg) * 	/* number of banks */
		SYN_VGM_REG_INFO_MEMORY_BANK_SIZE(reg); /* bank size */
#endif
  /* Allocate a page table large enough to map
   * the entire physical memory. We put the page
   * table at the top of the physical memory.
   */
  /* get minimal size (log base 2) of PT for
   * this board's memory
   */
#define PGTBL
#undef PGTBL_VME
#ifdef PGTBL

#ifdef PGTBL_VME
  ldPtSize = triv121PgTblLdMinSize(0x02000000)+1;
#else
  ldPtSize = triv121PgTblLdMinSize(BSP_mem_size);
#endif
  ldPtSize++; /* double this amount */
  if ( (pt = triv121PgTblInit(BSP_mem_size - (1<<ldPtSize), ldPtSize)) ) {
#ifdef PGTBL_VME
	tmp = triv121PgTblMap(
						pt,
						TRIV121_121_VSID,
						0x90000000,
						0x02000000 >> PG_SHIFT,
						TRIV121_ATTR_IO_PAGE, /* WIMG */
						TRIV121_PP_RW_PAGE);
#else
	extern unsigned long __DATA_START__, _etext;
	/* map text and RO data read-only */
	tmp = triv121PgTblMap(
						pt,
						TRIV121_121_VSID,
						0,
						(PAGE_ALIGN((unsigned long)&_etext) - 0) >> PG_SHIFT,
						0, /* WIMG */
						TRIV121_PP_RO_PAGE);
	if (TRIV121_MAP_SUCCESS != tmp) {
		printk("Unable to map page index %i; reverting to BAT0\n", 
				tmp);
		pt = 0;
	} else {
		/* map the rest (without the page table itself) RW */
		tmp = triv121PgTblMap(
						pt,
						TRIV121_121_VSID,
						(unsigned long)&__DATA_START__,
						(BSP_mem_size - (1<<ldPtSize) -  (unsigned long)&__DATA_START__ )>> PG_SHIFT,
						0, /* WIMG */
						TRIV121_PP_RW_PAGE);
		if (TRIV121_MAP_SUCCESS != tmp) {
			printk("Unable to map page index %i; reverting to BAT0\n", 
					tmp);
			pt = 0;
		}
	}
#endif
  } else {
	printk("WARNING: unable to allocate page table, keeping DBAT0\n");
  }
  if (pt) {
  	unsigned long tsill;
#warning TSILL
	/* reduce available memory by size of the page table */
	BSP_mem_size -= (1<<ldPtSize);
		if (0x0c0c!=(tsill=triv121IsRangeMapped(0,BSP_mem_size))) {
			printk("Mapping failure 0 at 0x%08x\n",tsill);
  		}
}
  /* switch the text/ro sements to RO only after
   * initializing the interrupts because the irq_mng
   * installs some code...
   */
  if (pt) {
		/* activate the page table; it is still masked by the
		 * DBAT0, however
		 */
		triv121PgTblActivate(pt);
#ifdef  SHOW_MORE_INIT_SETTINGS
		printk("Page table setup finished; going to deactivate DBAT0...\n");
#endif
	}

{
unsigned long tsill;
		if (0x0c0c!=(tsill=triv121IsRangeMapped(0,BSP_mem_size))) {
			printk("Mapping failure A at 0x%08x\n",tsill);
  		}
}
#endif
  reg = *SYN_VGM_REG_STAT_BOARD;
  switch (reg & SYN_VGM_REG_STAT_CPU_BUS_SPEED_MSK) {
		  default:
		  case SYN_VGM_REG_STAT_CPU_BUS_SPEED_66:
				BSP_bus_frequency = 66666667; break;
		  case SYN_VGM_REG_STAT_CPU_BUS_SPEED_83:
				BSP_bus_frequency = 83333333; break;
		  case SYN_VGM_REG_STAT_CPU_BUS_SPEED_100:
				BSP_bus_frequency = 100000000; break;
  } 
  BSP_processor_frequency	= 366000000; /* TODO */
  BSP_time_base_divisor		= 4000; /* 750 and 7400 clock the TB / DECR at 1/4 of the CPU speed */
  BSPBaseBaud				= 9600*156; /* TODO, found by experiment */
  
  /*
   * Set up our hooks
   * Make sure libc_init is done before drivers initialized so that
   * they can use atexit()
   */

  Cpu_table.pretasking_hook 	 = bsp_pretasking_hook;    /* init libc, etc. */
  Cpu_table.postdriver_hook 	 = bsp_postdriver_hook;
  Cpu_table.do_zero_of_workspace = TRUE;
  Cpu_table.interrupt_stack_size = CONFIGURE_INTERRUPT_STACK_MEMORY;
  /* TB is clocked by PPC bus clock / timebase divisor */
  Cpu_table.clicks_per_usec 	 = BSP_bus_frequency/(BSP_time_base_divisor * 1000);
  Cpu_table.exceptions_in_RAM 	 = TRUE;

#ifdef SHOW_MORE_INIT_SETTINGS
/*  printk("BSP_Configuration.work_space_size = %x\n", BSP_Configuration.work_space_size); */
#endif  
  work_space_start = 
    (unsigned char *)BSP_mem_size - BSP_Configuration.work_space_size;

  if ( work_space_start <= ((unsigned char *)&__rtems_end) + INIT_STACK_SIZE + INTR_STACK_SIZE) {
    printk( "bspstart: Not enough RAM!!!\n" );
    bsp_cleanup();
  }

  BSP_Configuration.work_space_start = work_space_start;
#ifdef PGTBL
{
unsigned long tsill;
		if (0x0c0c!=(tsill=triv121IsRangeMapped(0,BSP_mem_size))) {
			printk("Mapping failure B at 0x%08x\n",tsill);
  		}
}
#endif
  /*
   * Initalize RTEMS IRQ system (including the openPIC)
   */ 

  BSP_rtems_irq_mng_init(0);

#ifdef PGTBL
  if (pt) { extern void MMUoff(), MMUon();
#warning TSILL
		unsigned long tsill;
		/* switch of the DBAT0 mapping */
		if (0x0c0c!=(tsill=triv121IsRangeMapped(0,BSP_mem_size))) {
			printk("Mapping failure C at 0x%08x\n",tsill);
  		}
		MMUoff();
		__asm__ __volatile__(
			"sync; isync\n"
			"mfspr %%r0, %0\n"
			"andc  %%r0, %%r0, %1\n"
			"mtspr %0, %%r0\n"
			"sync\n"
			::"i"(DBAT0U),"r"(BAT_VS | BAT_VP)
			:"r0"
		);
		MMUon();
		if (0x0c0c!=(tsill=triv121IsRangeMapped(0,BSP_mem_size))) {
			printk("Mapping failure D at 0x%08x\n",tsill);
  		}
		__asm__ __volatile__("mfsdr1 %0":"=r"(tsill));
		printk("SDR1 is 0x%08x\n",tsill);
  }
#endif

  /*
   * Initialize VME bridge - needs working PCI
   * and IRQ subsystems...
   */
#ifdef SHOW_MORE_INIT_SETTINGS
  printk("Going to initialize VME bridge (disabling all windows)\n");
#endif  
  /* VME initialization is in a separate file so apps which don't use
   * VME or want a different configuration may link against a customized
   * routine.
   */
  BSP_vme_config();

#ifdef DEBUG_PROTECT_TEXT
  /* restrict the dbats to writing to the second 16M chunk of memory only */
  printk("protecting 1st 16M from write access\n");
  /* setdbat() doesn't allow to change bat0, we have to do it directly :-(
   * setdbat(0, 0x01000000, 0x01000000, 0x01000000, _PAGE_RW);
   */
  {
	unsigned long dbat0l,dbat0h;
	dbat0l = 0x01000000 | 2;
/*           ^ phys[0:14] ^ RW access [30:31], RO is 1 */
	dbat0h = 0x01000000 | (((0x01000000 >> 17) -1) << 2) | 2;
/*           ^ virt[0:14]    ^ size --> mask               ^ SUP USER [30:31] */
  __asm__ __volatile__(
			"mr	%%r3, %0\n"	/* make sure values are in registers before we tamper with the BATs */
			"mr	%%r4, %1\n"
			"sync; isync\n"
			"li %%r0, 0\n"
			"mtspr %3, %%r0\n" /* switch off */
			"mtspr %2, %%r0\n"
			"sync; isync\n"
			"mtspr %2, %%r3\n"
			"mtspr %3, %%r4\n"
			"sync; isync\n"
			::"r"(dbat0l),"r"(dbat0h),"i"(DBAT0L),"i"(DBAT0U)
			:"r0","r3","r4");
  }
  /* now make the 1st 16M read-only */
  setdbat(3, 0x00000000, 0x00000000, 0x01000000, 0);
#endif
#ifdef SHOW_MORE_INIT_SETTINGS
#ifdef BPNT
  printk("*BPNT is 0x%08x\n",bpval);
#endif
  /* printk("Exit from bspstart 0x%08x\n",*(unsigned long*)(0xffe4)); */
#endif  
}
