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
#include <bsp/VME.h>
#include <bsp/bspVGM.h>
#include <bsp/bspException.h>
#include <synergyregs.h>

/* for RTEMS_VERSION :-( I dont like the preassembled string */
#include <rtems/sptables.h>

#ifdef __RTEMS_APPLICATION__
#undef __RTEMS_APPLICATION__
#endif

/* there is no public Workspace_Free() variant :-( */
#include <rtems/score/wkspace.h>

/* a couple of declarations we have no header for :-( */
void
_BSP_pciCacheInit();

rtems_unsigned32
_bsp_sbrk_init(rtems_unsigned32 heap_start, rtems_unsigned32 *heap_size_p);


/* provide access to the command line parameters */
char *BSP_commandline_string = 0;

BSP_output_char_function_type BSP_output_char = BSP_output_char_via_serial;

#define USE_BOOTP_STUFF

#ifdef  USE_BOOTP_STUFF

#include <rtems/rtems_bsdnet.h>

#include <assert.h>
#include <sys/socket.h>
#include <netinet/in.h>

/* We'll store here what the application put into its
 * network configuration table.
 */
static void (*the_apps_bootp)(void)=0;
static void my_bootp_intercept(void);
static void fillin_srvrandfile(void);

/* NOTE the '__BSP_wrap_xxx' symbols are defined by
 * the linker script. The idea is to avoid referencing
 * any networking symbols, so a non-networked application
 * will not have to be linked against the networking code.
 *
 * These names are long and ugly to prevent name clashes
 * (global namespace :-()
 *
 * Use the ugly [] construct when declaring the external
 * (linker script defined) variables to prevent the compiler
 * from accessing them in the short data areas (-msdata=eabi)
 * which could result in a linkage error.
 */
extern char   						*__BSP_wrap_rtems_bsdnet_bootp_boot_file_name[];
extern char   						*__BSP_wrap_rtems_bsdnet_bootp_cmdline[];
extern struct in_addr				__BSP_wrap_rtems_bsdnet_bootp_server_address[];
extern struct rtems_bsdnet_config	__BSP_wrap_rtems_bsdnet_config[];
void								__BSP_wrap_rtems_bsdnet_do_bootp();
int									__BSP_wrap_inet_pton();
int									__BSP_wrap_rtems_bsdnet_loopattach();

#define bootp_file					(__BSP_wrap_rtems_bsdnet_bootp_boot_file_name[0])
#define bootp_cmdline				(__BSP_wrap_rtems_bsdnet_bootp_cmdline[0])
#define bootp_srvr					(__BSP_wrap_rtems_bsdnet_bootp_server_address[0])
#define net_config					(__BSP_wrap_rtems_bsdnet_config[0])
#define do_bootp					__BSP_wrap_rtems_bsdnet_do_bootp
#define INET_PTON					__BSP_wrap_inet_pton
#define loopattach					__BSP_wrap_rtems_bsdnet_loopattach


/* parameter table for network setup - separate file because
 * copied from the bootloader
 */
#include "bootpstuff.c"
#endif

#undef   SHOW_MORE_INIT_SETTINGS

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

/* prevent this from ending up in the short data area */
extern unsigned long __rtems_end[];
extern void		L1_caches_enables();
extern unsigned get_L2CR();
extern unsigned set_L2CR(unsigned);
extern void		bsp_cleanup(void);

typedef struct CmdLineRec_ {
		unsigned long	size;
		char			buf[0];
} CmdLineRec, *CmdLine;

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

/*
 * system init stack and soft ir stack size
 */
#define INIT_STACK_SIZE 0x1000
#define INTR_STACK_SIZE CONFIGURE_INTERRUPT_STACK_MEMORY

/* calculate the heap start */
static unsigned long
heapStart(void)
{
unsigned long rval;
    rval = ((rtems_unsigned32) __rtems_end) +INIT_STACK_SIZE + INTR_STACK_SIZE;
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
void			bsp_libc_init( void *, unsigned32, int );

void			BSP_vme_config(void);
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
    rtems_unsigned32        heap_size,heap_sbrk_spared;
	char					*buf;
#ifdef USE_BOOTP_STUFF
	Parm					p;
#endif

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

	heap_sbrk_spared=_bsp_sbrk_init(heap_start, &heap_size);

#ifdef SHOW_MORE_INIT_SETTINGS
   	printk(" HEAP start %x  size %x (%x bytes spared for sbrk)\n", heap_start, heap_size, heap_sbrk_spared);
#endif    

    bsp_libc_init((void *) 0, heap_size, heap_sbrk_spared);

	/* put the commandline parameters into the environment */
	if (buf) {
#ifdef USE_BOOTP_STUFF
		char		*beg,*end;
		for (beg=buf; beg; beg=end) {
			/* skip whitespace */
			while (' '==*beg) {
				if (!*++beg) {
					/* end of string reached; bail out */
					goto done;
				}
			}
			/* simple algorithm to find the end of quoted 'name=quoted'
			 * substrings. As a side effect, quotes are removed from
			 * the value.
			 */
			if ( (end = strchr(beg,'=')) ) {
				if ('\'' == *++end) {
					/* end points to the 1st char after '=' which is a '\'' */

					char *dst = end++;

					/* end points to 1st char after '\'' */

					while ('\'' != *end || '\'' == *++end) {
						if ( 0 == (*dst++=*end++) ) {
							/* NO TERMINATING QUOTE FOUND
							 * (for a properly quoted string we
							 * should never get here)
							 */
							end = 0;
							dst--;
							break;
						}
					}
					*dst = 0;
				} else {
					/* first space terminates non-quoted strings */
					if ( (end = strchr(end,' ')) )
						*(end++)=0;
				}
			}
			/* save special bootloader strings to our private environment
			 * and pass on the others
			 */
			for (p=parmList; p->name; p++) {
				if (!p->pval) continue;
				if (0 == strncmp(beg,p->name,strlen(p->name))) {
					/* found this one; since 'name' contains a '=' strchr will succeed */
					char *s=strchr(beg,'=')+1;

					/* p->pval might point into the
					 * network configuration which is invalid
					 * if we have no networking
					 */
					if (&net_config) {
						*p->pval=malloc(strlen(s)+1);
						strcpy(*p->pval,s);
					}
					break;
				}
			}
			/* unfound name=value pairs are dropped to the floor */
		}
#else
		BSP_commandline_string = strdup(buf);
#endif

		done:
			_Workspace_Free(buf);
	}

#ifdef USE_BOOTP_STUFF
	/* Check if we are linked with networking support */
	if (&net_config) {
		/* seems so - make sure the other symbols are
		 * defined also...
		 */
		assert(&bootp_file && &bootp_srvr && do_bootp && INET_PTON 
				&& "APP NOT LINKED WITH ALL NECESSARY NETWORKING SYMBOLS");
		/* now hack into the network configuration... */

		if (boot_use_bootp && 'N'==toupper(*boot_use_bootp)) {
			/* no bootp */
			net_config.bootp=0;
			/* get pointers to the first interface's configuration */
			if (&net_config) {
				struct rtems_bsdnet_ifconfig *ifc;
	
				for (ifc=net_config.ifconfig;
					ifc && loopattach==ifc->attach;
					ifc=ifc->next) {
					/* should probably make sure it's not a point-to-point
					 * IF either
					 */
				}
				assert(ifc && "NO INTERFACE CONFIGURATION STRUCTURE FOUND");

				ifc->ip_address = boot_my_ip;
				boot_my_ip=0;
				ifc->ip_netmask = boot_my_netmask;
				boot_my_netmask = 0;
				/* override the server/filename parameters */
				fillin_srvrandfile();
			}
			
		} else {
			the_apps_bootp=net_config.bootp;
			net_config.bootp=my_bootp_intercept;
			/* release the strings that will be set up by
			 * bootp - bootpc relies on them being NULL
			 */
			for (p=parmList; p->name; p++) {
				if (!p->pval) continue;
				if (p->flags & FLAG_CLRBP) {
					free(*p->pval); *p->pval=0;
				}
			}
		}
	}
#endif


#ifdef RTEMS_DEBUG
    rtems_debug_enable( RTEMS_DEBUG_ALL_MASK );
#endif
}

#ifdef USE_BOOTP_STUFF
static void
fillin_srvrandfile(void)
{
	/* OK - now let's see what we have */
	if (boot_srvname) {
		/* Seems we have a different file server */
		if (INET_PTON(AF_INET,
					boot_srvname,
					&bootp_srvr)) {
		}
	}
	if (boot_filename) {
		/* Ha - they manually changed the file name and the parameters */
		bootp_file    = boot_filename;
		/* (dont bother freeing the old one - we don't really know if its malloced */
		boot_filename = 0;
	}
	if (boot_cmdline) {
		/* comments for boot_filename apply here as well */
		bootp_cmdline = boot_cmdline;
	} else {
		boot_cmdline  = bootp_cmdline;
	}
}
/* if the bootloader loaded a different file
 * than what the BOOTP/DHCP server says we have
 * then we want to forge the respective system
 * variables.
 */
static void
my_bootp_intercept(void)
{
	/* Do bootp first */
	if (the_apps_bootp) {
		the_apps_bootp();
	} else {
		do_bootp();
	}
	/* override the server/filename parameters */
	fillin_srvrandfile();
}
#endif

void zero_bss()
{
  /* prevent these from being accessed in the short data areas */
  extern unsigned long __bss_start[], __sbss_start[], __sbss_end[];
  extern unsigned long __sbss2_start[], __sbss2_end[];
  memset(__sbss_start, 0, ((unsigned) __sbss_end) - ((unsigned)__sbss_start));
  memset(__sbss2_start, 0, ((unsigned) __sbss2_end) - ((unsigned)__sbss2_start));
  memset(__bss_start, 0, ((unsigned) __rtems_end) - ((unsigned)__bss_start));
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
 * the data to the environment / malloced areas...
 */

/* this routine is called early and must be safe with a not properly
 * aligned stack
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
  register unsigned int		intrNestingLevel = 0;
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
   * SPRG0 = interrupt nesting level count
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

  asm volatile ("mtspr	%2, %0"
				: "=r" (intrStack)
				: "0" (intrStack), "i"(SPRG1)
			   );
  asm volatile ("mtspr	%2, %0"
				: "=r" (intrNestingLevel)
				: "0" (intrNestingLevel), "i"(SPRG0)
			   );
  /*
   * Initialize default raw exception handlers. See vectors/vectors_init.c
   */
  initialize_exceptions();

  if (!(chpt=BSP_boardType())) {
		  BSP_panic("Unknown Synergy Board Type");
  }

  printk("-----------------------------------------\n");
  printk("Welcome to RTEMS RELEASE %s/svgm on %s/%s/%s\n",
		  RTEMS_VERSION, chpt, CPU_NAME, get_ppc_cpu_type_name(myCpu));
  printk("SSRL Release $Name$/$Date$\n");
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
  /* initialize pci driver. We just supply the SVGM's 
   * config_addr / config_data addresses here
   */
  InitializePCI();

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
   * (call this routine only after the CPU table has been initialized;
   * it uses rtems_bsp_delay())
   *
   * Experience tells us that we have to repeat this step a couple
   * of times...
   */
  _BSP_clear_hostbridge_errors(0/*enableMCP*/,1/*quiet*/);
  _BSP_clear_hostbridge_errors(0/*enableMCP*/,1/*quiet*/);
  _BSP_clear_hostbridge_errors(1/*enableMCP*/,0/*quiet*/);

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
  Cpu_table.exceptions_in_RAM 	 = TRUE;

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

  /*
   * Initialize VME bridge - needs working PCI
   * and IRQ subsystems...
   */
#ifdef SHOW_MORE_INIT_SETTINGS
  printk("Going to initialize VME bridge\n");
#endif  
  /* VME initialization is in a separate file so apps which don't use
   * VME or want a different configuration may link against a customized
   * routine.
   */
  BSP_vme_config();

}
