/* $Id$ */

/* A slightly improved exception 'default' exception handler for RTEMS / SVGM */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2002/5 */

#include <bsp.h>
#include <bsp/vectors.h>
#include <libcpu/raw_exception.h>
#include <bsp/pci.h>
#include <rtems/bspIo.h>

#include <bsp/bspException.h>

#define SRR1_TEA_EXC	(1<<(31-13))
#define SRR1_MCP_EXC	(1<<(31-12))

void
BSP_printStackTrace();

unsigned long
_BSP_clear_hostbridge_errors();

void
BSP_exceptionHandler(BSP_Exception_frame* excPtr)
{
rtems_unsigned32		note;
BSP_ExceptionExtension	ext=0;
rtems_id				id=0;
int						recoverable = 0;
char					*fmt="Uhuuuh, Exception %d in unknown task???\n";
	
	/* If we are in interrupt context, we are in trouble - skip the user
	 * hook and panic
	 */
	if (_ISR_Is_in_progress()) {
		fmt="Aieeh, Exception %d in interrupt handler\n";
	} else if ( !_Thread_Executing) {
		fmt="Aieeh, Exception %d in initialization code\n";
	} else {
		/* retrieve the notepad which possibly holds an extention pointer */
		if (RTEMS_SUCCESSFUL==rtems_task_ident(RTEMS_SELF,RTEMS_LOCAL,&id) &&
		    RTEMS_SUCCESSFUL==rtems_task_get_note(id, BSP_EXCEPTION_NOTEPAD, &note)) {
			ext = (BSP_ExceptionExtension)note;
			printk("Task (Id 0x%08x) got ",id);
			fmt="exception %d\n";
		}
	}
	
	if (ext && ext->hook && ext->hook(excPtr,ext,0)) {
		/* they did all the work and want us to do nothing! */
		return;
	}

	/* message about exception */
	printk(fmt, excPtr->_EXC_number);
	/* register dump */
	printk("\t Next PC or Address of fault = %x, ", excPtr->EXC_SRR0);
	printk("Saved MSR = %x\n", excPtr->EXC_SRR1);
	printk("\t R0  = %08x", excPtr->GPR0);
	printk(" R1  = %08x", excPtr->GPR1);
	printk(" R2  = %08x", excPtr->GPR2);
	printk(" R3  = %08x\n", excPtr->GPR3);
	printk("\t R4  = %08x", excPtr->GPR4);
	printk(" R5  = %08x", excPtr->GPR5);
	printk(" R6  = %08x", excPtr->GPR6);
	printk(" R7  = %08x\n", excPtr->GPR7);
	printk("\t R8  = %08x", excPtr->GPR8);
	printk(" R9  = %08x", excPtr->GPR9);
	printk(" R10 = %08x", excPtr->GPR10);
	printk(" R11 = %08x\n", excPtr->GPR11);
	printk("\t R12 = %08x", excPtr->GPR12);
	printk(" R13 = %08x", excPtr->GPR13);
	printk(" R14 = %08x", excPtr->GPR14);
	printk(" R15 = %08x\n", excPtr->GPR15);
	printk("\t R16 = %08x", excPtr->GPR16);
	printk(" R17 = %08x", excPtr->GPR17);
	printk(" R18 = %08x", excPtr->GPR18);
	printk(" R19 = %08x\n", excPtr->GPR19);
	printk("\t R20 = %08x", excPtr->GPR20);
	printk(" R21 = %08x", excPtr->GPR21);
	printk(" R22 = %08x", excPtr->GPR22);
	printk(" R23 = %08x\n", excPtr->GPR23);
	printk("\t R24 = %08x", excPtr->GPR24);
	printk(" R25 = %08x", excPtr->GPR25);
	printk(" R26 = %08x", excPtr->GPR26);
	printk(" R27 = %08x\n", excPtr->GPR27);
	printk("\t R28 = %08x", excPtr->GPR28);
	printk(" R29 = %08x", excPtr->GPR29);
	printk(" R30 = %08x", excPtr->GPR30);
	printk(" R31 = %08x\n", excPtr->GPR31);
	printk("\t CR  = %08x\n", excPtr->EXC_CR);
	printk("\t CTR = %08x\n", excPtr->EXC_CTR);
	printk("\t XER = %08x\n", excPtr->EXC_XER);
	printk("\t LR  = %08x\n", excPtr->EXC_LR);
	printk("\t DAR = %08x\n", excPtr->EXC_DAR);
	
	BSP_printStackTrace(excPtr);
	
	if (ASM_MACH_VECTOR == excPtr->_EXC_number) {
	/* ollah , we got a machine check - this could either
	 * be a TEA, MCP or internal; let's see and provide more info
	 */
		printk("Machine check; reason:");
		if ( ! (excPtr->EXC_SRR1 & (SRR1_TEA_EXC | SRR1_MCP_EXC)) ) {
			printk("SRR1\n");
		} else { 
			if (excPtr->EXC_SRR1 & (SRR1_TEA_EXC)) {
				printk(" TEA");
			}
			if (excPtr->EXC_SRR1 & (SRR1_MCP_EXC)) {
				unsigned char c;
				unsigned int  l;
				unsigned long gerr;

				printk(" MCP\n");

				/* it's MCP; gather info from the host bridge */

				gerr=_BSP_clear_hostbridge_errors();
				if (0x80 != gerr) {
					printk("Grackle Error Registers: PCI CSR: %04x, ERRDR1: %02x, ERRDR2: %02x\n",
							gerr & 0xffff, (gerr>>16) & 0xff, (gerr>>24)&0xff);
					pci_read_config_byte(0,0,0,0xc3,&c);
					printk("                           60x Bus Error Status: %02x\n",c);
					pci_read_config_byte(0,0,0,0xc7,&c);
					printk("                           PCI Bus Error Status: %02x\n",c);
					pci_read_config_dword(0,0,0,0xc8,&l);
					printk("                           Bus Error Address   : %08x\n",l);
				} else {
					printk("Grackle seems OK\n");
				}
			} else {
					printk("\n");
			}
		}
	} else if (ASM_DEC_VECTOR == excPtr->_EXC_number) {
		recoverable = 1;
	} else if (ASM_SYS_VECTOR == excPtr->_EXC_number) {
#ifdef TEST_RAW_EXCEPTION_CODE 
		recoverable = 1;
#else
		recoverable = 0;
#endif
	}

	/* call them for a second time giving a chance to intercept
	 * the task_suspend
	 */
	if (ext && ext->hook && ext->hook(excPtr, ext, 1))
		return;

	if (!recoverable) {
		if (id) {
			printk("unrecoverable exception!!! task %08x suspended\n",id);
			if (excPtr->EXC_SRR1 & MSR_FP) {
				register unsigned msr;
				/* thread dispatching is _not_ disabled at this point; hence
				 * we must make sure we have the FPU enabled...
				 */
				__asm__ __volatile__("mfmsr %0":"=r"(msr));
				msr |= MSR_FP;
				__asm__ __volatile__("mtmsr %0; isync"::"r"(msr));
			}
			rtems_task_suspend(id);
		} else {
			printk("PANIC...\n");
			while (1)
				;
		}
	}
}
