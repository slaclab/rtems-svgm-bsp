#include <sys/types.h>
#include <libcpu/spr.h>
#include "../../shared/bootloader/bootldr.h"
#include <bsp/residual.h>
#include <libcpu/mmu.h>
#include <libcpu/page.h>
#include <limits.h>


/* size of the decompressed image */
/*extern unsigned long _rtems_size; */

extern void fix_residual(RESIDUAL*);
extern void setLED(unsigned char);

#define SVGM_CACHEL_SZ 32
extern char __rtems_start,__rtems_end;

void
setLED(unsigned char val)
{
int i;
volatile unsigned char *ledreg=(volatile unsigned char*)0xffeffe80;
	for (i=0; i<8; i++,ledreg+=8,val>>=1)
		*ledreg = val&1;	
	__asm__ __volatile__("eieio");
}

#define BNUM(i) (1<<(31-(i)))
#define HID0_ICE        BNUM(16)
#define HID0_DCE        BNUM(17)
#define HID0_ILOCK      BNUM(18)
#define HID0_DLOCK      BNUM(19)
#define HID0_ICFI       BNUM(20)
#define HID0_DCFI       BNUM(21)

#define HIHI            (HID0_ICE|HID0_DCE|HID0_ILOCK|HID0_DLOCK|HID0_ICFI|HID0_DCFI)
#define HIHO            (HID0_ILOCK|HID0_DLOCK|HID0_ICFI|HID0_DCFI)



void cacheFlushAndDisable(void)
	{
#if 0
	__asm__ __volatile__(
        "mfspr %%r3, 1014       \n"     /* MSSCR0 */
        "mfspr %%r4, 1008       \n"     /* HID0 */
        "oris  %%r3, %%r3, %0   \n"
        "ori   %%r4, %%r4, %1   \n"
        "xori  %%r4, %%r4, %1   \n"
        "sync                   \n"
        "isync                  \n"
        "mtspr 1014, %%r3       \n"
        "sync                   \n"
        "isync                  \n"
        "mtspr  1008,%%r4       \n"
        "sync                   \n"
        "isync                  \n"
                ::"i"(BNUM(8)>>16),"i"(HIHI):"r3","r4");

	memset(0x3ff0000,0x0,0x10000);
#endif
#if 0
#if 0
		volatile unsigned char *p;
		unsigned long tmp;
		for (p=(volatile unsigned char*)0x0;
			p<(volatile unsigned char*) (3*1024*1024);
			p+=32) {
			tmp+=*p;

		}
		__asm__ __volatile__("sync");
		for (p=(volatile unsigned char*)0x0;
			p<(volatile unsigned char*) (3*1024*1024);
			p+=32) {
			__asm__ __volatile__ ("dcbf 0, %0"::"r"(p));
		}
		__asm__ __volatile__("sync");
		__asm__ __volatile__(
				"mfspr %%r3, 1008\n"
				"ori   %%r3, %%r3, 0x4400\n"
				"sync\n"
				"mtspr 1008, %%r3\n"
				"sync\n"
				"sync\n":::"r3");
#endif
#if 1
		__asm__ __volatile__(
				"mfspr %%r3, 1008\n"
				"ori   %%r3, %%r3, 0x4400\n"
				"sync\n"
				"mtspr 1008, %%r3\n"
				"xori  %%r3, %%r3,0x4400\n"
				"mtspr 1008, %%r3\n"
				"sync\n":::"r3");
#endif
#endif

	}
int svgm_early_setup(unsigned long img_size)
{
#if 0
unsigned long s = &__rtems_end - &__rtems_start;

	s = 0x100000;

	/* round up to a cacheline size */
	s = (s+SVGM_CACHEL_SZ-1) & ~(SVGM_CACHEL_SZ-1);
	bd->mover=s-0x10000;
	/* provide space for the stack */
	bd->image=(void*)s; /* where to move the image */
	/* the stack is set _after_ using the mover */ 
	bd->stack=(void*)(s-SVGM_CACHEL_SZ);
	bd->cache_lsize=SVGM_CACHEL_SZ;
	bd->residual=(RESIDUAL*)&dummyRes;
	memset(&dummyRes,0,sizeof(dummyRes));
#endif
	/* try to flush and disable the cache */
	bd->residual=(RESIDUAL*)0x200000;
	memset(bd->residual,0,sizeof(*bd->residual));
	fix_residual(bd->residual);
	early_setup(img_size);
	return 0;
}

void 
tillfill(void)
{
	memset(0x3ff0000,0xee,0x10000);
	{
		unsigned char *p;
		__asm__ __volatile__("sync");
		for (p=0x3ff0000;p<0x400000;p+=32)
			__asm__ __volatile__("dcbf 0, %0"::"r"(p));
		__asm__ __volatile__("sync");
	}
}

