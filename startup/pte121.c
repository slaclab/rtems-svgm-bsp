#undef  DEBUG_MAIN
#define DEBUG

#ifndef DEBUG_MAIN
#include <rtems.h>
#include <libcpu/cpu.h>
#include <bsp.h>
#include <bsp/vectors.h>
#include <libcpu/raw_exception.h>
#endif

#include <assert.h>

#include "pte121.h"

/* $Id$ */

/* Trivial page table setup for RTEMS
 * Purpose: allow write protection of text/ro-data
 *
 * Author: Till Straumann <strauman@slac.stanford.edu>, 4/2002
 */


/* base 2 logs of some sizes */

#ifndef DEBUG_MAIN
#define	LD_PHYS_SIZE	32		/* physical address space */
#define LD_PG_SIZE		12		/* page size */
#define LD_PTEG_SIZE	6		/* PTEG size */
#define LD_PTE_SIZE		3		/* PTE size  */
#define LD_SEG_SIZE		28		/* segment size */
#define LD_MIN_PT_SIZE	16		/* minimal size of a page table */
#define LD_HASH_SIZE	19		/* lengh of a hash */
#else
/* reduced 'fantasy' sizes for testing */
#define	LD_PHYS_SIZE	32		/* physical address space */
#define LD_PG_SIZE		6		/* page size */
#define LD_PTEG_SIZE	5		/* PTEG size */
#define LD_PTE_SIZE		3		/* PTE size  */
#define LD_SEG_SIZE		28		/* segment size */
#define LD_MIN_PT_SIZE	7		/* minimal size of a page table */
#define LD_HASH_SIZE	19		/* lengh of a hash */
#endif

#define LD_PI_SIZE		((LD_SEG_SIZE) - (LD_PG_SIZE)) 
#define PTE_PER_PTEG	(1<<((LD_PTEG_SIZE)-(LD_PTE_SIZE)))

#define KEY_SUP			(1<<30)
#define KEY_USR			(1<<29)

#define VSID121(ea) (((ea)>>LD_SEG_SIZE) & ((1<<(LD_PHYS_SIZE-LD_SEG_SIZE))-1))
#define PI121(ea)	(((ea)>>LD_PG_SIZE) & ((1<<LD_PI_SIZE)-1))

/* The range of effective addresses to scan with 'tlbie'
 * instructions in order to flush all TLBs.
 * On the 750 and 7400, there are 128 two way I and D TLBs,
 * indexed by EA[14:19]. Hence calling
 *	  tlbie rx
 * where rx scans 0x00000, 0x01000, 0x02000, ... 0x3f000
 * is sufficient to do the job
 */
#ifdef TSILL
#define FLUSH_EA_RANGE	(64<<LD_PG_SIZE)
#else
#warning "FLUSH_EA_RANGE"
#define FLUSH_EA_RANGE	(0x10000000)
#endif

/* A PTE entry */
typedef struct PTERec_ {
		unsigned long v:1,    vsid:24, h:1, api: 6;
		unsigned long rpn:20, pad: 3, r:1, c:1, wimg:4, marked:1, pp:2;
} PTERec, *PTE;

typedef struct Triv121PgTblRec_ {
		PTE				base;
		unsigned long	size;
		int				active;
} Triv121PgTblRec;

/* dont malloc - we might have to use this before
 * we have malloc or even RTEMS workspace available
 */
static Triv121PgTblRec pgTbl={0};

#ifdef DEBUG
unsigned long
triv121PgTblConsistency(Triv121PgTbl pt, int pass);
#define CONS(pass,expect) do { \
		unsigned long r=triv121PgTblConsistency(&pgTbl,(pass)); \
		if ( expect>0 && r!=(expect)) { \
			printk("Wrong # of occupied slots detected during pass %i; should be %i (0x%x) is %i (0x%x)\n",\
				pass, (expect), r);	\
		}	\
		} while (0)
#else
#define CONS(pass,expect)
#endif

#define PTE_HASH1(vsid, pi) (((vsid)^(pi))&((1<<LD_HASH_SIZE)-1))
#define PTE_HASH2(hash1) ((~(hash1))&((1<<LD_HASH_SIZE)-1))

#define API(pi)	((pi)>>((LD_MIN_PT_SIZE)-(LD_PTEG_SIZE)))

static inline PTE
ptegOf(Triv121PgTbl pt, unsigned long hash)
{
	hash &= ((1<<LD_HASH_SIZE)-1);
	return (PTE)(((unsigned long)pt->base) | ((hash<<LD_PTEG_SIZE) & (pt->size-1)));
}

static PTE
alreadyMapped(Triv121PgTbl pt, unsigned long vsid, unsigned long pi)
{
int				i;
unsigned long	hash,api;
PTE				pte;

	if (!pt->size)
		return 0;

	hash = PTE_HASH1(vsid,pi);
	api=API(pi);
	for (i=0, pte=ptegOf(pt,hash); i<PTE_PER_PTEG; i++,pte++)
		if (pte->v && pte->vsid==vsid && pte->api==api && 0==pte->h)
			return pte;
	/* try the secondary hash table */
	hash = PTE_HASH2(hash);
	for (i=0, pte=ptegOf(pt,hash); i<PTE_PER_PTEG; i++,pte++)
		if (pte->v && pte->vsid==vsid && pte->api==api && 1==pte->h)
			return pte;
	return 0;
}

/* find the first available slot for  vsid/pi */
static PTE
slotFor(Triv121PgTbl pt, unsigned long vsid, unsigned long pi)
{
int				i;
unsigned long	hash,api;
PTE				pte;

	hash = PTE_HASH1(vsid,pi);
	api=API(pi);
	for (i=0, pte=ptegOf(pt,hash); i<PTE_PER_PTEG; i++,pte++) {
		if (!pte->v && !pte->marked) {
			/* mark this pte as potentially used */
			pte->h=0;
			pte->marked=1;
			return pte;
		}
	}
#ifdef DEBUG
	printk("## First hash bucket full - hash 0x%08x, pteg 0x%08x (vsid 0x%08x, pi 0x%08x)\n",
			hash, ptegOf(pt,hash), vsid, pi);
	for (i=0, pte=ptegOf(pt,hash); i<PTE_PER_PTEG; i++,pte++) {
		printk("pte 0x%08x is 0x%08x : 0x%08x\n",
			pte,*(unsigned long*)pte, *(((unsigned long*)pte)+1));
	}
#endif
	hash = PTE_HASH2(hash);
#ifdef DEBUG
	printk("   Secondary pteg is 0x%08x\n",ptegOf(pt,hash));
#endif
	for (i=0, pte=ptegOf(pt,hash); i<PTE_PER_PTEG; i++,pte++) {
		if (!pte->v && !pte->marked) {
			/* mark this pte as potentially used */
			pte->marked=1;
			pte->h=1;
			return pte;
		}
	}
#ifdef DEBUG
	printk("## Second hash bucket full - hash 0x%08x, pteg 0x%08x (vsid 0x%08x, pi 0x%08x)\n",
			hash, ptegOf(pt,hash), vsid, pi);
#endif
	return 0;
}

static void
unmarkAll(Triv121PgTbl pt)
{
unsigned long	n=pt->size / sizeof(PTERec);
unsigned long	i;
PTE				pte;
	for (i=0,pte=pt->base; i<n; i++,pte++)
		pte->marked=0;

}

unsigned long
triv121PgTblLdMinSize(unsigned long size)
{
unsigned long i;
	/* page align 'size' */
	size += (1<<LD_PG_SIZE)-1;
	size &= ~((1<<LD_PG_SIZE)-1);
	/* number of PTEs * sizeof(PTERec) */
	size >>= LD_PG_SIZE - LD_PTE_SIZE;
	/* find the next power of 2 >= size */
	for (i=0; i<LD_PHYS_SIZE; i++) {
		if ((1<<i) >= size)
			break;
	}
	return i;
}

/* initialize a trivial page table at 'base', allocate a size of 2^ldSize bytes */
Triv121PgTbl
triv121PgTblInit(unsigned long base, unsigned ldSize)
{
	if (pgTbl.size) {
		/* already initialized */
		return 0;
	}
	if (ldSize < LD_MIN_PT_SIZE)
		return 0; /* too small */
	if (base & ((1<<ldSize)-1))
		return 0; /* misaligned */
	pgTbl.base=(PTE)base;
	pgTbl.size=1<<ldSize;
	/* clear all page table entries */
	memset(pgTbl.base, 0, pgTbl.size);

	CONS(0,0);

	/* map the page table itself 'm' and 'readonly' */
	if (triv121PgTblMap(&pgTbl,
						TRIV121_121_VSID,
						base,
						(pgTbl.size >> LD_PG_SIZE),
						TRIV121_ATTR_M,
						TRIV121_PP_RO_PAGE) >= 0)
		return 0;

	CONS(1, (pgTbl.size>>LD_PG_SIZE));

	return &pgTbl;
}

Triv121PgTbl
triv121PgTblGet(void)
{
	return pgTbl.size ? &pgTbl : 0;
}

static void *ohdl;

static void myhdl(BSP_Exception_frame* excPtr)
{
if (0 && 3==excPtr->_EXC_number) {
	unsigned long dsisr;

	/* make this exception recoverable */
	excPtr->_EXC_number = ASM_DEC_VECTOR;
	/* reactivate DBAT0 */
	__asm__ __volatile__(
			"mfspr %0, %1\n"
			"ori	%0,%0,3\n"
			"mtspr %1, %0\n"
			"sync\n"
			"mfspr %0, %2\n":"=r"(dsisr):"i"(DBAT0U),"i"(DSISR));
	printk("DSISR 0x%08x\n",dsisr);
}
((void(*)())ohdl)(excPtr);
}

/* NOTE: this routine returns -1 on success;
 *       on failure, the page table index for
 *       which no PTE could be allocated is returned
 */
long
triv121PgTblMap(
				Triv121PgTbl	pt,
				long			vsid,
				unsigned long	start,
				unsigned long	numPages,
				unsigned		attributes,
				unsigned		protection
				)
{
int				i,pass;
unsigned long	pi;
PTE				pte;

	/* already active */
	if (pt->active)
			return -1;

	if (vsid < 0) {
			/* use 1:1 mapping */
			vsid = VSID121(start);
	}

#ifdef DEBUG
	printk("Mapping %i (0x%x) pages at 0x%08x for VSID 0x%08x\n",
		numPages, numPages, start, vsid);
#endif

	for (pass=0; pass<2; pass++) {
		/* check if we would succeed during the first pass */
		for (i=0, pi=PI121(start); i<numPages; i++,pi++) {
			if (!alreadyMapped(pt, vsid, pi)) {
				if (!(pte=slotFor(pt, vsid, pi))) {
					unmarkAll(pt);
					return pi; /* no free slot for page index 'pi' */
				} else {
					if (pass) {
						/* second pass; do the real work */
						pte->vsid=vsid;
						/* H was set by slotFor() */
						pte->api =API(pi);
						/* set up 1:1 mapping */
						pte->rpn =((((unsigned long)vsid)&((1<<(LD_PHYS_SIZE-LD_SEG_SIZE))-1))<<LD_PI_SIZE) | pi;
						pte->wimg=attributes & 0xf;
						pte->pp=protection&0x3;
						/* mark it valid */
						pte->v=1;
						pte->marked=0;
#ifdef DEBUG
assert(alreadyMapped(pt, vsid, pi) == pte);
#endif
					}
				}
			}
		}
		unmarkAll(pt);
	}
#ifdef DEBUG
	{
	unsigned long triv121IsRangeMapped();
	unsigned long failedat;
	CONS(100,-1);
	failedat=triv121IsRangeMapped(start, start + (1<<LD_PG_SIZE)*numPages);
	if (0x0C0C != failedat) {
		printk("triv121 mapping failed at 0x%08x\n",failedat);
		return PI121(failedat);
	}
	}
#endif
	return TRIV121_MAP_SUCCESS; /* -1 !! */
}

unsigned long
triv121PgTblSDR1(Triv121PgTbl pt)
{
	return (((unsigned long)pt->base) & ~(LD_MIN_PT_SIZE-1)) |
		   ( ((pt->size-1) >> LD_MIN_PT_SIZE) &
			 ((1<<(LD_HASH_SIZE-(LD_MIN_PT_SIZE-LD_PTEG_SIZE)))-1)
		   );
}

/* these are implemented by libbsp/powerpc/shared/start/start.S */
void MMUon(void);
void MMUoff(void);

void
triv121PgTblActivate(Triv121PgTbl pt)
{
#ifndef DEBUG_MAIN
unsigned long msr=0; /* keep compiler happy */
unsigned long sdr1=triv121PgTblSDR1(pt);
#endif
	pt->active=1;
#ifndef DEBUG_MAIN
	assert(current_ppc_cpu == PPC_750 || current_ppc_cpu == PPC_7400);

#warning TSILL
#if 0
	ohdl=globalExceptHdl;
	globalExceptHdl=myhdl;
__asm__ __volatile__ ("sync");
#endif

	/* get MSR and switch interrupts off - just in case */
	__asm__ __volatile__(
		"mfmsr %0\n"
		"andc  %0, %0, %2\n"
		"mtmsr %0\n"
		:"=r&"(msr):"0"(msr),"r"(MSR_EE)
	);
	/* switch MMU off if it is not off already */
	if ( msr & (MSR_IR|MSR_DR) )
		MMUoff();

	/* - load up the segment registers with a
	 *   1:1 effective <-> virtual mapping;
	 *   give user & supervisor keys
	 *
	 * - setup SDR1
	 *
	 * - Then flush all TLBs;
	 *   NOTE: the TLB flushing code is probably
	 *         CPU dependent!
	 */
	__asm__ __volatile(
		"	mtctr	%0\n"
		"	li		%0, 0\n"
		"1:	mtsrin	%1, %0\n"
		"	addis	%0, %0, 0x1000\n"	/* address next SR */
		"	addi	%1, %1, 1\n"		/* increment VSID  */
		"	bdnz	1b\n"
		/* set up SDR1 */
		"   mtspr	%4, %5\n"
		/* Now flush all TLBs, starting with the topmost index */
		"	lis		%0, %2@h\n"	
		"2:	addic.	%0, %0, -%3\n"		/* address the next one (decrementing) */
		"	tlbie	%0\n"				/* invalidate & repeat */
		"	bgt		2b\n"
		"	tlbsync\n"
		"	sync\n"
		"	li		%0, 0\n"
		"	mfspr	%1, %6\n"
		"	andc	%1, %1, %0\n"
		"	mtspr	%6,	%1\n"
		"	sync\n"
		::"b"(16), "b"(KEY_USR | KEY_SUP),
		  "i"(FLUSH_EA_RANGE), "i"(1<<LD_PG_SIZE),
		  "i"(SDR1), "r"(sdr1),
		  "i"(DBAT0U)
		: "ctr","cc");

	/* switch MMU back on if it was on on entry */
	if ( msr & (MSR_IR|MSR_DR) )
		MMUon();

	/* At this point, BAT0 is probably still active; it's the
	 * caller's job to deactivate it...
	 */

	/* restore original MSR  */
	__asm__ __volatile__("mtmsr %0"::"r"(msr));
#endif
}

#ifndef DEBUG_MAIN

#endif

#if defined(DEBUG_MAIN) || defined(DEBUG)
#include <stdlib.h>
#include <stdio.h>

static void
dumpPte(PTE pte)
{
int (*p)();
int printk();
	if (_Thread_Executing) {
		p=printf;
	} else {
		p=printk;
	}
	if (0==((unsigned long)pte & ((1<<LD_PTEG_SIZE)-1)))
		p("PTEG--");
	else
		p("......");
	if (pte->v) {
		p("VSID: 0x%08x H:%1i API: 0x%02x\n",
					pte->vsid, pte->h, pte->api);
		p("      ");
		p("RPN:  0x%08x WIMG: 0x%1x, (m %1i), pp: 0x%1x\n",
					pte->rpn, pte->wimg, pte->marked, pte->pp);
	} else {
		p("xxxxxx\n");
		p("      ");
		p("xxxxxx\n");
	}
}

#ifdef DEBUG
PTE
triv121DumpPte(unsigned long ea)
{
PTE pte;

	pte=alreadyMapped(
					&pgTbl,
					VSID121(ea),
					PI121(ea)
					);
	if (pte)
		dumpPte(pte);
	return pte;
}

unsigned long
triv121IsRangeMapped(unsigned long start, unsigned long end)
{
	start&=~((1<<LD_PG_SIZE)-1);
	while (start < end) {
		if (!alreadyMapped(&pgTbl,VSID121(start),PI121(start)))
			return start;
		start+=1<<LD_PG_SIZE;
	}
	return 0x0C0C; /* OKOK - not on a page boundary */
}
#endif


#ifndef DEBUG
static
#endif
int
triv121PgTblDump(Triv121PgTbl pt, unsigned from, unsigned to)
{
int i;
PTE	pte;
	printf("Dumping PT [size 0x%08lx == %li] at 0x%08lx\n", pt->size, pt->size, (unsigned long)pt->base);
	if (from> pt->size>>LD_PTE_SIZE)
		from=0;
	if (to  > pt->size>>LD_PTE_SIZE)
		to=(pt->size>>LD_PTE_SIZE)-1;
	for (i=from,pte=pt->base+from; i<=(long)to; i++, pte++) {
		dumpPte(pte);
	}
	return 0;
}

unsigned long
triv121PgTblConsistency(Triv121PgTbl pt, int pass)
{
PTE pte;
int i;
unsigned long v,m;
int warn=0;
static int maxw=20;

	printk("Checking page table at %p (size %i==0x%x)\n",
			pt->base, pt->size, pt->size);

	if (!pt->base || !pt->size) {
		printk("Uninitialized Page Table!\n");
		return 0;
	}

	v=m=0;
	for (i=0, pte=pt->base; i<pt->size/sizeof(PTERec); i++,pte++) {
		int				err=0;
		char			buf[500];
		unsigned long	*lp=(unsigned long*)pte;
		if ( (*lp & (0xfffff0<<7)) || *(lp+1) & 0xe00 || (pte->v && pte->marked)) {
			/* check for vsid (without segment bits) == 0, unused bits == 0, valid && marked */
			sprintf(buf,"invalid VSID , unused bits or v && m");
			err=1;
		} else {
			if (pte->v) v++;
			if (pte->marked) m++;
		}
		if (err && maxw) {
			printk("Pass %i -- strange PTE at 0x%08x found for page index %i == 0x%08x:\n",
				pass,pte,i,i);
			printk("Reason: %s\n",buf);
			dumpPte(pte);
			warn++;
			maxw--;
		}
	}
	if (warn) {
		printk("%i errors found; currently %i entries marked, %i are valid\n",
			warn, m, v);
	}
	return m+v;
}


#if defined(DEBUG_MAIN)

#define LD_DBG_PT_SIZE	LD_MIN_PT_SIZE

int
main(int argc, char **argv)
{
unsigned long	base,start,numPages;
unsigned long	size=1<<LD_DBG_PT_SIZE;
Triv121PgTbl	pt;

	base=(unsigned long)malloc(size<<1);

	assert(base);

	/* align pt */
	base += size-1;
	base &= ~(size-1);

	assert(pt=triv121PgTblInit(base,LD_DBG_PT_SIZE));

	triv121PgTblDump(pt,(unsigned)-1, (unsigned)-1);
	do {
		do {
		printf("Start Address:"); fflush(stdout);
		} while (1!=scanf("%i",&start));
		do {
		printf("# pages:"); fflush(stdout);
		} while (1!=scanf("%i",&numPages));
	} while (TRIV121_MAP_SUCCESS==triv121PgTblMap(pt,TRIV121_121_VSID,start,numPages,
							TRIV121_ATTR_IO_PAGE,2) &&
			0==triv121PgTblDump(pt));
}
#endif
#endif
