/*
 * Mini-loader for the SVGM BSP.
 *
 * $Id$
 *
 * Author: Till Straumann, 10/2001 <strauman@slac.stanford.edu>
 *
 * Some ideas are borrowed from the powerpc/shared/bootloader
 * by
 *  Copyright (C) 1998, 1999 Gabriel Paubert, paubert@iram.es
 *  Copyright (C) 1999 Eric Valette. valette@crf.canon.fr
 *
 * The SMON firmware is unable to load the RTEMS image below
 * 0x2000 (I believe their stack is growing below 0x1000).
 * 
 * The code provided by this file is responsible for the performing
 * the following steps:
 *
 *  1) Move the entire image (including this very file) to
 *     its final location starting at 0x0000.
 *     It is important to note that _NO_STACK_ is available
 *     during this step. Also, there is probably no return to
 *     SMON because relocating RTEMS will destroy vital SMON
 *     data (such as its stack).
 *  2) Flush the cache to make sure the relocated image is actually
 *     in memory.
 *  3) setup RTEMS environment (initial register values), most 
 *     notably an initial STACK. The initial stack may be small and
 *     is used by RTEMS only at a very early stage.
 *     A safe place for the stack seems to be the 00..0x100 area.
 *  4) switch the MMU off (because that's what RTEMS is expecting
 *     it to be at startup).
 *  5) fire up rtems...
 *
 *
 *  Calling convention:
 *     R1: SMON SP
 *     R5: where SMON put the image
 *     R6: end of the image (i.e. R6-R5 is the image length)
 *     R7: the machine's cache line size - MUST be set
 *         by a script!
 *
 *  ASSUMPTIONS:
 *    The code RELIES on the assumption that the image will be
 *    moved DOWNWARDS in memory and that the this loader is
 *    prepended to the image, i.e. it is safe to do
 *        codemove(codemove,0,codemove_end - codemove);
 *        (*0)(codemove_end, codemove_end-codemove, __rtems_end-codemove_end);
 *    where codemove(from, to, nbytes) is defined as
 *        codemove(from, to, nbytes) { while (nbytes--) *(to++)=*(from++); }
 *    Implicit to these assumptions is the assumption that the destination
 *    address is cache block aligned.
 *    Furthermore, the byte count is assumed to be a multiple
 *    of four
 */
#if 0
/* TODO: I dont know where the appropriate CPU model is to be defined
 *       when including this to get PPC_CACHE_ALIGNMENT I get an error...
 */
#include <rtems/score/ppc.h>
#else
#ifndef PPC_CACHE_ALIGNMENT
#define PPC_CACHE_ALIGNMENT	32
#endif
#endif
#include <libcpu/cpu.h>
#include "asm.h"

/* Note that major modifications may be needed
 * if DESTINATION_ADDR is not 0
 */
#define KERNELBASE		0x0
#define INITIAL_STACK		0xf8	/* 8-byte aligned */
#define CACHE_LINE_SIZE		PPC_CACHE_ALIGNMENT 	/* autodetect doesn't work, see below */
#define	ASSUME_RTEMS_INSTALLS_VECTORS	/* assume we need not load vectors */

	/* put this into its own section which we want to
         * be loaded at the very beginning. We should probably
         * not use more than 255 bytes.
	 */
	PUBLIC_VAR(__rtems_entry_point)
	.section .svgm_preloader_section,"awx",@progbits
preloader:
	/* check if we are where we think we are, i.e.
	 * 'here' should be at the address we got passed
	 * in r5 + 4.
	 */
	bl	here
here:	mflr	r0
	sub	r0,r0,r5
	cmpwi	r0,4
	bne	panic
	sub	r30,r6,r5	/* save image length */

#ifndef CACHE_LINE_SIZE
	/* Oh well, SMON has inhibited the cache, so this
         * nice routine doesn't work...
         */
	/* figure out the cache line size */
	li	r6, 0x100
	cmpw	r5, r6		/* 'from' must be > 0x100 */
	blt	panic
	
1:	/* store some arbitrary, nonzero stuff in 0..0xfc */
	stwu	r6,-4(r6)
	cmpwi	r6,0
	bne	1b
	sync			/* make sure it's there */
	dcbz	0,r6		/* zero out one cache line */
	subi	r6,r6,4
2:	lwzu	r0,4(r6)	/* search for a non-zero word */
	cmpwi	r0,0
	beq	2b
	/* OK, r6 now hold the size of a cache line in bytes */
#else
	li	r6,CACHE_LINE_SIZE
#endif
	
	li	r3,KERNELBASE
	mr	r4,r5		/* from-addr */
	li	r5,_preload_size/* this is never > 16k */
	/* now move ourselves to KERNELBASE.
	 * We set up the LR, so domove() 'returns' to the
	 * relocated copy
	 */
	li	r0,return_here@l
	mtlr	r0	
	b	domove		/* move the preloader itself */
return_here:
	/* now we move the entire rest of the image */
#ifdef ASSUME_RTEMS_INSTALLS_VECTORS
	lis	r3,__rtems_entry_point@h
	ori	r3,r3,__rtems_entry_point@l
	add	r4,r4,r3	/* should add __rtems_entry_point - KERNELBASE */
	add	r5,r5,r3	/* but that is 0 */
	sub	r5,r30,r3
#else
	addi	r3,r3,r5	/* add preloader size to destination */
	addi	r4,r4,r5	/* and source addresses	*/
	sub	r5,r30,r5	/* length of the remaining rest */
#endif
	bl	domove
	/* OK, now everything should be in place.
         * we are ready to start...
	 */

	/* setup initial stack for rtems early boot */
	li	r1,INITIAL_STACK
	/* disable the MMU and fire up rtems */
	mfmsr	r0
	ori 	r0,r0,MSR_IR|MSR_DR|MSR_IP
	xori	r0,r0,MSR_IR|MSR_DR
	mtsrr1	r0
#ifdef ASSUME_RTEMS_INSTALLS_VECTORS
	mtsrr0	r3
#else
	lis	r0,__rtems_entry_point@h
	ori	r0,r0,__rtems_entry_point@l
	mtsrr0	r0
#endif
	rfi

	/* domove(to, from, nbytes):
         *
	 * move a R5 bytes from R4 to R3 and flush
         * the caches for the destination memory
         * region. R6 provides the cache line size.
         */
domove:
	addi	r0,r5,3		/* convert to word count */
	srwi.	r0,r0,2
	beq	3f		/* nothing to do */
	mtctr	r0
	la	r8,-4(r4)
	la	r7,-4(r3)
1:	lwzu	r0,4(r8)
	stwu	r0,4(r7)
	bdnz	1b		/* move data */
	/* now, we must flush the destination cache region */
	cmpwi	r6,0
	beq	3f		/* nothing to do */
	add	r7,r3,r5	/* target end pointer */
	subi	r0,r6,1
	add	r7,r7,r0
	andc	r7,r7,r0	/* cache aligned target end pointer */
	mr	r8,r3
2:	cmpw	r8,r7
	dcbst	0,r8		/* write out data cache line */
	icbi	0,r8		/* invalidate corresponding i-cache line */
	add	r8,r8,r6
	blt	2b
	sync			/* make sure data is written back */
	isync			/* invalidate possibly preloaded instructions */
3:
	blr
panic:
	li	r10,0x63
	mfmsr	r0
	ori	r0,r0,MSR_IP
	mtmsr	r0
	sc

/* DONT PUT ANY CODE BELOW HERE */
_preload_size = . - preloader
