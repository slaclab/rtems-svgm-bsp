/* $Id$ */

#include <rtems.h>
#include <libcpu/mmu.h>
#include <libcpu/page.h>
#include <rtems/bspIo.h>
#include <libcpu/pte121.h>


/* Default setup of the page tables. This is a weak
 * alias, so applications may easily override this
 * default setup.
 *
 * NOTE: while it is possible to change the individual
 *       mappings, the page table itself MUST be
 *       allocated at the top of the physical memory!
 *       bspstart.c RELIES on this.
 *       Also, the 'setup' routine must reduce
 *       *pmemsize by the size of the page table.
 */

/* Author: Till Straumann, <strauman@slac.stanford.edu>, 4/2002 */

Triv121PgTbl
BSP_pgtbl_setup(unsigned long) __attribute__ (( weak, alias("__VGM_default_pgtbl_setup") ));


Triv121PgTbl
__VGM_default_pgtbl_setup(unsigned int *pmemsize)
{
Triv121PgTbl	pt;
unsigned		ldPtSize,tmp;

  /* Allocate a page table large enough to map
   * the entire physical memory. We put the page
   * table at the top of the physical memory.
   */

  /* get minimal size (log base 2) of PT for
   * this board's memory
   */
  ldPtSize = triv121PgTblLdMinSize(*pmemsize);
  ldPtSize++; /* double this amount -- then why? */

  /* allocate the page table at the top of the physical
   * memory - THIS IS NOT AN OPTION - bspstart.c RELIES
   * ON THIS LAYOUT! (the size, however may be changed)
   */
  if ( (pt = triv121PgTblInit(*pmemsize - (1<<ldPtSize), ldPtSize)) ) {
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
						(*pmemsize - (1<<ldPtSize) -  (unsigned long)&__DATA_START__ )>> PG_SHIFT,
						0, /* WIMG */
						TRIV121_PP_RW_PAGE);
		if (TRIV121_MAP_SUCCESS != tmp) {
			printk("Unable to map page index %i; reverting to BAT0\n", 
					tmp);
			pt = 0;
		}
	}
  } else {
	printk("WARNING: unable to allocate page table, keeping DBAT0\n");
  }
  if (pt) {
#ifdef SHOW_MORE_INIT_SETTINGS
	printk("Setting up page table mappings; protecting text/read-only data from write access\n");
#endif
	/* SUCCESS; reduce available memory by size of the page table */
	*pmemsize -= (1<<ldPtSize);
  }
  return pt;
}
