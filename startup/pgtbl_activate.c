/* $Id$ */

#include <rtems.h>
#include <libcpu/pte121.h>

/* Default activation of the page tables. This is a weak
 * alias, so applications may easily override this
 * default activation procedure.
 */

/* Author: Till Straumann, <strauman@slac.stanford.edu>, 4/2002 */

void
BSP_pgtbl_activate(Triv121PgTbl) __attribute__ (( weak, alias("__VGM_default_pgtbl_activate") ));

/* bat valid bits */
#define  BAT_VS 2
#define  BAT_VP 1

void
__VGM_default_pgtbl_activate(Triv121PgTbl pt)
{
	if (!pt) return;

	/* switch the text/ro sements to RO only after
	 * initializing the interrupts because the irq_mng
	 * installs some code...
	 *
	 * activate the page table; it is still masked by the
	 * DBAT0, however
	 */
	triv121PgTblActivate(pt);

	/* finally, switch off DBAT0 */
	__asm__ __volatile__(
			"mfspr %%r0, %0\n"
			"andc  %%r0, %%r0, %1\n"
			"sync;\n"
			"mtspr %0, %%r0\n"
			"sync\n"
			::"i"(DBAT0U),"r"(BAT_VS | BAT_VP)
			:"r0"
	);
	/* At this point, DBAT0 is available for other use... */
}
