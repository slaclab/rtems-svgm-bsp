#ifndef PPC_TRIVIAL_PTE_MAPPING_H
#define PPC_TRIVIAL_PTE_MAPPING_H
/* $Id$ */

typedef struct Triv121PgTblRec_ *Triv121PgTbl;

/* initialize a trivial page table at 'base', allocate a size of 2^ldSize bytes */
Triv121PgTbl
triv121PgTblInit(unsigned long base, unsigned ldSize);

/* get the log2 of the minimal page size needed
 * for mapping 'size' bytes
 */
unsigned long
triv121PgTblLdMinSize(unsigned long size);

/* Map an address range 1:1 in pgTbl with the given protection;
 *
 * NOTE: this routine returns -1 (TRIV121_MAP_SUCCESS) on success;
 *       on failure, the page table index for
 *       which no PTE could be allocated is returned
 */
long
triv121PgTblMap(
				Triv121PgTbl  pgTbl,			/* handle, returned by Init or Get */
				long          vsid,				/* vsid for this mapping (contains topmost 4 bits of EA);
												 * NOTE: it is allowed to pass a VSID < 0 to tell this
												 *       routine it should use a VSID corresponding to a
												 *       1:1:1  effective - virtual - physical  mapping
												 */
				unsigned long start,			/* start of address range (lowermost 28 bits of EA) */
				unsigned long numPages,			/* number of pages to map */
				unsigned wimgAttr,				/* 'wimg' attributes
												 * (Write thru, cache Inhibit, coherent Memory,
												 * Guarded memory)
												 */
				unsigned protection				/* access protection:
												 * 0: Sup r/w; User none 1: Sup r/w, User r;
												 * 2; Sup r/w, User r/w; 3: Sup ro, User ro
												 */
				);

#define TRIV121_ATTR_W	8	
#define TRIV121_ATTR_I	4
#define TRIV121_ATTR_M	2
#define TRIV121_ATTR_G	1

#define TRIV121_ATTR_IO_PAGE	(TRIV121_ATTR_I|TRIV121_ATTR_G)

#define TRIV121_PP_RO_PAGE		(3)
#define TRIV121_PP_RW_PAGE		(2)

#define TRIV121_121_VSID		(-1)

#define TRIV121_MAP_SUCCESS		(-1)

/* get a handle to the one and only page table
 * (must have been initialized)
 */
Triv121PgTbl
triv121PgTblGet(void);

/*
 * compute the SDR1 register value for the page table
 */

unsigned long
triv121PgTblSDR1(Triv121PgTbl pgTbl);

/*
 * Activate the page table:
 *	- set up the segment registers for a 1:1 effective <-> virtual address mapping,
 *    give user and supervisor keys.
 *  - set up the SDR1 register
 *  - flush all tlbs
 *  - 'lock' pgTbl, i.e. prevent all further modifications.
 *
 */
void
triv121PgTblActivate(Triv121PgTbl pgTbl);

#endif
