/*
 *  pci.c -- Crude pci handling for early boot.
 *
 *  Copyright (C) 1998, 1999 Gabriel Paubert, paubert@iram.es
 *
 *  Modified to compile in RTEMS development environment
 *  by Eric Valette
 *
 *  Copyright (C) 1999 Eric Valette. valette@crf.canon.fr
 *
 *  The license and distribution terms for this file may be
 *  found in found in the file LICENSE in this distribution or at
 *  http://www.OARcorp.com/rtems/license.html.
 *
 * $Id$
 */


#include <sys/types.h>
#include <libcpu/spr.h>
#include "../../shared/bootloader/bootldr.h"
#include "../../shared/bootloader/pci.h"
#include <libcpu/io.h>
#include <bsp/consoleIo.h>

typedef unsigned int u32;

#define PCI_DEBUG
/* Used to reorganize PCI space on stupid machines which spread resources
 * across a wide address space. This is bad when P2P bridges are present
 * or when it limits the mappings that a resource hog like a PCI<->VME
 * bridge can use.
 */

typedef struct _pci_resource {
	struct _pci_resource *next;
	struct pci_dev *dev; 
  	u_long base;    /* will be 64 bits on 64 bits machines */
	u_long size;
  	u_char type;	/* 1 is I/O else low order 4 bits of the memory type */
	u_char reg;	/* Register # in conf space header */
  	u_short cmd;    /* Original cmd byte */
} pci_resource;

typedef struct _pci_area {
	struct _pci_area *next;
	u_long start;
	u_long end;
	struct pci_bus *bus;
	u_int flags;
} pci_area;

typedef struct _pci_area_head {
	pci_area *head;
	u_long mask;
	int high;	/* To allocate from top */
} pci_area_head;

#define PCI_AREA_PREFETCHABLE 0
#define PCI_AREA_MEMORY 1
#define PCI_AREA_IO 2

struct _pci_private {
	volatile u_int * config_addr;
	volatile u_char * config_data;
	struct pci_dev **last_dev_p;
  	struct pci_bus pci_root;
	pci_resource *resources;
	pci_area_head io, mem;

} pci_private = {
	config_addr: NULL, 
        config_data: (volatile u_char *) 0x80800000, 
        last_dev_p: NULL, 
	resources: NULL,
	io: {NULL, 0xfff, 0},
	mem: {NULL, 0xfffff, 0}
};

#define pci ((struct _pci_private *)(bd->pci_private))
#define pci_root pci->pci_root

#if !defined(DEBUG)
#undef PCI_DEBUG
/*
  #else
  #define PCI_DEBUG
*/
#endif

#if defined(PCI_DEBUG)
static void 
print_pci_resources(const char *s) {
	pci_resource *p;
	printk("%s", s);
	for (p=pci->resources; p; p=p->next) {
		printk("  %p:%p %06x %08lx %08lx %d\n", 
		       p, p->next,
		       (p->dev->devfn<<8)+(p->dev->bus->number<<16)
		       +0x10+p->reg*4,
		       p->base,
		       p->size,
		       p->type); 
	}
}

static void 
print_pci_area(pci_area *p) {
	for (; p; p=p->next) {
		printk("    %p:%p %p %08lx %08lx\n",
		       p, p->next, p->bus, p->start, p->end);
	}
}

static void 
print_pci_areas(const char *s) {
	printk("%s  PCI I/O areas:\n",s);
	print_pci_area(pci->io.head);
	printk("  PCI memory areas:\n");
	print_pci_area(pci->mem.head);
}
#else
#define print_pci_areas(x) 
#define print_pci_resources(x)
#endif

/* Maybe there are some devices who use a size different
 * from the alignment. For now we assume both are the same.
 * The blacklist might be used for other weird things in the future too,
 * since weird non PCI complying devices seem to proliferate these days.
 */

struct blacklist_entry {
	u_short vendor, device;
	u_char reg;
	u_long actual_size;
};

#define BLACKLIST(vid, did, breg, actual_size) \
	{PCI_VENDOR_ID_##vid, PCI_DEVICE_ID_##vid##_##did, breg, actual_size} 

static struct blacklist_entry blacklist[] = {
	BLACKLIST(S3, TRIO, 0, 0x04000000),
	{0xffff, 0, 0, 0}
};


/* This function filters resources and then inserts them into a list of
 * configurable pci resources. 
 */


#define AREA(r) \
(((r->type&PCI_BASE_ADDRESS_SPACE)==PCI_BASE_ADDRESS_SPACE_IO) ? PCI_AREA_IO :\
	  ((r->type&PCI_BASE_ADDRESS_MEM_PREFETCH) ? PCI_AREA_PREFETCHABLE :\
	   PCI_AREA_MEMORY))

static int insert_before(pci_resource *e, pci_resource *t) {
	if (e->dev->bus->number != t->dev->bus->number) 
	  return e->dev->bus->number > t->dev->bus->number;
	if (AREA(e) != AREA(t)) return AREA(e)<AREA(t);
	return (e->size > t->size);
}

static void insert_resource(pci_resource *r) {
	struct blacklist_entry *b;
	pci_resource *p;
	if (!r) return;

	/* First fixup in case we have a blacklist entry. Note that this 
	 * may temporarily leave a resource in an inconsistent state: with 
	 * (base & (size-1)) !=0. This is harmless.
	 */
	for (b=blacklist; b->vendor!=0xffff; b++) {
		if ((r->dev->vendor==b->vendor) &&
		    (r->dev->device==b->device) &&
		    (r->reg==b->reg)) {
			r->size=b->actual_size;
			break;
		}
	}
	
	/* Motorola NT firmware does not configure pci devices which are not
 	 * required for booting, others do. For now:
	 * - allocated devices in the ISA range (64kB I/O, 16Mb memory)
	 *   but non zero base registers are left as is.
	 * - all other registers, whether already allocated or not, are 
	 *   reallocated unless they require an inordinate amount of 
	 *   resources (>256 Mb for memory >64kB for I/O). These
	 *   devices with too large mapping requirements are simply ignored
	 *   and their bases are set to 0. This should disable the
	 *   corresponding decoders according to the PCI specification.
	 *   Many devices are buggy in this respect, however, but the
	 *   limits have hopefully been set high enough to avoid problems.
	 */

	if ((r->type==PCI_BASE_ADDRESS_SPACE_IO) 
	    ? (r->base && r->base <0x10000)
	    : (r->base && r->base <0x1000000)) {
		sfree(r);
		return;
	}

	if ((r->type==PCI_BASE_ADDRESS_SPACE_IO) 
	    ? (r->size >= 0x10000)
	    : (r->size >= 0x10000000)) {
		r->size  = 0;
		r->base  = 0;
	}

	/* Now insert into the list sorting by 
	 * 1) decreasing bus number
	 * 2) space: prefetchable memory, non-prefetchable and finally I/O
	 * 3) decreasing size
	 */
	if (!pci->resources || insert_before(r, pci->resources)) {
		r->next = pci->resources;
		pci->resources=r;
	} else {
		for (p=pci->resources; p->next; p=p->next) {
			if (insert_before(r, p->next)) break;
		}
		r->next=p->next;
		p->next=r;
	}
}

/* This version only works for bus 0. I don't have any P2P bridges to test 
 * a more sophisticated version which has therefore not been implemented.
 * Prefetchable memory is not yet handled correctly either.
 * And several levels of PCI bridges much less even since there must be
 * allocated together to be able to setup correctly the top bridge. 
 */

static u_long find_range(u_char bus, u_char type, 
		       pci_resource **first,
		       pci_resource **past, u_int *flags) {
	pci_resource *p;
	u_long total=0;
	u_int fl=0;

	for (p=pci->resources; p; p=p->next) {
		if ((p->dev->bus->number == bus) &&
		    AREA(p)==type) break;
	}
	*first = p;
	for (; p; p=p->next) {
		if ((p->dev->bus->number != bus) ||
		    AREA(p)!=type || p->size == 0) break;
		total = total+p->size;
		fl |= 1<<p->type; 
	}
	*past = p;
	/* This will be used later to tell whether there are any 32 bit
	 * devices in an area which could be mapped higher than 4Gb
	 * on 64 bits architectures
	 */
	*flags = fl;
	return total;
}

static inline void init_free_area(pci_area_head *h, u_long start, 
			    u_long end, u_int mask, int high) {
	pci_area *p;
	p = salloc(sizeof(pci_area));
	if (!p) return;
	h->head = p;
	p->next = NULL;
	p->start = (start+mask)&~mask;
	p->end = (end-mask)|mask;
	p->bus = NULL;
	h->mask = mask;
	h->high = high;
}

static void insert_area(pci_area_head *h, pci_area *p) {
	pci_area *q = h->head;
	if (!p) return;
	if (q && (q->start< p->start)) {
        	for(;q->next && q->next->start<p->start; q = q->next);
		if ((q->end >= p->start) ||
		    (q->next && p->end>=q->next->start)) {
			sfree(p);
			printk("Overlapping pci areas!\n");
			return;
		}
		p->next = q->next;
		q->next = p;
	} else { /* Insert at head */
	  	if (q && (p->end >= q->start)) {
		  	sfree(p);
			printk("Overlapping pci areas!\n");
			return;
		}
		p->next = q;
		h->head = p;
	}
}

static
void remove_area(pci_area_head *h, pci_area *p) {
	pci_area *q = h->head;

	if (!p || !q) return;
  	if (q==p) {
		h->head = q->next;
		return;
	}
	for(;q && q->next!=p; q=q->next);
	if (q) q->next=p->next;
}

static pci_area * alloc_area(pci_area_head *h, struct pci_bus *bus,
				u_long required, u_long mask, u_int flags) {
	pci_area *p;
	pci_area *from, *split, *new;

	required = (required+h->mask) & ~h->mask;
	for (p=h->head, from=NULL; p; p=p->next) {
		u_long l1 = ((p->start+required+mask)&~mask)-1;
		u_long l2 = ((p->start+mask)&~mask)+required-1;
		/* Allocated areas point to the bus to which they pertain */
		if (p->bus) continue;
		if ((p->end)>=l1 || (p->end)>=l2) from=p;
		if (from && !h->high) break;
	}
	if (!from) return NULL;

	split = salloc(sizeof(pci_area));
	new = salloc(sizeof(pci_area));
	/* If allocation of new succeeds then allocation of split has
	 * also been successful (given the current mm algorithms) !
	 */
	if (!new) { 
		sfree(split); 
		return NULL; 
	}
	new->bus = bus;
	new->flags = flags;
	/* Now allocate pci_space taking alignment into account ! */
	if (h->high) {
		u_long l1 = ((from->end+1)&~mask)-required;
		u_long l2 = (from->end+1-required)&~mask; 
		new->start = (l1>l2) ? l1 : l2;
		split->end = from->end;
		from->end = new->start-1;
		split->start = new->start+required;
		new->end = new->start+required-1;
	} else {
		u_long l1 = ((from->start+mask)&~mask)+required-1;
		u_long l2 = ((from->start+required+mask)&~mask)-1; 
		new->end = (l1<l2) ? l1 : l2;
		split->start = from->start;
		from->start = new->end+1;
		new->start = new->end+1-required;
		split->end = new->start-1;
	}
	
	if (from->end+1 == from->start) remove_area(h, from);
	if (split->end+1 != split->start) {
		split->bus = NULL;
		insert_area(h, split);
	} else {
		sfree(split);
	}
	insert_area(h, new);
	print_pci_areas("alloc_area called:\n");
	return new;
}

static inline
void alloc_space(pci_area *p, pci_resource *r) {
	if (p->start & (r->size-1)) {
		r->base = p->end+1-r->size;
		p->end -= r->size;
	} else {
		r->base = p->start;
		p->start += r->size;
	}
}

static void reconfigure_bus_space(u_char bus, u_char type, pci_area_head *h) {
	pci_resource *first, *past, *r;
	pci_area *area, tmp;
	u_int flags;
	u_int required = find_range(bus, type, &first, &past, &flags);

	if (required==0) return;
	area = alloc_area(h, first->dev->bus, required, first->size-1, flags);
	if (!area) return;
	tmp = *area;
	for (r=first; r!=past; r=r->next) {
	  	alloc_space(&tmp, r);
	}
}

static void reconfigure_pci(void) {
	pci_resource *r;
	struct pci_dev *dev;
	/* FIXME: for now memory is relocated from low, it's better
	 * to start from higher addresses.
	 */
	init_free_area(&pci->io, 0x10000, 0x7fffff, 0xfff, 0);
	init_free_area(&pci->mem, 0x1000000, 0x3cffffff, 0xfffff, 0);

  	/* First reconfigure the I/O space, this will be more
	 * complex when there is more than 1 bus. And 64 bits 
	 * devices are another kind of problems. 
	 */
	reconfigure_bus_space(0, PCI_AREA_IO, &pci->io);
	reconfigure_bus_space(0, PCI_AREA_MEMORY, &pci->mem);
	reconfigure_bus_space(0, PCI_AREA_PREFETCHABLE, &pci->mem);

	/* Now we have to touch the configuration space of all
	 * the devices to remap them better than they are right now.
	 * This is done in 3 steps: 
	 * 1) first disable I/O and memory response of all devices
	 * 2) modify the base registers
	 * 3) restore the original PCI_COMMAND register.
	 */
	for (r=pci->resources; r; r= r->next) {
		if (!r->dev->sysdata) {
			r->dev->sysdata=r;
			pci_read_config_word(r->dev, PCI_COMMAND, &r->cmd);
			pci_write_config_word(r->dev, PCI_COMMAND,
					      r->cmd & ~(PCI_COMMAND_IO|
							 PCI_COMMAND_MEMORY));
		}
	}

	for (r=pci->resources; r; r= r->next) {
	  	pci_write_config_dword(r->dev, 
				       PCI_BASE_ADDRESS_0+(r->reg<<2),
				       r->base);
		if ((r->type&
		     (PCI_BASE_ADDRESS_SPACE|
		      PCI_BASE_ADDRESS_MEM_TYPE_MASK)) == 
		    (PCI_BASE_ADDRESS_SPACE_MEMORY|
		     PCI_BASE_ADDRESS_MEM_TYPE_64)) {
		  	pci_write_config_dword(r->dev,
					       PCI_BASE_ADDRESS_1+
					       (r->reg<<2),
					       0);
		}
	}
	for (dev=bd->pci_devices; dev; dev= dev->next) {
	  	if (dev->sysdata) {
			pci_write_config_word(dev, PCI_COMMAND,
					      ((pci_resource *)dev->sysdata)
					      ->cmd);
			dev->sysdata=NULL;
		}
	}
}

static int
indirect_pci_read_config_byte(unsigned char bus, unsigned char dev_fn, 
			      unsigned char offset, unsigned char *val) {
	out_be32(pci->config_addr, 
		 0x80|(bus<<8)|(dev_fn<<16)|((offset&~3)<<24));
	*val=in_8(pci->config_data + (offset&3));
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_read_config_word(unsigned char bus, unsigned char dev_fn, 
			      unsigned char offset, unsigned short *val) {
	*val = 0xffff; 
	if (offset&1) return PCIBIOS_BAD_REGISTER_NUMBER;
	out_be32(pci->config_addr, 
		 0x80|(bus<<8)|(dev_fn<<16)|((offset&~3)<<24));
	*val=in_le16((volatile u_short *)(pci->config_data + (offset&3)));
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_read_config_dword(unsigned char bus, unsigned char dev_fn, 
			      unsigned char offset, unsigned int *val) {
	*val = 0xffffffff; 
	if (offset&3) return PCIBIOS_BAD_REGISTER_NUMBER;
	out_be32(pci->config_addr, 
		 0x80|(bus<<8)|(dev_fn<<16)|(offset<<24));
	*val=in_le32((volatile u_int *)pci->config_data);
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_write_config_byte(unsigned char bus, unsigned char dev_fn, 
			       unsigned char offset, unsigned char val) {
	out_be32(pci->config_addr, 
		 0x80|(bus<<8)|(dev_fn<<16)|((offset&~3)<<24));
	out_8(pci->config_data + (offset&3), val);
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_write_config_word(unsigned char bus, unsigned char dev_fn, 
			       unsigned char offset, unsigned short val) {
	if (offset&1) return PCIBIOS_BAD_REGISTER_NUMBER;
	out_be32(pci->config_addr, 
		 0x80|(bus<<8)|(dev_fn<<16)|((offset&~3)<<24));
	out_le16((volatile u_short *)(pci->config_data + (offset&3)), val);
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_pci_write_config_dword(unsigned char bus, unsigned char dev_fn, 
				unsigned char offset, unsigned int val) {
	if (offset&3) return PCIBIOS_BAD_REGISTER_NUMBER;
	out_be32(pci->config_addr, 
		 0x80|(bus<<8)|(dev_fn<<16)|(offset<<24));
	out_le32((volatile u_int *)pci->config_data, val);
	return PCIBIOS_SUCCESSFUL;
}

static const struct pci_config_access_functions indirect_functions = {
  	indirect_pci_read_config_byte,
  	indirect_pci_read_config_word,
  	indirect_pci_read_config_dword,
  	indirect_pci_write_config_byte,
  	indirect_pci_write_config_word,
  	indirect_pci_write_config_dword
};


static int
direct_pci_read_config_byte(unsigned char bus, unsigned char dev_fn, 
			      unsigned char offset, unsigned char *val) {
	if (bus != 0 || (1<<PCI_SLOT(dev_fn) & 0xff8007fe)) {
		*val=0xff;
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	*val=in_8(pci->config_data + ((1<<PCI_SLOT(dev_fn))&~1) 
		  + (PCI_FUNC(dev_fn)<<8) + offset);
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_read_config_word(unsigned char bus, unsigned char dev_fn, 
			      unsigned char offset, unsigned short *val) {
	*val = 0xffff; 
	if (offset&1) return PCIBIOS_BAD_REGISTER_NUMBER;
	if (bus != 0 || (1<<PCI_SLOT(dev_fn) & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	*val=in_le16((volatile u_short *)
		     (pci->config_data + ((1<<PCI_SLOT(dev_fn))&~1)
		      + (PCI_FUNC(dev_fn)<<8) + offset));
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_read_config_dword(unsigned char bus, unsigned char dev_fn, 
			      unsigned char offset, unsigned int *val) {
	*val = 0xffffffff; 
	if (offset&3) return PCIBIOS_BAD_REGISTER_NUMBER;
	if (bus != 0 || (1<<PCI_SLOT(dev_fn) & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	*val=in_le32((volatile u_int *)
		     (pci->config_data + ((1<<PCI_SLOT(dev_fn))&~1)
		      + (PCI_FUNC(dev_fn)<<8) + offset));
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_write_config_byte(unsigned char bus, unsigned char dev_fn, 
			       unsigned char offset, unsigned char val) {
	if (bus != 0 || (1<<PCI_SLOT(dev_fn) & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	out_8(pci->config_data + ((1<<PCI_SLOT(dev_fn))&~1) 
	      + (PCI_FUNC(dev_fn)<<8) + offset, 
	      val);
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_write_config_word(unsigned char bus, unsigned char dev_fn, 
			       unsigned char offset, unsigned short val) {
	if (offset&1) return PCIBIOS_BAD_REGISTER_NUMBER;
	if (bus != 0 || (1<<PCI_SLOT(dev_fn) & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	out_le16((volatile u_short *)
		 (pci->config_data + ((1<<PCI_SLOT(dev_fn))&~1)
		  + (PCI_FUNC(dev_fn)<<8) + offset),
		 val);
	return PCIBIOS_SUCCESSFUL;
}

static int
direct_pci_write_config_dword(unsigned char bus, unsigned char dev_fn, 
				unsigned char offset, unsigned int val) {
	if (offset&3) return PCIBIOS_BAD_REGISTER_NUMBER;
	if (bus != 0 || (1<<PCI_SLOT(dev_fn) & 0xff8007fe)) {
 		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	out_le32((volatile u_int *)
		 (pci->config_data + ((1<<PCI_SLOT(dev_fn))&~1)
		  + (PCI_FUNC(dev_fn)<<8) + offset),
		 val);
	return PCIBIOS_SUCCESSFUL;
}

static const struct pci_config_access_functions direct_functions = {
  	direct_pci_read_config_byte,
  	direct_pci_read_config_word,
  	direct_pci_read_config_dword,
  	direct_pci_write_config_byte,
  	direct_pci_write_config_word,
  	direct_pci_write_config_dword
};


void pci_read_bases(struct pci_dev *dev, unsigned int howmany)
{
	unsigned int reg, nextreg;
#define REG (PCI_BASE_ADDRESS_0 + (reg<<2))
        u_short cmd;
	u32 l, ml;
        pci_read_config_word(dev, PCI_COMMAND, &cmd);

	for(reg=0; reg<howmany; reg=nextreg) {
	  	pci_resource *r;
		nextreg=reg+1;
		pci_read_config_dword(dev, REG, &l);
#if 0
		if (l == 0xffffffff /*AJF || !l*/) continue; 
#endif
		/* Note that disabling the memory response of a host bridge 
		 * would lose data if a DMA transfer were in progress. In a 
		 * bootloader we don't care however. Also we can't print any 
		 * message for a while since we might just disable the console.
		 */
		pci_write_config_word(dev, PCI_COMMAND, cmd & 
				      ~(PCI_COMMAND_IO|PCI_COMMAND_MEMORY));
		pci_write_config_dword(dev, REG, ~0);
		pci_read_config_dword(dev, REG, &ml);
		pci_write_config_dword(dev, REG, l);

		/* Reenable the device now that we've played with 
		 * base registers. 
		 */
		pci_write_config_word(dev, PCI_COMMAND, cmd);

		/* seems to be an unused entry skip it */
		if ( ml == 0 || ml == 0xffffffff ) continue;

		if ((l & 
		     (PCI_BASE_ADDRESS_SPACE|PCI_BASE_ADDRESS_MEM_TYPE_MASK))
		    == (PCI_BASE_ADDRESS_MEM_TYPE_64
		    |PCI_BASE_ADDRESS_SPACE_MEMORY)) {
			nextreg=reg+2;
		}
		dev->base_address[reg] = l;
		r = salloc(sizeof(pci_resource));
		if (!r) {
		  	printk("Error allocating pci_resource struct.\n");
			continue;
		}
		r->dev = dev;
		r->reg = reg;
		if ((l&PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO) {
			r->type = l&~PCI_BASE_ADDRESS_IO_MASK;
			r->base = l&PCI_BASE_ADDRESS_IO_MASK;
			r->size = ~(ml&PCI_BASE_ADDRESS_IO_MASK)+1;
		} else {
			r->type = l&~PCI_BASE_ADDRESS_MEM_MASK;
			r->base = l&PCI_BASE_ADDRESS_MEM_MASK;
			r->size = ~(ml&PCI_BASE_ADDRESS_MEM_MASK)+1;
		}
		/* Check for the blacklisted entries */
		insert_resource(r);
	}
}




u_int pci_scan_bus(struct pci_bus *bus)
{
	unsigned int devfn, l, max, class;
	unsigned char irq, hdr_type, is_multi = 0;
	struct pci_dev *dev, **bus_last;
	struct pci_bus *child;

	bus_last = &bus->devices;
	max = bus->secondary;
	for (devfn = 0; devfn < 0xff; ++devfn) {
		if (PCI_FUNC(devfn) && !is_multi) {
			/* not a multi-function device */
			continue;
		}
		if (pcibios_read_config_byte(bus->number, devfn, PCI_HEADER_TYPE, &hdr_type))
			continue;
		if (!PCI_FUNC(devfn))
			is_multi = hdr_type & 0x80;

		if (pcibios_read_config_dword(bus->number, devfn, PCI_VENDOR_ID, &l) ||
		    /* some broken boards return 0 if a slot is empty: */
		    l == 0xffffffff || l == 0x00000000 || l == 0x0000ffff || l == 0xffff0000) {
			is_multi = 0;
			continue;
		}

		dev = salloc(sizeof(*dev));
		dev->bus = bus;
		dev->devfn  = devfn;
		dev->vendor = l & 0xffff;
		dev->device = (l >> 16) & 0xffff;

		pcibios_read_config_dword(bus->number, devfn, 
                                          PCI_CLASS_REVISION, &class);
		class >>= 8;				    /* upper 3 bytes */
		dev->class = class;
		class >>= 8;
		dev->hdr_type = hdr_type;

		switch (hdr_type & 0x7f) {		    /* header type */
		case PCI_HEADER_TYPE_NORMAL:		    /* standard header */
			if (class == PCI_CLASS_BRIDGE_PCI)
				goto bad;
			/*
			 * If the card generates interrupts, read IRQ number
			 * (some architectures change it during pcibios_fixup())
			 */
			pcibios_read_config_byte(bus->number, dev->devfn, PCI_INTERRUPT_PIN, &irq);
			if (irq)
				pcibios_read_config_byte(bus->number, dev->devfn, PCI_INTERRUPT_LINE, &irq);
			dev->irq = irq;
			/*
			 * read base address registers, again pcibios_fixup() can
			 * tweak these
			 */
			pci_read_bases(dev, 6);
			pcibios_read_config_dword(bus->number, devfn, PCI_ROM_ADDRESS, &l);
			dev->rom_address = (l == 0xffffffff) ? 0 : l;
			break;
		case PCI_HEADER_TYPE_BRIDGE:		    /* bridge header */
			if (class != PCI_CLASS_BRIDGE_PCI)
				goto bad;
			pci_read_bases(dev, 2);
			pcibios_read_config_dword(bus->number, devfn, PCI_ROM_ADDRESS1, &l);
			dev->rom_address = (l == 0xffffffff) ? 0 : l;
			break;
		case PCI_HEADER_TYPE_CARDBUS:		    /* CardBus bridge header */
			if (class != PCI_CLASS_BRIDGE_CARDBUS)
				goto bad;
			pci_read_bases(dev, 1);
			break;
		default:				    /* unknown header */
		bad:
			printk("PCI device with unknown "
			       "header type %d ignored.\n",
			       hdr_type&0x7f);
			continue;
		}

		/*
		 * Put it into the global PCI device chain. It's used to
		 * find devices once everything is set up.
		 */
		*pci->last_dev_p = dev;
		pci->last_dev_p = &dev->next;

		/*
		 * Now insert it into the list of devices held
		 * by the parent bus.
		 */
		*bus_last = dev;
		bus_last = &dev->sibling;

	}

	/*
	 * After performing arch-dependent fixup of the bus, look behind
	 * all PCI-to-PCI bridges on this bus.
	 */
	for(dev=bus->devices; dev; dev=dev->sibling)
		/*
		 * If it's a bridge, scan the bus behind it.
		 */
		if ((dev->class >> 8) == PCI_CLASS_BRIDGE_PCI) {
			unsigned int buses;
			unsigned int devfn = dev->devfn;
			unsigned short cr;

			/*
			 * Insert it into the tree of buses.
			 */
			child = salloc(sizeof(*child));
			child->next = bus->children;
			bus->children = child;
			child->self = dev;
			child->parent = bus;

			/*
			 * Set up the primary, secondary and subordinate
			 * bus numbers.
			 */
			child->number = child->secondary = ++max;
			child->primary = bus->secondary;
			child->subordinate = 0xff;
			/*
			 * Clear all status bits and turn off memory,
			 * I/O and master enables.
			 */
			pcibios_read_config_word(bus->number, devfn, PCI_COMMAND, &cr);
			pcibios_write_config_word(bus->number, devfn, PCI_COMMAND, 0x0000);
			pcibios_write_config_word(bus->number, devfn, PCI_STATUS, 0xffff);
			/*
			 * Read the existing primary/secondary/subordinate bus
			 * number configuration to determine if the PCI bridge
			 * has already been configured by the system.  If so,
			 * do not modify the configuration, merely note it.
			 */
			pcibios_read_config_dword(bus->number, devfn, PCI_PRIMARY_BUS, &buses);
			if ((buses & 0xFFFFFF) != 0)
			  {
			    unsigned int cmax;

			    child->primary = buses & 0xFF;
			    child->secondary = (buses >> 8) & 0xFF;
			    child->subordinate = (buses >> 16) & 0xFF;
			    child->number = child->secondary;
			    cmax = pci_scan_bus(child);
			    if (cmax > max) max = cmax;
			  }
			else
			  {
			    /*
			     * Configure the bus numbers for this bridge:
			     */
			    buses &= 0xff000000;
			    buses |=
			      (((unsigned int)(child->primary)     <<  0) |
			       ((unsigned int)(child->secondary)   <<  8) |
			       ((unsigned int)(child->subordinate) << 16));
			    pcibios_write_config_dword(bus->number, devfn, PCI_PRIMARY_BUS, buses);
			    /*
			     * Now we can scan all subordinate buses:
			     */
			    max = pci_scan_bus(child);
			    /*
			     * Set the subordinate bus number to its real
			     * value:
			     */
			    child->subordinate = max;
			    buses = (buses & 0xff00ffff)
			      | ((unsigned int)(child->subordinate) << 16);
			    pcibios_write_config_dword(bus->number, devfn, PCI_PRIMARY_BUS, buses);
			  }
			pcibios_write_config_word(bus->number, devfn, PCI_COMMAND, cr);
		}

	/*
	 * We've scanned the bus and so we know all about what's on
	 * the other side of any bridges that may be on this bus plus
	 * any devices.
	 *
	 * Return how far we've got finding sub-buses.
	 */
	return max;
}

void
pci_fixup(void) {
	struct pci_dev *p;
	struct pci_bus *bus;
	for (bus = &pci_root; bus; bus=bus->next) {
	
        	for (p=bus->devices; p; p=p->sibling) {
                }
        }
}

void pci_init(void) {
	PPC_DEVICE *hostbridge;

	if (pci->last_dev_p) {
		printk("Two or more calls to pci_init!\n");
		return;
	}
	pci->last_dev_p = &(bd->pci_devices);
#if 0
	hostbridge=residual_find_device(PROCESSORDEVICE, NULL, 
					BridgeController,
					PCIBridge, -1, 0);
	if (hostbridge) {
		if (hostbridge->DeviceId.Interface==PCIBridgeIndirect) {
			bd->pci_functions=&indirect_functions;
			/* Should be extracted from residual data, 
			 * indeed MPC106 in CHRP mode is different,
			 * but we should not use residual data in
			 * this case anyway. 
			 */
			pci->config_addr = ((volatile u_int *) 
					       (ptr_mem_map->io_base+0xcf8));
			pci->config_data = ptr_mem_map->io_base+0xcfc;
		} else if(hostbridge->DeviceId.Interface==PCIBridgeDirect) {
		  	bd->pci_functions=&direct_functions;
			pci->config_data=(u_char *) 0x80800000;
		} else {
		}
	} else {
		/* Let us try by experimentation at our own risk! */
		u_int id0;
	  	bd->pci_functions = &direct_functions;
		/* On all direct bridges I know the host bridge itself
		 * appears as device 0 function 0. 
		 */
		pcibios_read_config_dword(0, 0, PCI_VENDOR_ID, &id0);
		if (id0==~0U) {
			bd->pci_functions = &indirect_functions;
			pci->config_addr = ((volatile u_int *)
					       (ptr_mem_map->io_base+0xcf8));
			pci->config_data = ptr_mem_map->io_base+0xcfc;
		}
		/* Here we should check that the host bridge is actually
		 * present, but if it not, we are in such a desperate
		 * situation, that we probably can't even tell it.
		 */
	}
#else
	bd->pci_functions = &indirect_functions;
	pci->config_addr = (volatile u_int*)0xfec00000;
	pci->config_data = (volatile u_int*)0xfee00000;
#endif
	/* Now build a small database of all found PCI devices */
	printk("\nPCI: Probing PCI hardware\n");
	pci_root.subordinate=pci_scan_bus(&pci_root);
	print_pci_resources("Configurable PCI resources:\n");
#if 0
	reconfigure_pci();
	print_pci_resources("Allocated PCI resources:\n");
#endif
}

