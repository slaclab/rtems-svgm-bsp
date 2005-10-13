#include <rtems.h>
#include <bsp/pci.h>
#include <bsp/irq.h>

#define NULL_INTMAP {-1,-1,-1,{}}
#define NULL_PINMAP {-1,{-1,-1,-1,-1}}

static unsigned char pci_intpins[4][4] =
{
	{ 1, 2, 3, 4 },  /* Buses 0, 4, 8, ... */
	{ 2, 3, 4, 1 },  /* Buses 1, 5, 9, ... */
	{ 3, 4, 1, 2 },  /* Buses 2, 6, 10 ... */
	{ 4, 1, 2, 3 },  /* Buses 3, 7, 11 ... */
};

static int pci_swizzle(int slot, int pin)
{
   return pci_intpins[ slot % 4 ][ pin-1 ];
}

#define OPTS	PCI_FIXUP_OPT_OVERRIDE_NAME

static struct _int_map iroutemap[] = {
	/* bus, slot, opts, route { pin, names[4] } */
	/* SCSI/Ethernet */
	{    0,   12, OPTS,  { { 1, { BSP_PCI_IRQ0 + 11 , -1, -1, -1 } },
	                       { 2, { BSP_PCI_IRQ0 + 10 , -1, -1, -1 } },
                           NULL_PINMAP } },
	/* PMC I */
	{    0,   13, OPTS,  { { 1, { BSP_PCI_IRQ0 + 3 , -1, -1, -1 } },
	                       { 2, { BSP_PCI_IRQ0 + 2 , -1, -1, -1 } },
	                       { 3, { BSP_PCI_IRQ0 + 9 , -1, -1, -1 } },
	                       { 4, { BSP_PCI_IRQ0 + 0 , -1, -1, -1 } },
                           NULL_PINMAP } },
	/* PMC II */
	{    0,   14, OPTS,  { { 4, { BSP_PCI_IRQ0 + 3 , -1, -1, -1 } }, 
	                       { 1, { BSP_PCI_IRQ0 + 2 , -1, -1, -1 } }, 
	                       { 2, { BSP_PCI_IRQ0 + 9 , -1, -1, -1 } }, 
	                       { 3, { BSP_PCI_IRQ0 + 0 , -1, -1, -1 } }, 
                           NULL_PINMAP } },
	/* Universe */
	{    0,   17, OPTS,  { { 1, { BSP_PCI_IRQ0 + 3 , BSP_PCI_IRQ0 + 2, BSP_PCI_IRQ0 + 9, BSP_PCI_IRQ0 + 0 } },
                           NULL_PINMAP } },
	NULL_INTMAP
};

void
_BSP_pciIRouteFixup()
{
	FixupPCI(iroutemap, pci_swizzle);
}
