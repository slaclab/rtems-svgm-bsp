#include <libcpu/io.h>

#ifndef SVGM_SERIAL_BASE
#define SVGM_SERIAL_BASE 0xffeffb08
#endif

#define COM1 0x3f8


/* dirty hack around inb/outb */
static inline void eieio(void)
{
	__asm__ __volatile__("eieio");
}

#undef inb
#undef outb

static inline unsigned char
inb(unsigned long port)
{
	if (COM1 <= port && port < COM1+8) {
		eieio();
		return *(volatile unsigned char *)(SVGM_SERIAL_BASE + port - COM1);
	}
	/* make sure kbd detection fails and returns */
	return (unsigned char) -1;
}

static inline void
outb(unsigned char val, unsigned long port)
{
	if (COM1 <= port && port < COM1+8) {
		eieio();
		*(volatile unsigned char *)(SVGM_SERIAL_BASE + port - COM1) = val;
	} /* else do nothing */
}

#include "../../shared/console/polled_io.c"
