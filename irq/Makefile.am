##
## $Id$
##

VPATH = @srcdir@:@srcdir@/../../shared/irq

C_FILES = irq_init.c irq.c openpic_i8259_irq.c
C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.$(OBJEXT))


include_bspdir = $(includedir)/bsp
include_bsp_HEADERS = irq.h ../../shared/irq/bsp_common_irq.h

H_FILES = irq.h

S_FILES = irq_asm.S
S_O_FILES = $(S_FILES:%.S=$(ARCH)/%.$(OBJEXT))

EXTRA_DIST = irq_init.c

OBJS = $(S_O_FILES) $(C_O_FILES)

include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/bsp/%.h: %.h
	$(INSTALL_DATA) $< $@

PREINSTALL_FILES = $(PROJECT_INCLUDE)/bsp/irq.h $(PROJECT_INCLUDE)/bsp/bsp_common_irq.h

all-local: $(ARCH) $(PREINSTALL_FILES) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
