##
## $Id$
##

VPATH = @srcdir@:@srcdir@/../../shared/vme:@srcdir@/../../../shared/vmeUniverse:

H_FILES = ../pci/pci.h ../irq/irq.h ../vme/VME.h ../vme/VMEConfig.h

C_FILES = vmeUniverse.c vmeconfig.c vme_universe.c


C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.$(OBJEXT))

OBJS = $(C_O_FILES)

include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

include_bspdir = $(includedir)/bsp
include_bsp_HEADERS = VMEConfig.h
include_bsp_HEADERS += ../../shared/vme/VME.h
include_bsp_HEADERS += ../../../shared/vmeUniverse/vmeUniverse.h
include_bsp_HEADERS += ../../../shared/vmeUniverse/vme_am_defs.h

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/bsp/%.h: %.h
	$(INSTALL_DATA) $< $@

PREINSTALL_FILES = $(PROJECT_INCLUDE)/bsp \
    $(addprefix $(PROJECT_INCLUDE)/bsp/,$(notdir $(include_bsp_HEADERS)))


all-local: $(PREINSTALL_FILES) $(ARCH) $(OBJS)

#
# (OPTIONAL) Add local stuff here using +=
#
EXTRA_DIST = vmeconfig.c

include $(top_srcdir)/../../../../../../automake/local.am
