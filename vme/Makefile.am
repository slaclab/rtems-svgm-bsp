##
## $Id$
##

AUTOMAKE_OPTIONS = foreign 1.4

VPATH = @srcdir@:@srcdir@/../../shared/vme:@srcdir@/../../../shared/vmeUniverse:

H_FILES = ../pci/pci.h ../irq/irq.h ../vme/bspVME.h ../vme/bspVMEConfig.h

C_FILES = vmeUniverse.c vmeconfig.c
C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.o)

OBJS = $(C_O_FILES)

$(PROJECT_INCLUDE):
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/bsp/vmeUniverse.h: vmeUniverse.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp/VME.h: bspVME.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp/VMEConfig.h: bspVMEConfig.h
	$(INSTALL_DATA) $< $@

all-local: $(TMPINSTALL_FILES) $(ARCH) $(OBJS)

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../../../../../../automake/force-preinstall.am
include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#

PREINSTALL_FILES += $(PROJECT_INCLUDE) $(PROJECT_INCLUDE)/bsp \
    $(PROJECT_INCLUDE)/bsp/vmeUniverse.h $(PROJECT_INCLUDE)/bsp/VME.h \
	$(PROJECT_INCLUDE)/bsp/VMEConfig.h

EXTRA_DIST = vmeconfig.c

all-local: $(ARCH) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
