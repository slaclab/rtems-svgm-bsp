##
## $Id$
##

AUTOMAKE_OPTIONS = foreign 1.4

VPATH = @srcdir@/../../../shared/vmeUniverse:

H_FILES = ../pci/pci.h ../irq/irq.h

C_FILES = vmeUniverse.c
C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.o)

OBJS = $(C_O_FILES)

$(PROJECT_INCLUDE):
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/bsp/vmeUniverse.h: vmeUniverse.h
	$(INSTALL_DATA) $< $@

all-local: $(TMPINSTALL_FILES) $(ARCH) $(OBJS)

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../../../../../../automake/force-preinstall.am
include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#

TMPINSTALL_FILES += $(PROJECT_INCLUDE) $(PROJECT_INCLUDE)/bsp \
    $(PROJECT_INCLUDE)/bsp/vmeUniverse.h

all-local: $(ARCH) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
