##
## $Id$
##

AUTOMAKE_OPTIONS = foreign 1.4

VPATH = @srcdir@:@srcdir@/../../shared/pci

C_FILES = pci.c pciUtils.c
C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.o)

H_FILES = ../../shared/pci/pci.h pci.h pciExt.h

OBJS = $(C_O_FILES)

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#

AM_CFLAGS = -O2

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $<

$(PROJECT_INCLUDE)/bsp/pciBase.h: ../../shared/pci/pci.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp/pciExt.h: pciExt.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp/pci.h: pci.h
	$(INSTALL_DATA) $< $@

# the .rel file built here will be put into libbsp.a by ../wrapup/Makefile
PREINSTALL_FILES += $(PROJECT_INCLUDE)/bsp $(PROJECT_INCLUDE)/bsp/pciBase.h $(PROJECT_INCLUDE)/bsp/pciExt.h $(PROJECT_INCLUDE)/bsp/pci.h

all-local: $(ARCH) $(PREINSTALL_FILES) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
