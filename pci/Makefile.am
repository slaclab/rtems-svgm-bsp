##
## $Id$
##

VPATH = @srcdir@:@srcdir@/../../shared/pci

C_FILES = pci.c detect_grackle_bridge.c pcifinddevice.c iroutemap.c
C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.$(OBJEXT))

include_bspdir = $(includedir)/bsp
include_bsp_HEADERS = ../../shared/pci/pci.h

H_FILES = ../../shared/pci/pci.h

EXTRA_DIST = detect_grackle_bridge.c

OBJS = $(C_O_FILES)

include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#

AM_CFLAGS += -O2

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $<

$(PROJECT_INCLUDE)/bsp/pci.h: ../../shared/pci/pci.h
	$(INSTALL_DATA) $< $@

# the .rel file built here will be put into libbsp.a by ../wrapup/Makefile
PREINSTALL_FILES = $(PROJECT_INCLUDE)/bsp $(PROJECT_INCLUDE)/bsp/pci.h

all-local: $(ARCH) $(PREINSTALL_FILES) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
