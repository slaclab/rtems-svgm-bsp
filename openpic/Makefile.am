##
## Makefile.am,v 1.7 2002/12/17 15:10:31 ralf Exp
##


VPATH = @srcdir@:@srcdir@/../../shared/openpic

C_FILES = openpic.c

include_bspdir = $(includedir)/bsp
include_bsp_HEADERS = ../../shared/openpic/openpic.h

H_FILES = ../../shared/openpic/openpic.h

OBJS = $(C_FILES:%.c=$(ARCH)/%.$(OBJEXT))

include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $< $@

$(PROJECT_INCLUDE)/bsp/openpic.h: ../../shared/openpic/openpic.h
	$(INSTALL_DATA) $< $@

#
# (OPTIONAL) Add local stuff here using +=
#

PREINSTALL_FILES = $(PROJECT_INCLUDE)/bsp $(PROJECT_INCLUDE)/bsp/openpic.h

all-local: $(ARCH) $(PREINSTALL_FILES) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
