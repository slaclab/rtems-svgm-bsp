##
## $Id$
##

AUTOMAKE_OPTIONS = foreign 1.4

VPATH = @srcdir@:@srcdir@/../console:@srcdir@/../../shared/vectors

C_FILES = vectors_init.c exceptionhandler.c
C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.o)

H_FILES = ../../shared/vectors/vectors.h bspException.h

S_FILES = vectors.S
S_O_FILES = $(S_FILES:%.S=$(ARCH)/%.o)

OBJS = $(S_O_FILES) $(C_O_FILES)

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#

EXTRA_DIST = bspException.h exceptionhandler.c

$(PGM): $(OBJS)
	$(make-rel)

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/bsp/vectors.h: ../../shared/vectors/vectors.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp/bspException.h: bspException.h
	$(INSTALL_DATA) $< $@

PREINSTALL_FILES = $(PROJECT_INCLUDE)/bsp $(PROJECT_INCLUDE)/bsp/vectors.h $(PROJECT_INCLUDE)/bsp/bspException.h

all-local: $(ARCH) $(PREINSTALL_FILES) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
