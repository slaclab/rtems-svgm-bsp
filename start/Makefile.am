##
## $Id$
##

## use a prefix for sake of clarity.
## svgmstart contains both, the preloader and the
## shared start file
SVGM_PREFIX = svgm

AUTOMAKE_OPTIONS = foreign 1.4

VPATH = @srcdir@:@srcdir@/../../shared/start

PGM = $(ARCH)/$(SVGM_PREFIX)start.o

S_FILES = preload.S start.S
S_O_FILES = $(S_FILES:%.S=$(ARCH)/%.o)

OBJS = $(S_O_FILES)

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#
$(PGM):	$(OBJS)
	$(LD) -r -o $@ $^

$(PROJECT_RELEASE)/lib/$(SVGM_PREFIX)start$(LIB_VARIANT).o: $(PGM)
	$(INSTALL_DATA) $< $@

TMPINSTALL_FILES += $(PROJECT_RELEASE)/lib/$(SVGM_PREFIX)start$(LIB_VARIANT).o

all-local: $(ARCH) $(OBJS) $(PGM) $(TMPINSTALL_FILES)

.PRECIOUS: $(PGM)

include $(top_srcdir)/../../../../../../automake/local.am
