##
## $Id$
##

## use a prefix for sake of clarity.
## svgmstart contains both, the preloader and the
## shared start file
SVGM_PREFIX = svgm

VPATH = @srcdir@:@srcdir@/../../shared/start

PGM = $(ARCH)/$(SVGM_PREFIX)start.$(OBJEXT)

S_FILES = preload.S start.S
S_O_FILES = $(S_FILES:%.S=$(ARCH)/%.$(OBJEXT))

EXTRA_DIST = preload.S

OBJS = $(S_O_FILES)

include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#
bsplib_DATA = $(PROJECT_RELEASE)/lib/$(SVGM_PREFIX)start$(LIB_VARIANT).$(OBJEXT)

$(PROJECT_RELEASE)/lib/$(SVGM_PREFIX)start$(LIB_VARIANT).$(OBJEXT): $(PGM)
	$(INSTALL_DATA) $< $@

TMPINSTALL_FILES += $(PROJECT_RELEASE)/lib/$(SVGM_PREFIX)start$(LIB_VARIANT).$(OBJEXT)

all-local: $(ARCH) $(OBJS) $(PGM) $(TMPINSTALL_FILES)

$(OBJS):$(ARCH)

$(PGM):	$(OBJS)
	$(LD) -r -o $@ $^

include $(top_srcdir)/../../../../../../automake/local.am
