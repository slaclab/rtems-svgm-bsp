##
## $Id$
##

## use a prefix for sake of clarity.
## svgmstart contains both, the preloader and the
## shared start file
SVGM_PREFIX = svgm

VPATH = @srcdir@:@srcdir@/../../shared/start

PGM = $(ARCH)/$(SVGM_PREFIX)start.$(OBJEXT)

S_FILES = preload.S start.S rtems_crti.S
S_O_FILES = $(S_FILES:%.S=$(ARCH)/%.$(OBJEXT))

#EXTRA_DIST = preload.S

OBJS = $(ARCH)/preload.$(OBJEXT) $(ARCH)/start.$(OBJEXT)

include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#
bsplib_DATA =  $(PROJECT_RELEASE)/lib/$(SVGM_PREFIX)start$(LIB_VARIANT).$(OBJEXT)
bsplib_DATA += $(PROJECT_RELEASE)/lib/rtems_crti.$(OBJEXT)

$(PROJECT_RELEASE)/lib/$(SVGM_PREFIX)start$(LIB_VARIANT).$(OBJEXT): $(PGM)
	$(INSTALL_DATA) $< $@

$(PROJECT_RELEASE)/lib/rtems_crti.$(OBJEXT): $(ARCH)/rtems_crti.$(OBJEXT)
	$(INSTALL_DATA) $< $@

TMPINSTALL_FILES += $(PROJECT_RELEASE)/lib/$(SVGM_PREFIX)start$(LIB_VARIANT).$(OBJEXT)
TMPINSTALL_FILES += $(PROJECT_RELEASE)/lib/rtems_crti.$(OBJEXT)

all-local: $(ARCH) $(S_O_FILES) $(PGM) $(TMPINSTALL_FILES)

$(OBJS):$(ARCH)

$(PGM):	$(OBJS)
	$(LD) -r -o $@ $^

include $(top_srcdir)/../../../../../../automake/local.am
