##
## $Id$
##

VPATH = @srcdir@:@srcdir@/../console:@srcdir@/../../shared/startup:@srcdir@/../../../shared

INCLUDES = -I @srcdir@/../synergy -I @srcdir@/../../shared/startup

C_FILES = bootcard.c main.c bspstart.c bsppost.c bsplibc.c sbrk.c bspclean.c \
    gnatinstallhandler.c build_date.c pgtbl_setup.c pgtbl_activate.c panic.c
C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.$(OBJEXT))

build_date.c::
	echo 'char *BSP_build_date="'`date`'";' > $@

OBJS = $(C_O_FILES)

include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#
bsplib_DATA = linkcmds

$(PROJECT_RELEASE)/lib/linkcmds: linkcmds
	$(INSTALL_DATA) $< $@

TMPINSTALL_FILES += $(PROJECT_RELEASE)/lib/linkcmds

all-local: $(ARCH) $(OBJS) $(TMPINSTALL_FILES) 

EXTRA_DIST = linkcmds bspstart.c bspclean.c

include $(top_srcdir)/../../../../../../automake/local.am
