##
## $Id$
##

AUTOMAKE_OPTIONS = foreign 1.4

VPATH = @srcdir@:

INCLUDES = -I @srcdir@/../synergy

C_FILES = board.c  led.c  reboot.c  watchdog.c

C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.o)

H_FILES = ../synergy/bspVGM.h ../synergy/synergyregs.h

EXTRA_DIST = board.c  led.c  reboot.c  watchdog.c


OBJS = $(C_O_FILES)

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#

AM_CFLAGS += -O2

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $<

$(PROJECT_INCLUDE)/bsp/bspVGM.h: bspVGM.h
	$(INSTALL_DATA) $< $@

# the .rel file built here will be put into libbsp.a by ../wrapup/Makefile
PREINSTALL_FILES = $(PROJECT_INCLUDE)/bsp $(PROJECT_INCLUDE)/bsp/bspVGM.h

all-local: $(ARCH) $(PREINSTALL_FILES) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
