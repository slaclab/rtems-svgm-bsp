##
## $Id$
##

AUTOMAKE_OPTIONS = foreign 1.4

VPATH = @srcdir@:

C_FILES = yellowfin.c
C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.o)

H_FILES = yf_rtemscompat.h

OBJS = $(C_O_FILES)

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#

AM_CPPFLAGS += -DSTATIC_LOG_ALLOC

all-local: $(ARCH) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
