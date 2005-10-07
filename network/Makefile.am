##
## $Id$
##

VPATH = @srcdir@:@srcdir@/../../shared/network

C_FILES = yellowfin.c fixup_net_config.c
C_O_FILES = $(C_FILES:%.c=$(ARCH)/%.$(OBJEXT))

EXTRA_DIST = yellowfin.c

H_FILES = yf_rtemscompat.h

OBJS = $(C_O_FILES)

include $(top_srcdir)/../../../../../../automake/compile.am
include $(top_srcdir)/../../../../../../automake/lib.am

#
# (OPTIONAL) Add local stuff here using +=
#

AM_CPPFLAGS += -DSTATIC_LOG_ALLOC

all-local: $(ARCH) $(OBJS)

include $(top_srcdir)/../../../../../../automake/local.am
