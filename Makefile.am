##
## $Id$
##

AUTOMAKE_OPTIONS = foreign 1.4
ACLOCAL_AMFLAGS = -I ../../../../../../aclocal

# wrapup is the one that actually builds and installs the library
#  from the individual .rel files built in other directories
# NOTE: svgm does _not_ use the bootloader subdir; it's left
#       here for the moment, however...
SUBDIRS = include clock console pci openpic irq vectors start \
    startup network @exceptions@ wrapup

include $(top_srcdir)/../../bsp.am

EXTRA_DIST = README.booting bsp_specs times.svgm5

include $(top_srcdir)/../../../../../../automake/subdirs.am
include $(top_srcdir)/../../../../../../automake/local.am
