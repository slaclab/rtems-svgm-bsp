##
## $Id$
##

ACLOCAL_AMFLAGS = -I ../../../../aclocal

SUBDIRS         = .
CLEANFILES      =
DISTCLEANFILES  =
noinst_PROGRAMS =
EXTRA_DIST      = 


include $(top_srcdir)/../../../../automake/compile.am
include $(top_srcdir)/../../bsp.am

include_bspdir = $(includedir)/bsp

dist_project_lib_DATA = bsp_specs
project_lib_DATA      = 

#include
include_HEADERS = include/bsp.h

nodist_include_HEADERS = include/bspopts.h
nodist_include_bsp_HEADERS = ../../shared/include/bootcard.h
DISTCLEANFILES += include/bspopts.h

include_bsp_HEADERS =

#start
EXTRA_DIST += ../../powerpc/shared/start/rtems_crti.S
rtems_crti.$(OBJEXT): ../../powerpc/shared/start/rtems_crti.S
	$(CPPASCOMPILE) -DASM -o $@ -c $<
project_lib_DATA += rtems_crti.$(OBJEXT)


EXTRA_DIST += ../../powerpc/shared/start/start.S
start.$(OBJEXT): ../../powerpc/shared/start/start.S
	$(CPPASCOMPILE) -DASM -o $@ -c $<

EXTRA_DIST += ../shared/start/preload.S
preload.$(OBJEXT): ../shared/start/preload.S
	$(CPPASCOMPILE) -DASM -o $@ -c $<

EXTRA_DIST += ../shared/start/vectors_entry.S
vectors_entry.$(OBJEXT): ../shared/start/vectors_entry.S
	$(CPPASCOMPILE) -DASM -o $@ -c $<

svgmstart.$(OBJEXT):	preload.$(OBJEXT) vectors_entry.$(OBJEXT) start.$(OBJEXT)
	$(LD) -o $@ -r $^

project_lib_DATA += svgmstart.$(OBJEXT)


#startup

dist_project_lib_DATA += ../shared/startup/linkcmds

build_date.c::
	echo 'char *BSP_build_date="'`date`'";' > $@

noinst_LIBRARIES = libbsp.a
libbsp_a_SOURCES =

libbsp_a_SOURCES += startup/bspstart.c \
    build_date.c startup/bspclean.c\
    ../../powerpc/shared/startup/panic.c \
	../../powerpc/shared/startup/bspgetworkarea.c \
    ../../powerpc/shared/startup/probeMemEnd.c \
    ../../powerpc/shared/startup/pretaskinghook.c \
    ../../powerpc/shared/startup/zerobss.c \
    ../../powerpc/shared/startup/pgtbl_setup.c \
    ../../powerpc/shared/startup/pgtbl_activate.c \
    ../../powerpc/shared/startup/sbrk.c ../../shared/bootcard.c \
    ../../shared/bsplibc.c ../../shared/bsppost.c \
    ../../shared/bsppredriverhook.c \
    ../../shared/gnatinstallhandler.c

#clock
libbsp_a_SOURCES += ../../powerpc/shared/clock/p_clock.c

#console
include_bsp_HEADERS += ../../powerpc/shared/console/consoleIo.h
include_bsp_HEADERS += ../../powerpc/shared/console/uart.h

libbsp_a_SOURCES +=  \
    ../../powerpc/shared/console/uart.c \
    ../../powerpc/shared/console/console.c \
	../../powerpc/shared/console/consoleIo.h \
	../../powerpc/shared/console/uart.h

#irq
include_bsp_HEADERS += irq/irq.h 

libbsp_a_SOURCES += irq/irq_init.c \
    ../../powerpc/shared/irq/openpic_i8259_irq.c

#openpic
include_bsp_HEADERS += ../../powerpc/shared/openpic/openpic.h

libbsp_a_SOURCES += \
    ../../powerpc/shared/openpic/openpic.c \
    ../../powerpc/shared/openpic/openpic.h

#pci
include_bsp_HEADERS += ../../powerpc/shared/pci/pci.h

libbsp_a_SOURCES += ../../powerpc/shared/pci/pci.c \
	../../powerpc/shared/pci/pci.h \
    pci/detect_grackle_bridge.c pci/iroutemap.c \
    ../../powerpc/shared/pci/pcifinddevice.c

#vectors
include_bsp_HEADERS += ./vectors/ssrl/bspException.h \
	../../../libcpu/@RTEMS_CPU@/@exceptions@/bspsupport/vectors.h \
	../../../libcpu/@RTEMS_CPU@/@exceptions@/bspsupport/irq_supp.h 

libbsp_a_SOURCES += \
    ./vectors/ssrl/exceptionhandler.c \
    ./vectors/machinecheckclear.c

#vme
include_bsp_HEADERS += vme/VMEConfig.h \
    ../../shared/vmeUniverse/vmeUniverse.h \
    ../../shared/vmeUniverse/vme_am_defs.h \
    ../../shared/vmeUniverse/VME.h\
	../../shared/vmeUniverse/vmeUniverseDMA.h\
	../../shared/vmeUniverse/bspVmeDmaList.h\
	../../shared/vmeUniverse/VMEDMA.h

libbsp_a_SOURCES += ../shared/vme/vmeconfig.c \
    ../shared/vme/vme_universe.c \
    ../../shared/vmeUniverse/vmeUniverse.c \
    ../../shared/vmeUniverse/bspVmeDmaList.c

#synergy
include_bsp_HEADERS += synergy/svgmRtc.h synergy/bspVGM.h

libbsp_a_SOURCES += synergy/board.c \
    synergy/led.c synergy/reboot.c \
    synergy/rtc.c synergy/watchdog.c \
    synergy/synergyregs.h \
	synergy/svgmRtc.h synergy/bspVGM.h

#network
if HAS_NETWORKING
noinst_PROGRAMS += network.rel
network_rel_SOURCES = network/yellowfin.c network/yf_rtemscompat.h
network_rel_CPPFLAGS = $(AM_CPPFLAGS)
network_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)
endif


libbsp_a_LIBADD = ../../../libcpu/@RTEMS_CPU@/shared/cpuIdent.rel \
    ../../../libcpu/@RTEMS_CPU@/shared/cache.rel \
    ../../../libcpu/@RTEMS_CPU@/shared/stack.rel \
    ../../../libcpu/@RTEMS_CPU@/@exceptions@/rtems-cpu.rel \
    ../../../libcpu/@RTEMS_CPU@/@exceptions@/exc_bspsupport.rel \
    ../../../libcpu/@RTEMS_CPU@/@exceptions@/irq_bspsupport.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/clock.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/mmu.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/timer.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/altivec.rel

if HAS_NETWORKING
libbsp_a_LIBADD += network.rel
endif

all-local: $(PREINSTALL_FILES) $(TMPINSTALL_FILES)

EXTRA_DIST += README ChangeLog README.booting TODO

include $(srcdir)/preinstall.am
include $(top_srcdir)/../../../../automake/local.am
