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

#prevent the compiler from generating FP instructions
AM_CFLAGS      += -msoft-float

dist_project_lib_DATA = bsp_specs
project_lib_DATA      = 

#include
include_HEADERS = include/bsp.h

nodist_include_HEADERS = include/bspopts.h
DISTCLEANFILES += include/bspopts.h

include_bspdir = $(includedir)/bsp
include_bsp_HEADERS =

#start
EXTRA_DIST += ../../powerpc/shared/start/rtems_crti.S
rtems_crti.$(OBJEXT): ../../powerpc/shared/start/rtems_crti.S
	$(CPPASCOMPILE) -DASM -o $@ -c $<
project_lib_DATA += rtems_crti.$(OBJEXT)


EXTRA_DIST += ../../powerpc/shared/start/start.S
start.$(OBJEXT): ../../powerpc/shared/start/start.S
	$(CPPASCOMPILE) -DASM -o $@ -c $<
EXTRA_DIST += ./start/ssrl/preload.S
preload.$(OBJEXT): ./start/ssrl/preload.S
	$(CPPASCOMPILE) -DASM -o $@ -c $<

svgmstart.$(OBJEXT):	preload.$(OBJEXT) start.$(OBJEXT)
	$(LD) -o $@ -r $^

project_lib_DATA += svgmstart.$(OBJEXT)


#startup

dist_project_lib_DATA += ../shared/startup/linkcmds

build_date.c::
	echo 'char *BSP_build_date="'`date`'";' > $@

noinst_PROGRAMS += startup.rel
startup_rel_SOURCES = startup/bspstart.c \
    build_date.c startup/bspclean.c\
    ../../powerpc/shared/startup/panic.c \
    ../../powerpc/shared/startup/probeMemEnd.c \
    ../../powerpc/shared/startup/pretaskinghook.c \
    ../../powerpc/shared/startup/zerobss.c \
    ../../powerpc/shared/startup/pgtbl_setup.c \
    ../../powerpc/shared/startup/pgtbl_activate.c \
    ../../powerpc/shared/startup/sbrk.c ../../shared/bootcard.c \
    ../../shared/bsplibc.c ../../shared/bsppost.c \
    ../../shared/main.c ../../shared/gnatinstallhandler.c
startup_rel_CPPFLAGS = $(AM_CPPFLAGS)
startup_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

#clock
noinst_PROGRAMS += pclock.rel
pclock_rel_SOURCES = ../../powerpc/shared/clock/p_clock.c
pclock_rel_CPPFLAGS = $(AM_CPPFLAGS)
pclock_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

#console
include_bsp_HEADERS += ../../powerpc/shared/console/consoleIo.h
include_bsp_HEADERS += ../../powerpc/shared/console/uart.h

console_CPPFLAGS = -DSTATIC_LOG_ALLOC
noinst_PROGRAMS += console.rel
console_rel_SOURCES =  \
    ../../powerpc/shared/console/uart.c \
    ../../powerpc/shared/console/console.c

console_rel_CPPFLAGS = $(AM_CPPFLAGS) $(console_CPPFLAGS)
console_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

#irq
include_bsp_HEADERS += irq/irq.h

noinst_PROGRAMS += irq.rel
irq_rel_SOURCES = irq/irq_init.c \
    ../../powerpc/shared/irq/openpic_i8259_irq.c \
    ../../powerpc/shared/irq/irq.c \
    ../../powerpc/shared/irq/irq_asm.S
irq_rel_CPPFLAGS = $(AM_CPPFLAGS)
irq_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

#openpic
include_bsp_HEADERS += ../../powerpc/shared/openpic/openpic.h

noinst_PROGRAMS += openpic.rel
openpic_rel_SOURCES = ../../powerpc/shared/openpic/openpic.h \
    ../../powerpc/shared/openpic/openpic.c \
    ../../powerpc/shared/openpic/openpic.h

openpic_rel_CPPFLAGS = $(AM_CPPFLAGS)
openpic_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

#pci
include_bsp_HEADERS += ../../powerpc/shared/pci/pci.h

noinst_PROGRAMS += pci.rel
pci_rel_SOURCES = ../../powerpc/shared/pci/pci.c \
    pci/detect_grackle_bridge.c pci/iroutemap.c \
    ../../powerpc/shared/pci/pcifinddevice.c
pci_rel_CPPFLAGS = $(AM_CPPFLAGS)
pci_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

#vectors
include_bsp_HEADERS += ../../powerpc/shared/vectors/vectors.h
include_bsp_HEADERS += ./vectors/ssrl/bspException.h

noinst_PROGRAMS += vectors.rel
vectors_rel_SOURCES = \
    ../../powerpc/shared/vectors/vectors_init.c \
    ../../powerpc/shared/vectors/vectors.S \
    ./vectors/ssrl/exceptionhandler.c \
    ./vectors/machinecheckclear.c
vectors_rel_CPPFLAGS = $(AM_CPPFLAGS)
vectors_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

#vme
include_bsp_HEADERS += vme/VMEConfig.h \
    ../../shared/vmeUniverse/vmeUniverse.h \
    ../../shared/vmeUniverse/vme_am_defs.h \
    ../../shared/vmeUniverse/VME.h\
	../../shared/vmeUniverse/vmeUniverseDMA.h\
	../../shared/vmeUniverse/bspVmeDmaList.h\
	../../shared/vmeUniverse/VMEDMA.h

noinst_PROGRAMS += vme.rel
vme_rel_SOURCES = ../shared/vme/vmeconfig.c \
    ../shared/vme/vme_universe.c \
	../shared/vme/vme_universe_dma.c \
    ../../shared/vmeUniverse/vmeUniverse.c \
    ../../shared/vmeUniverse/bspVmeDmaList.c
vme_rel_CPPFLAGS = $(AM_CPPFLAGS)
vme_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

#synergy
include_bsp_HEADERS += synergy/svgmRtc.h synergy/bspVGM.h

noinst_PROGRAMS += synergy.rel
synergy_rel_SOURCES = synergy/board.c \
    synergy/led.c synergy/reboot.c \
    synergy/rtc.c synergy/watchdog.c \
    synergy/synergyregs.h

synergy_rel_CPPFLAGS = $(AM_CPPFLAGS)
synergy_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)
#network

noinst_PROGRAMS += network.rel
network_rel_SOURCES = network/yellowfin.c network/yf_rtemscompat.h
network_rel_CPPFLAGS = $(AM_CPPFLAGS)
network_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)


noinst_LIBRARIES = libbsp.a
libbsp_a_SOURCES =
libbsp_a_LIBADD = startup.rel pclock.rel console.rel irq.rel \
    openpic.rel pci.rel vectors.rel vme.rel synergy.rel network.rel
libbsp_a_LIBADD += ../../../libcpu/@RTEMS_CPU@/shared/cpuIdent.rel \
    ../../../libcpu/@RTEMS_CPU@/shared/stack.rel \
    ../../../libcpu/@RTEMS_CPU@/@exceptions@/rtems-cpu.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/clock.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/exceptions.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/mmu.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/timer.rel

all-local: $(PREINSTALL_FILES) $(TMPINSTALL_FILES)

EXTRA_DIST +=

noinst_DATA = preinstall-stamp
preinstall-stamp:
	$(MAKE) $(AM_MAKEFLAGS) preinstall
	touch preinstall-stamp
CLEANFILES += preinstall-stamp

include $(srcdir)/preinstall.am

include $(top_srcdir)/../../../../automake/local.am
