##
## $Id$
##

AUTOMAKE_OPTIONS = foreign 1.4

H_FILES = ../../shared/include/nvram.h ../include/bsp.h \
    ../../../shared/include/coverhd.h ../include/residual.h

BSP_H_FILES = ../../shared/console/consoleIo.h ../../shared/console/uart.h \
    ../irq/irq.h

$(PROJECT_INCLUDE):
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/bsp:
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/nvram.h: ../../shared/include/nvram.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp.h: ../include/bsp.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/coverhd.h: ../../../shared/include/coverhd.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp/consoleIo.h: ../../shared/console/consoleIo.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp/uart.h: ../../shared/console/uart.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp/irq.h: ../irq/irq.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/bsp/residual.h: ../include/residual.h
	$(INSTALL_DATA) $< $@


PREINSTALL_FILES += $(PROJECT_INCLUDE) $(PROJECT_INCLUDE)/bsp \
    $(PROJECT_INCLUDE)/nvram.h $(PROJECT_INCLUDE)/bsp.h \
    $(PROJECT_INCLUDE)/coverhd.h $(PROJECT_INCLUDE)/bsp/consoleIo.h \
    $(PROJECT_INCLUDE)/bsp/uart.h $(PROJECT_INCLUDE)/bsp/residual.h\
     $(PROJECT_INCLUDE)/bsp/irq.h

all-local: $(PREINSTALL_FILES)

include $(top_srcdir)/../../../../../../automake/local.am
