##
## $Id$
##

include_HEADERS = bspopts.h

include_HEADERS += bsp.h ../../../shared/include/coverhd.h


H_FILES = bsp.h \
    ../../../shared/include/coverhd.h

$(PROJECT_INCLUDE):
	$(mkinstalldirs) $@

$(PROJECT_INCLUDE)/%.h: %.h
	$(INSTALL_DATA) $< $@

$(PROJECT_INCLUDE)/coverhd.h: ../../../shared/include/coverhd.h
	$(INSTALL_DATA) $< $@


PREINSTALL_FILES = $(PROJECT_INCLUDE)
PREINSTALL_FILES += $(PROJECT_INCLUDE)/bsp.h
PREINSTALL_FILES += $(PROJECT_INCLUDE)/bspopts.h
PREINSTALL_FILES += $(PROJECT_INCLUDE)/coverhd.h

all-local: $(PREINSTALL_FILES)

include $(top_srcdir)/../../../../../../automake/local.am
