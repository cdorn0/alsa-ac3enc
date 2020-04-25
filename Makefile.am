ACLOCAL_AMFLAGS = -I m4
AUTOMAKE_OPTIONS = foreign

CONF_FILES = ac3.conf

EXTRA_DIST = $(CONF_FILES) pa_mapping.conf

asound_module_pcm_ac3_LTLIBRARIES = libasound_module_pcm_ac3.la
asound_module_pcm_conf_DATA = $(CONF_FILES)

asound_module_pcm_ac3dir = @ALSA_PLUGIN_DIR@
asound_module_pcm_confdir = @ALSA_DATA_DIR@/pcm

AM_CFLAGS = -Wall -g @ALSA_CFLAGS@
AM_LDFLAGS = -module -avoid-version -export-dynamic -no-undefined $(LDFLAGS_NOUNDEFINED)

libasound_module_pcm_ac3_la_SOURCES = pcm_ac3.c
libasound_module_pcm_ac3_la_LIBADD = @ALSA_LIBS@ -laften

install-conf-hook:
	mkdir -p $(DESTDIR)$(ALSA_LCONF_DIR)
	@(echo cd $(DESTDIR)$(ALSA_LCONF_DIR);			\
	  cd $(DESTDIR)$(ALSA_LCONF_DIR);			\
	  for i in $(CONF_FILES); do				\
	    echo $(RM) $$i";" ln -s $(ALSA_DATA_DIR)/pcm/$$i .;	\
	    $(RM) $$i;						\
	    ln -s $(ALSA_DATA_DIR)/pcm/$$i .;			\
	  done)
	echo $(DESTDIR)
	echo $(ALSA_DATA_DIR)
	mkdir -p $(DESTDIR)$(PA_DATA_DIR)/alsa-mixer/profile-sets
	cat $(SRCDIR)pa_mapping.conf >> $(DESTDIR)$(PA_DATA_DIR)/alsa-mixer/profile-sets/default.conf

uninstall-conf-hook:
	@(echo cd $(DESTDIR)$(ALSA_LCONF_DIR);			\
	  cd $(DESTDIR)$(ALSA_LCONF_DIR);			\
	  for i in $(CONF_FILES); do				\
	    echo $(RM) $$i;					\
	    $(RM) $$i;						\
	  done)


install-data-hook: install-conf-hook

uninstall-local: uninstall-conf-hook


dist-hook:
	-chmod -R a+r $(distdir)
	@if ! test -z "$(AMTAR)"; then \
		$(AMTAR) --create --verbose --file=- $(distdir) | bzip2 -c -9 > $(distdir).tar.bz2 ; \
	else \
		$(TAR) --create --verbose --file=- $(distdir) | bzip2 -c -9 > $(distdir).tar.bz2 ; \
	fi
