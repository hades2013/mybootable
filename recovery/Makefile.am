ACLOCAL_AMFLAGS := -I m4

SUBDIRS := minelf mtdutils edify minzip applypatch updater

AM_CFLAGS := -Wundef \
	    -Wstrict-prototypes \
	    -Wno-trigraphs \
	    -g -O0 \
	    -fno-inline \
	    -fno-short-enums \
	    -fpic \
	    -D__reboot=reboot \
	    -Dstrlcat=strncat

AM_CPPFLAGS := \
              -I. \
              -I$(WORKSPACE)/system/core/include

c_sources := install.c \
	    recovery.c \
	    bootloader.c \
	    roots.c \
	    verifier.c \
	    ui.c

requiredlibs := -lm \
               -lpthread \
               -lz \
               -lcutils \
               -lmincrypt \
               ./minelf/libminelf.la \
               ./mtdutils/libmtdutils.la \
               ./edify/libedify.la \
               ./minzip/libminzip.la \
               ./applypatch/libapplypatch.la


recovery_SOURCES := $(c_sources)
recovery_CFLAGS := $(AM_CFLAGS)
recovery_CPPFLAGS := $(AM_CPPFLAGS)
recovery_LDADD := $(requiredlibs)


verifier_test_libs := -lmincrypt -lm -lpthread -lcutils
verifier_test_CFLAGS := $(AM_CFLAGS)

verifier_test_CPPFLAGS := $(AM_CPPFLAGS)
verifier_test_LDADD := $(verifier_test_libs)

verifier_test_SOURCES := verifier_test.c verifier.c

bin_PROGRAMS := verifier_test recovery

pkgconfigdir := $(libdir)/pkgconfig
pkgconfig_DATA := recovery.pc
EXTRA_DIST := $(pkgconfig_DATA)
