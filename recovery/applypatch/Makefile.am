AM_CFLAGS := -Wundef \
	    -Wstrict-prototypes \
	    -Wno-trigraphs \
	    -g -O0 \
	    -fno-inline \
	    -fno-short-enums \
	    -fpic

AM_CPPFLAGS := \
              -I. \
              -I$(WORKSPACE)/system/core/include

AM_LDFLAGS := -shared -version-info 1:0:0
c_sources := applypatch.c \
	    bspatch.c \
	    bsdiff.c \
	    imgdiff.c \
	    imgpatch.c \
	    freecache.c \
	    utils.c

h_sources := applypatch.h

library_includedir := $(pkgincludedir)
library_include_HEADERS := $(h_sources)
requiredlibs := -lm \
               -lpthread \
               -lz \
               -lbz2 \
               -lmincrypt \
               -lcutils \
               ../minelf/libminelf.la \
               ../mtdutils/libmtdutils.la \
               ../edify/libedify.la \
               ../minzip/libminzip.la

lib_LTLIBRARIES := libapplypatch.la

if USE_GLIB
libapplypatch_la_SOURCES := $(c_sources) $(h_sources)
libapplypatch_la_CFLAGS := $(AM_CFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
libapplypatch_la_CPPFLAGS := $(AM_CPPFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
libapplypatch_la_LIBADD := $(requiredlibs) @GLIB_LIBS@
libapplypatch_la_LDFLAGS := -all-static
else
libapplypatch_la_SOURCES := $(c_sources) $(h_sources)
libapplypatch_la_CFLAGS := $(AM_CFLAGS)
libapplypatch_la_CPPFLAGS := $(AM_CPPFLAGS)
libapplypatch_la_LIBADD := $(requiredlibs)
libapplypatch_la_LDFLAGS := -all-static
endif

applypatch_c_sources := applypatch.c \
	    bspatch.c \
	    bsdiff.c \
	    imgpatch.c \
	    freecache.c \
	    utils.c \
	    main.c

applypatch_h_sources := applypatch.h utils.h

bin_PROGRAMS := applypatch

applypatch_SOURCES := $(applypatch_c_sources) $(applypatch_h_sources)

if USE_GLIB
applypatch_CFLAGS := $(AM_CFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
applypatch_CPPFLAGS := $(AM_CPPFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
applypatch_LDADD := $(requiredlibs) @GLIB_LIBS@
applypatch_LDFLAGS := -all-static
else
applypatch_CFLAGS := $(AM_CFLAGS)
applypatch_CPPFLAGS := $(AM_CPPFLAGS)
applypatch_LDADD := $(requiredlibs)
applypatch_LDFLAGS := -all-static
endif
