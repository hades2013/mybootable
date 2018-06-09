AM_CFLAGS := -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        -g -O0 \
        -fno-inline \
        -fno-short-enums \
        -fpic

AM_CPPFLAGS := \
              -I.

AM_LDFLAGS := -shared -version-info 1:0:0
c_sources := Retouch.c
h_sources := Retouch.h

library_includedir := $(pkgincludedir)
library_include_HEADERS := $(h_sources)

lib_LTLIBRARIES := libminelf.la
if USE_GLIB
libminelf_la_SOURCES := $(c_sources) $(h_sources)
libminelf_la_CFLAGS := $(AM_CFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
libminelf_la_CPPFLAGS := $(AM_CPPFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
libminelf_la_LIBADD := -lm -lpthread @GLIB_LIBS@
libminelf_la_LDFLAGS := -all-static
else
libminelf_la_SOURCES := $(c_sources) $(h_sources)
libminelf_la_CFLAGS := $(AM_CFLAGS)
libminelf_la_CPPFLAGS := $(AM_CPPFLAGS)
libminelf_la_LIBADD := -lm -lpthread
libminelf_la_LDFLAGS := -all-static
endif
