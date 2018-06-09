ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS := \
              -I. \
              -I$(WORKSPACE)/system/core/include

c_sources := fs_config.c
bin_PROGRAMS := fs_config

fsconfig_SOURCES := $(c_sources)
fsconfig_CPPFLAGS := $(AM_CPPFLAGS)

fsconfig_LDFLAGS := -all-static



