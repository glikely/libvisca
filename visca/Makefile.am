# the libvisca itself
MAINTAINERCLEANFILES = Makefile.in
lib_LTLIBRARIES = libvisca.la
AM_CFLAGS = -I$(top_srcdir)
EXTRA_DIST = libvisca_avr.c libvisca_win32.c

libvisca_la_LDFLAGS = -no-undefined -version-info @lt_major@:@lt_revision@:@lt_age@

libvisca_la_SOURCES = libvisca.c libvisca.h

if HOST_WIN32
libvisca_la_SOURCES += libvisca_win32.c
AM_CFLAGS += -DDLL_EXPORTS
else
libvisca_la_SOURCES += libvisca_posix.c
endif

# headers to be installed
pkginclude_HEADERS = libvisca.h

