# process this file with autoconf to get a configure script
AC_INIT(libvisca, 1.1.1)
AC_CONFIG_SRCDIR([visca/libvisca.c])
AM_INIT_AUTOMAKE

AM_CONFIG_HEADER(config.h)

AC_PROG_CC
AC_PROG_LIBTOOL

AC_HEADER_STDC
AC_CHECK_HEADERS(fcntl.h sys/ioctl.h unistd.h)

AC_TYPE_SIZE_T

AC_FUNC_MMAP

dnl ---------------------------------------------------------------------------
dnl When making a release:
dnl  1. If the library source code has changed at all since the last release,
dnl     then increment REVISION.
dnl  2. If the interface (ABI) has been modified since the last release,
dnl     increment MAJOR, and set REVISION to 0.
dnl  3. If the interface changes consist solely of additions, increment AGE.
dnl  4. If the interface has removed or changed elements, set AGE to 0.
dnl ---------------------------------------------------------------------------
LT_INIT([win32-dll])
lt_major=3
lt_revision=1
lt_age=0

AC_SUBST(lt_major)
AC_SUBST(lt_revision)
AC_SUBST(lt_age)

AC_CANONICAL_HOST
AM_CONDITIONAL([HOST_WIN32], [test x$host_os = xmingw32])

pkgincludedir='$(includedir)/visca'
AC_SUBST(pkgincludedir)

CFLAGS="$CFLAGS -Wall -Wunused"
AC_OUTPUT([ Makefile visca/Makefile examples/Makefile VC2005/Makefile])
