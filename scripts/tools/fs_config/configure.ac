AC_INIT([fsconfig-native],
	1.0.0)
AM_INIT_AUTOMAKE([foreign])

AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_CPP

AC_SUBST([CUTILS_CFLAGS])
AC_SUBST([CUTILS_LIBS])


AM_CONDITIONAL(USE_GLIB, test "x${with_glib}" = "xyes")

AC_CONFIG_FILES([     \
        Makefile    \
        ])

AC_OUTPUT
