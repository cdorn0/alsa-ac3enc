AC_PREREQ(2.59)
AC_INIT([alsa-ac3enc], [0.1], [christian.dorn@dorn-engineering.com])
AM_INIT_AUTOMAKE
AC_PREFIX_DEFAULT(/usr)

AM_MAINTAINER_MODE([enable])

AC_CONFIG_HEADERS(config.h)
AC_CONFIG_MACRO_DIR([m4])

#### Checks for programs. ####
# ln -s

AC_PROG_LN_S

# CC

AC_PROG_CC
# install
AC_PROG_INSTALL

# pkg-config

PKG_PROG_PKG_CONFIG

# libtool

AM_PROG_LIBTOOL

###################################
#   Environment checks            #
###################################

#### Check for ALSA ###

PKG_CHECK_MODULES(ALSA, alsa >= 1.1.6)


#### Checks for header files. ####

# ISO
AC_HEADER_STDC

#AFTEN
AC_CHECK_HEADERS_ONCE([aften/aften.h aften/aften-types.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_INLINE
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

### Checks for libraries ####

#POSIX
AC_SEARCH_LIBS([dlopen], [dl])

#ALSA
AC_CHECK_LIB(asound, snd_pcm_extplug_create,, AC_ERROR([*** libasound has no external plugin SDK]), -ldl)

#AFTEN
AC_CHECK_LIB(aften, aften_encode_init,, AC_ERROR([*** libaften not found]), )

### Settings ###
AC_DISABLE_STATIC


###################################
#   ALSA Directories              #
###################################

test "x$prefix" = xNONE && prefix=$ac_default_prefix
test "x$exec_prefix" = xNONE && exec_prefix=$prefix

dnl ALSA plugin directory
AC_ARG_WITH(plugindir,
    AS_HELP_STRING([--with-plugindir=dir],
	[path where ALSA plugin files are stored]),
    plugindir="$withval", plugindir="")
if test -z "$plugindir"; then
    eval dir="$libdir"
    case "$dir" in
    /*) ;;
    *) dir="$dir"
    esac
    plugindir="$dir/alsa-lib"
fi
AC_DEFINE_UNQUOTED(ALSA_PLUGIN_DIR, "$plugindir", [directory containing ALSA add-on modules])
ALSA_PLUGIN_DIR="$plugindir"
AC_SUBST(ALSA_PLUGIN_DIR)

dnl ALSA data directory
AC_ARG_WITH(alsadatadir,
    AS_HELP_STRING([--with-alsadatadir=dir],
	[path where ALSA data files are stored]),
    alsadatadir="$withval", alsadatadir="")
if test -z "$alsadatadir"; then
    eval dir="$datadir"
    case "$dir" in
    /*) ;;
    *) eval dir="$dir"
    esac
    alsadatadir="$dir/alsa"
fi
AC_DEFINE_UNQUOTED(ALSA_DATA_DIR, "$alsadatadir", [directory containing ALSA data files])
ALSA_DATA_DIR="$alsadatadir"
AC_SUBST(ALSA_DATA_DIR)

dnl ALSA add-on local config directory
AC_ARG_WITH(alsalconfdir,
    AS_HELP_STRING([--with-alsalconfdir=dir],
	[path where ALSA local add-on config files are stored]),
    alsalconfdir="$withval", alsalconfdir="")
if test -z "$alsalconfdir"; then
    eval dir="$sysconfdir"
    case "$dir" in
    /*) ;;
    *) dir="$dir"
    esac
    alsalconfdir="$dir/alsa/conf.d"
fi

AC_DEFINE_UNQUOTED(ALSA_LCONF_DIR, "$alsalconfdir", [directory containing local ALSA add-on config files])
ALSA_LCONF_DIR="$alsalconfdir"
AC_SUBST(ALSA_LCONF_DIR)

###################################
#   Pulseaudio Directories        #
###################################
dnl Pulseaudio data directory
AC_ARG_WITH(padatadir,
    AS_HELP_STRING([--with-padatadir=dir],
	[path where Pulseaudio data files are stored]),
    padatadir="$withval", padatadir="")
if test -z "$padatadir"; then
    eval dir="$datadir"
    case "$dir" in
    /*) ;;
    *) eval dir="$dir"
    esac
    padatadir="$dir/pulseaudio"
fi
AC_DEFINE_UNQUOTED(PA_DATA_DIR, "padatadir", [directory containing pulse audio data])
PA_DATA_DIR="$padatadir"
AC_SUBST(PA_DATA_DIR)

AC_OUTPUT([
	Makefile
])

dnl Show the build conditions

echo
echo "ALSA Plugin directory: $ALSA_PLUGIN_DIR"
echo "Pulseaudio Data directory: $PA_DATA_DIR"