##############################################################################
#
#    file                 : Makefile
#    created              : Mon Jun 12 18:40:16 UTC 2006
#    copyright            : (C) 2006 Daniel Schellhammer
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = dummy_2012
SHIPDIR     = drivers/${ROBOT}/setups
SHIP        = $(shell find *.xml -maxdepth 0 -type f -print)
SHIPSUBDIRS = qualifying

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-dummy_2012_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-dummy_2012_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))
include ${MAKE_DEFAULT}


