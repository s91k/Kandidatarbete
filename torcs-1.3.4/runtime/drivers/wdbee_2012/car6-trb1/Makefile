##############################################################################
#
#    file                 : Makefile
#    created              : 07 Dec 2007
#    last changed         : 08 Mar 2012
#    copyright            : (C) 2007-2012 Wolf-Dieter Beelitz
#    version              : $Id$
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = wdbee_2012
SHIPDIR     = drivers/${ROBOT}/car6-trb1
SHIP        = $(shell find * -maxdepth 0 -type f -print)
SHIPSUBDIRS = $(shell find * -maxdepth 0 -type d -print)

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-wdbee_2012_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-wdbee_2012_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
