##############################################################################
#
#    file                 : Makefile
#    created              : 15 May 2010
#    last changed         : 08 May 2012
#    copyright            : (C) 2010-2012 Wolf-Dieter Beelitz
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = blacky_2012
SHIPDIR     = drivers/${ROBOT}/0
SHIP        = car4-trb1.rgb tex-wheel.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-blacky_2012_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-blacky_2012_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
