##############################################################################
#
#    file                 : Makefile
#    created              : 05 Dec 2007
#    last changed         : 08 May 2012
#    copyright            : (C) 2007-2012 Wolf-Dieter Beelitz
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
SHIPDIR     = drivers/${ROBOT}/0
SHIP        = car6-trb1.rgb tex-wheel.rgb shadow.rgb driver.rgb 
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-wdbee_2012_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-wdbee_2012_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
