##############################################################################
#
#    file                 : Makefile
#    created              : 14 Dec 2008
#    last changed         : 26 Mar 2012
#    copyright            : (C) 2008-2012 Wolf-Dieter Beelitz
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
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = unitmain.cpp \
		unitcarparam.cpp \
		unitcharacteristic.cpp \
		unitclothoid.cpp \
		unitcollision.cpp \
		unitcommon.cpp \
		unitcommondata.cpp \
		unitcubic.cpp \
		unitcubicspline.cpp \
		unitdriver.cpp \
		unitfixcarparam.cpp \
		unitlane.cpp \
		unitlanepoint.cpp \
		unitlinalg.cpp \
		unitlinreg.cpp \
		unitopponent.cpp \
		unitparabel.cpp \
		unitparam.cpp \
		unitpidctrl.cpp \
		unitpit.cpp \
		unitpitparam.cpp \
		unitsection.cpp \
		unitstrategy.cpp \
		unitsysfoo.cpp \
		unitteammanager.cpp \
		unittmpcarparam.cpp \
		unittrack.cpp \
		unitvec2d.cpp \
		unitvec3d.cpp
SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml logo.rgb readme.txt
SHIPSUBDIRS = 0 1 car6-trb1 tracks

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-wdbee_2012_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-wdbee_2012_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
