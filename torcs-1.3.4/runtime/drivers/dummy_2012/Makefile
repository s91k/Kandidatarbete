##############################################################################
#
#    file                 : Makefile
#    created              : Sun Jun 11 17:14:57 UTC 2006
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
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp driver.cpp cubic.cpp danpath.cpp path.cpp opponent.cpp spline.cpp pit.cpp pidcontroller.cpp cubiccurve.cpp



SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml logo.rgb
SHIPSUBDIRS = 0 1 setups tracks

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-dummy_2012_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-dummy_2012_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
