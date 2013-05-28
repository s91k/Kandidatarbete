//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitcarparam.cpp
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter für TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Tuningparameter des Fahrzeugs
//
// Datei    : unitcarparam.cpp
// Erstellt : 25.11.2007
// Stand    : 27.03.2009
// Copyright: © 2007-2009 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 2.00.000
//--------------------------------------------------------------------------*
// Ein erweiterter TORCS-Roboters
//--------------------------------------------------------------------------*
// Diese Unit basiert auf dem Roboter mouse_2006
//
//    Copyright: (C) 2006-2007 Tim Foden
//
//--------------------------------------------------------------------------*
// Diese Version wurde mit MS Visual C++ 2005 Express Edition entwickelt.
//--------------------------------------------------------------------------*
// Das Programm wurde unter Windows XP entwickelt und getestet.
// Fehler sind nicht bekannt, dennoch gilt:
// Wer die Dateien verwendet erkennt an, dass für Fehler, Schäden,
// Folgefehler oder Folgeschäden keine Haftung übernommen wird.
//
// Im übrigen gilt für die Nutzung und/oder Weitergabe die
// GNU GPL (General Public License)
// Version 2 oder nach eigener Wahl eine spätere Version.
//--------------------------------------------------------------------------*
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//--------------------------------------------------------------------------*
#include <math.h>

#include "unitglobal.h"
#include "unitcommon.h"

#include "unitparabel.h"
#include "unitcarparam.h"

//==========================================================================*
// Default constructor
//--------------------------------------------------------------------------*
TCarParam::TCarParam():
  oScaleMu(1.0),
  oScaleMinMu(0.8),
  oScaleBrake(1.0),
  oScaleBrakePit(1.0),
  oScaleBump(0),
  oScaleBumpOuter(0),
  oScaleBumpLeft(0),
  oScaleBumpRight(0)
{
}
//==========================================================================*

//==========================================================================*
// Destructor
//--------------------------------------------------------------------------*
TCarParam::~TCarParam()
{
}
//===========================================================================

//==========================================================================*
// Zuweisung
//--------------------------------------------------------------------------*
TCarParam& TCarParam::operator= (const TCarParam& CarParam)
{
  oScaleMu = CarParam.oScaleMu;
  oScaleMinMu = CarParam.oScaleMinMu;
  oScaleBrake = CarParam.oScaleBrake;
  oScaleBrakePit = CarParam.oScaleBrakePit;
  oScaleBump = CarParam.oScaleBump;
  oScaleBumpOuter = CarParam.oScaleBumpOuter;
  oScaleBumpLeft = CarParam.oScaleBumpLeft;
  oScaleBumpRight = CarParam.oScaleBumpRight;

  return *this;
}
//===========================================================================

//--------------------------------------------------------------------------*
// end of file unitcarparam.cpp
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
