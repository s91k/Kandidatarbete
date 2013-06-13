//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitcommondata.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Zentrale Klasse f�r das Fahren bzw. den Fahrer/Roboter
//
// Datei    : unitcommondata.h
// Erstellt : 17.11.2007
// Stand    : 10.06.2012
// Copyright: � 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.04.000 (Championship 2012 Alpine-1)
//--------------------------------------------------------------------------*
// Ein erweiterter TORCS-Roboters
//--------------------------------------------------------------------------*
// Teile diese Unit basieren auf diversen Header-Dateien von TORCS
//
//    Copyright: (C) 2000 by Eric Espie
//    eMail    : torcs@free.fr
//
// dem erweiterten Robot-Tutorial bt
//
//    Copyright: (C) 2002-2004 Bernhard Wymann
//    eMail    : berniw@bluewin.ch
//
// dem Roboter delphin
//
//    Copyright: (C) 2006-2007 Wolf-Dieter Beelitz
//    eMail    : wdb@wdbee.de
//
// dem Roboter wdbee_2007
//
//    Copyright: (C) 2006-2007 Wolf-Dieter Beelitz
//    eMail    : wdb@wdbee.de
//
// und dem Roboter mouse_2006
//
//    Copyright: (C) 2006-2007 Tim Foden
//
//--------------------------------------------------------------------------*
// Diese Version wurde mit MS Visual C++ 2005 Express Edition entwickelt.
//--------------------------------------------------------------------------*
// Das Programm wurde unter Windows XP entwickelt und getestet.
// Fehler sind nicht bekannt, dennoch gilt:
// Wer die Dateien verwendet erkennt an, dass f�r Fehler, Sch�den,
// Folgefehler oder Folgesch�den keine Haftung �bernommen wird.
//
// Im �brigen gilt f�r die Nutzung und/oder Weitergabe die
// GNU GPL (General Public License)
// Version 2 oder nach eigener Wahl eine sp�tere Version.
//--------------------------------------------------------------------------*
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//--------------------------------------------------------------------------*
#ifndef _UNITCOMMONDATA_H_
#define _UNITCOMMONDATA_H_

#include <track.h>
#include "unitglobal.h"
#include "unitteammanager.h"
#include "unitclothoid.h"

//==========================================================================*
// Deklaration der Klasse TCommonData
//--------------------------------------------------------------------------*
class TCommonData
{
  public:
    TCommonData();                               // Default constructor
    ~TCommonData();                              // Destructor

  public:
    TTeamManager TeamManager;                    // Team manager
    PTrack Track;                                // TORCS Track data
};
//==========================================================================*
#endif // _UNITCOMMONDATA_H_
//--------------------------------------------------------------------------*
// end of file unitcommondata.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
