//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitcollision.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Kollisionen ausweichen
//
// Datei    : unitcollision.h
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
#ifndef _UNITCOLLISION_H_
#define _UNITCOLLISION_H_

#include <car.h>

#include "unitglobal.h"
#include "unitcommon.h"

#include "unitopponent.h"

//==========================================================================*
// Deklaration der Klasse TCollision
//--------------------------------------------------------------------------*
class TCollision
{
  public:
	struct TCollInfo                             // Infos to possible Collision
	{
	  int Flags;                                 // Flags
	  int LappersBehind;                         // Lappers behind?
	  double MinLSideDist;
	  double MinRSideDist;
	  double MinLDist;                           // Min dist. left side
	  double MinRDist;                           // Min dist. right side
      int NextSide;                              // Side of next curve
	  int OppsAhead;                             // Opponents ahead?
	  int OppsAtSide;                            // Opponents at side?
	  double TargetSpeed;                        // Adjusted target speed
	  double AvoidSide;                          // Avoid to side
	  int OppsNearBehind;
	  int OppsNearInFront;
	  bool Blocked[MAXBLOCKED];

	  TCollInfo():                                // Default constructor
		Flags(0),
		LappersBehind(0),
		MinLSideDist(INT_MAX),
		MinRSideDist(INT_MAX),
		MinLDist(INT_MAX),
		MinRDist(INT_MAX),
		NextSide(0),
		OppsAhead(0),
		OppsAtSide(0),
		TargetSpeed(500),
  	    AvoidSide(0.0),
	    OppsNearBehind(0),
	    OppsNearInFront(0)
	  {
        for (int I = 0; I < MAXBLOCKED; I++)
  	      Blocked[I] = false;
	  }
	};

  public:
	TCollision();                                // Default constructor
	~TCollision();                               // Destructor

	double AvoidTo                               // Direction to go
	  (const TCollInfo& CollInfo,                //   to avoid collision
	  const PCarElt Car,
	  TDriver& Me,
	  bool& AvoidAhead);
};
//==========================================================================*
#endif // _UNITCOLLISION_H_
//--------------------------------------------------------------------------*
// end of file unitcollision.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
