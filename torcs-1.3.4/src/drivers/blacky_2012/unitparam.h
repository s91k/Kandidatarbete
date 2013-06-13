//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitparam.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Container f�r Parameter des Fahrzeugs, der Fahrspuren, der Box usw.
//
// Datei    : unitparam.h
// Erstellt : 11.04.2008
// Stand    : 10.06.2012
// Copyright: � 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.04.000 (Championship 2012 Alpine-1)
//--------------------------------------------------------------------------*
// Ein erweiterter TORCS-Roboters
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
#ifndef _UNITPARAM_H_
#define _UNITPARAM_H_

#include "unitglobal.h"
#include "unitcommon.h"

#include "unitcarparam.h"
#include "unitfixcarparam.h"
#include "unitpitparam.h"
#include "unittmpcarparam.h"

//==========================================================================*
// Deklaration der Klasse TParam
//--------------------------------------------------------------------------*
class TParam
{
  public:
	TParam();                                    // Default constructor
    ~TParam();                                   // Destructor

	void Initialize(
	  PtCarElt Car,
	  PDriver Driver);
	void SetEmptyMass(float EmptyMass);
    void Update();

    PtCarElt oCar;                               // Pointer to TORCS data of car
	PDriver oDriver;

	TCarParam oCarParam;                         // Main parameter set
	TCarParam oCarParam2;                        // Avoiding parameter set
	TCarParam oCarParam3;                        // Pitting parameter set

	TPitParam Pit;                               // Parameters of the pit
	TTmpCarParam Tmp;                            // State of the car
	TFixCarParam Fix;                            // Data of the car
};
//==========================================================================*
#endif // _UNITPARAM_H_
//--------------------------------------------------------------------------*
// end of file unitparam.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
