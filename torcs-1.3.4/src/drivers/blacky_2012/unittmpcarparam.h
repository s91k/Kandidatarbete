//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unittmpcarparam.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter für TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Zeitlich variable Parameter des Fahrzeugs
//
// Datei    : unittmpcarparam.h
// Erstellt : 25.11.2007
// Stand    : 10.06.2012
// Copyright: © 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.04.000 (Championship 2012 Alpine-1)
//--------------------------------------------------------------------------*
// Ein erweiterter TORCS-Roboters
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
#ifndef _UNITTMPCARPARAM_H_
#define _UNITTMPCARPARAM_H_

#include "unitglobal.h"
#include "unitcommon.h"

//==========================================================================*
// Deklaration der Klasse TTmpCarParam
//--------------------------------------------------------------------------*
class TTmpCarParam
{
  private:

  public:
    PtCarElt oCar;                               // Pointer to TORCS data of car

	TTmpCarParam();                              // Default constructor
	~TTmpCarParam();                             // Destructor

	void Initialize(PtCarElt Car);
    bool Needed();
	void Update();

  public:
	double oDamage;                              // Damage of this car
	double oEmptyMass;                           // Mass of car wihtout fuel
	double oFuel;                                // Mass of fuel in car
	double oMass;                                // Mass of car.with fuel
};
//==========================================================================*
#endif // _UNITTMPCARPARAM_H_
//--------------------------------------------------------------------------*
// end of file unittmpcarparam.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
