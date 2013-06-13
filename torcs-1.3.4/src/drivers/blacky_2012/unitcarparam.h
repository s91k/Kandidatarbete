//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitcarparam.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Tuningparameter des Fahrzeugs
//
// Datei    : unitcarparam.h
// Erstellt : 25.11.2007
// Stand    : 10.06.2012
// Copyright: � 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.04.000 (Championship 2012 Alpine-1)
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
#ifndef _UNITCARPARAM_H_
#define _UNITCARPARAM_H_

//==========================================================================*
// Deklaration der Klasse TCarParam
//--------------------------------------------------------------------------*
class TCarParam
{
  public:
	TCarParam();                                 // Default constructor
	~TCarParam();                                // Destructor
	virtual TCarParam& operator= (const TCarParam& CarParam);

  public:
	float oScaleMu;                              // Scaling of MU
	double oScaleMinMu;                          // Scaling of Min MU
	double oScaleBrake;                          // Scaling of decelleration
	double oScaleBrakePit;                       // Scaling of decelleration
	double oScaleBump;                           // Bump sensitivity
	double oScaleBumpOuter;                      // Bump sensitivity at sides

	double oScaleBumpLeft;                       // Bump sensitivity
	double oScaleBumpRight;                      // Bump sensitivity

};
//==========================================================================*
#endif // _UNITCARPARAM_H_
//--------------------------------------------------------------------------*
// end of file unitcarparam.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
