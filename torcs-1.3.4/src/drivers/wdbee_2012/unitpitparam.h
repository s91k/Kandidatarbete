//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitpitparam.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Parameter der Box und der Anfahrt zur Box
//
// Datei    : unitpitparam.h
// Erstellt : 11.04.2007
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
#ifndef _UNITPITPARAM_H_
#define _UNITPITPARAM_H_

//==========================================================================*
// Deklaration der Klasse TPitParam
//--------------------------------------------------------------------------*
class TPitParam
{
  private:

  public:
    TPitParam();                                 // Default constructor
	~TPitParam();                                // Destructor

  public:
	double oEntryLong;	   	                     // Translation longitudinal
	double oExitLong;		                     // Translation longitudinal
    float oExitLength;                           // Dist in m

	double oLaneEntryOffset;                     // Additional offset to pit
	double oLaneExitOffset;                      // Additional offset to pit

	double oLatOffset;                           // Lateral offset of pit
	double oLongOffset;                          // Longitudinal offset of pit

	double oStoppingDist;                        // Stopping distance

	int oUseFirstPit;                            // Use special path to first pit
    int oUseSmoothPit;                           // Use smoothing pitlane

};
//==========================================================================*
#endif // _UNITPITPARAM_H_
//--------------------------------------------------------------------------*
// end of file unitpitparam.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
