//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitstrategy.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Boxenstop-Strategie
// (C++-Portierung der Unit UnitStrategy.pas)
//
// Datei    : unitstrategy.h
// Erstellt : 20.02.2007
// Stand    : 10.06.2012
// Copyright: � 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.04.000 (Championship 2012 Alpine-1)
//--------------------------------------------------------------------------*
// Teile diese Unit basieren auf dem erweiterten Robot-Tutorial bt
//
//    Copyright: (C) 2002-2004 Bernhard Wymann
//    eMail    : berniw@bluewin.ch
//
// dem Roboter delphin
//
//    Copyright: (C) 2006-2007 Wolf-Dieter Beelitz
//    eMail    : wdb@wdbee.de
//
// und dem Roboter mouse_2006
//    Copyright: (C) 2006 Tim Foden
//
//--------------------------------------------------------------------------*
// Diese Version wurde mit MS VC++ 6.0 entwickelt und unter Windows getestet.
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
#ifndef _UNITSTRATEGY_H_
#define _UNITSTRATEGY_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "unitglobal.h"
#include "unitcommon.h"
#include "unitdriver.h"
#include "unitpit.h"

//==========================================================================*
// Basisklasse f�r Strategien zum Boxenstopp
//--------------------------------------------------------------------------*
class TAbstractStrategy
{
  protected:
	enum
	{
	  PIT_NONE,
	  PIT_BEFORE,
	  PIT_ENTER,
	  PIT_ASKED,
	  PIT_SERVICE,
	  PIT_EXIT,
	  PIT_GONE
	};
	int	oState;		                             // Current pitting state
    bool oGoToPit;                               // Pit stop needed

  public:
    PtCarElt oCar;                               // TORCS-Daten
	PTrack oTrack;                               // Track-Daten
	PPit oPit;
	PDriver oDriver;

	double oDistToSwitch;                        // Dist to Pit

	TAbstractStrategy()
	  :oState(PIT_NONE),oGoToPit(false),oCar(NULL),oTrack(NULL),oPit(NULL){};
	virtual ~TAbstractStrategy(){};

	virtual bool IsPitFree() = 0;
	virtual bool NeedPitStop() = 0;
	virtual float PitRefuel() = 0;
	virtual int PitRepair() = 0;
	virtual int RepairWanted
	  (int AcceptedDamage) = 0;
	virtual double SetFuelAtRaceStart
	  (PTrack Track, PCarSettings *CarSettings, PSituation Situation, float Fuel) = 0;
	virtual void Update
	  (PtCarElt Car) = 0;
	void PitIsFree();

    virtual void CheckPitState(float PitScaleBrake) = 0;
	virtual bool GoToPit() = 0;
	virtual bool StartPitEntry(float& Ratio) = 0;
	virtual bool StopPitEntry(float Offset) = 0;

	void TestPitStop(){oGoToPit = true;};
	void PitRelease();
	int GetState(){return oState;};
	bool OutOfPitlane(){return oState < PIT_ENTER;};
};
//==========================================================================*

//==========================================================================*
// Einfache Strategie f�r Boxenstopps (Tanken und Reparieren)
//--------------------------------------------------------------------------*
class TSimpleStrategy
  : public TAbstractStrategy
{
  private:

  public:
	bool oWasInPit;             // Was in pit?
    bool oFuelChecked;          // Treibstoffverbrauchsstatistik berechnet?
    float oFuelPerM;            // Mittlerer Treibstoffverbrauch in kg pro m
    float oLastPitFuel;         // Getankte Menge an Treibstoff
    float oLastFuel;            // Tankinhalt beim �berqueren der Startlinie
    float oExpectedFuelPerM;    // Gesch. Treibst.bedarf f�rs Rennen in kg/m
	int oPitTicker;             // Overrunner
    float oRaceDistance;        // Gesamtl�nge des Rennens in m
    float oRemainingDistance;   // Verbeleibende L�nge des Rennens in m
    float oReserve;             // Reserve in m
    float oTrackLength;         // L�nge der Rennstrecke in m
    float oMaxFuel;             // Maximaler Tankinhalt in kg
	float oStartFuel;           // Fuel at start
    int oMinLaps;               // Mindestanzahl von Runden mit Tankinhalt

	TSimpleStrategy();
	~TSimpleStrategy();

    void Init(PDriver Driver);

	bool IsPitFree();
	bool NeedPitStop();
	float PitRefuel();
	int PitRepair();
	int RepairWanted
	  (int AcceptedDamage);
	double SetFuelAtRaceStart
	  (PTrack Track, PCarSettings *CarSettings, PSituation Situation, float Fuel);
	void Update
	  (PtCarElt Car);

    void CheckPitState(float PitScaleBrake);
	bool GoToPit();
	bool StartPitEntry(float& Ratio);
	bool StopPitEntry(float Offset);

	static const float cMAX_FUEL_PER_METER;	// [kg/m] Gesch�tzter Spritverbrauch
	static int cPIT_DAMAGE;			        // Grenzwert f�r Sch�den
};
//==========================================================================*
#endif // _UNITSTRATEGY_H_
//--------------------------------------------------------------------------*
// end of file unitstrategy.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
