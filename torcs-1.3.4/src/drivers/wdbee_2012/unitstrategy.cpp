//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitstrategy.cpp
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Boxenstop-Strategie
// (C++-Portierung der Unit UnitStrategy.pas)
//
// Datei    : unitstrategy.cpp
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
#include "unitglobal.h"
#include "unitcommon.h"

#include "unitstrategy.h"

//==========================================================================*
// Konstanten
//--------------------------------------------------------------------------*
const float TSimpleStrategy::cMAX_FUEL_PER_METER = 0.0008f;
int TSimpleStrategy::cPIT_DAMAGE = 5000;
//==========================================================================*

//==========================================================================*
// Strategie erzeugen
//--------------------------------------------------------------------------*
TSimpleStrategy::TSimpleStrategy():
	oWasInPit(false),
    oFuelChecked(false),
    oFuelPerM(0.0f),
    oLastPitFuel(0.0f),
    oLastFuel(0.0f),
    oExpectedFuelPerM(0.0f),
    oRaceDistance(0.0f),
    oRemainingDistance(0.0f),
    oReserve(0.0f),
    oTrackLength(0.0f),
    oMaxFuel(91.0f),
	oMinLaps(3)
{
}
//==========================================================================*

//==========================================================================*
// Aufr�umen
//--------------------------------------------------------------------------*
TSimpleStrategy::~TSimpleStrategy()
{
  if (oPit != NULL)
	delete oPit;
}
//==========================================================================*

//==========================================================================*
// Strategie initialisieren
//--------------------------------------------------------------------------*
void TSimpleStrategy::Init(TDriver *Driver)
{
  oDriver = Driver;
  oPit = new TPit(Driver);
}
//==========================================================================*

//==========================================================================*
// Ist die Box frei?
//--------------------------------------------------------------------------*
bool TSimpleStrategy::IsPitFree()
{
  if (CarPit != NULL)
  {
	TTeamManager::TTeam* Team = oDriver->GetTeam();
	if (Team->PitState != PIT_IS_FREE)
	{
      // Get teammate
	  TTeamManager::TTeammate* Mate = Team->Member;
	  while (Mate->Car == oDriver->Car())
	    Mate = Mate->Next;

      // Check teammate's race state
      if ((Mate->Car->_state & RM_CAR_STATE_NO_SIMU)    // If out of race
        && (Mate->Car->_state & RM_CAR_STATE_PIT) != 0 )//   and not in pit
	    Team->PitState = PIT_IS_FREE;                   // Free pit 

	}
	if ((CarPit->pitCarIndex == TR_PIT_STATE_FREE)
      && ((Team->PitState == CarDriverIndex) || (Team->PitState == PIT_IS_FREE)))
      return true;
  }
  return false;
}
//==========================================================================*

//==========================================================================*
// Entscheiden, ob Boxenstopp n�tig ist
//--------------------------------------------------------------------------*
bool TSimpleStrategy::NeedPitStop()
{
  double FuelConsum;                             // Spritverbrauch (pro m)
  double FuelNeeded;                             // F�r n�chste Runde n�tig

  bool Result = false;                           // Annahme: Kein Boxenstopp

  if (CarPit == NULL)                            // Ist eine Box vorhanden?
    return Result;                               //   Wenn nicht, Pech!

  if (!IsPitFree())                              // Ist Box frei?
    return Result;                               //   Wenn nicht, Pech!

  oRemainingDistance =                           // Restliche Strecke des
    oRaceDistance - DistanceRaced;               //   Rennens ermitteln

  oRemainingDistance -=                          // Verk�rzt um R�ckstand
	oTrackLength * CarLapsBehindLeader;          //   auf den F�hrenden

  if (oRemainingDistance > oTrackLength + 100)   // Wenn noch mehr km
  {                                              //   zu fahren sind
    if (oFuelPerM == 0.0)                        // Spritverbrauch pro m
      FuelConsum = oExpectedFuelPerM;            //   sch�tzen
    else                                         // oder den gemessenen
      FuelConsum = oFuelPerM;                    //   Wert nehmen

    FuelNeeded =                                 // Bedarf an Treibstoff
      MIN(oTrackLength+oReserve,                 // Bis z. n. Boxenstopp oder
	  oRemainingDistance+oReserve) * FuelConsum; // bis zum Ende des Rennens

    if (CarFuel < FuelNeeded)                    // Wenn der Tankinhalt nicht
	{
      Result = true;                             //   reicht, tanken
      GfOut("Pitstop by fuel\n");
	}
	else                                         // Ansonsten pr�fen, f�r
	{                                            //   welche Anzahl von Runden
	  TTeamManager::TTeam* Team =                //   alle Teammitglieder
		oDriver->GetTeam();                      //   noch Treibstoff haben

      FuelNeeded = FuelConsum * oTrackLength;    // Treibstoff f�r eine Runde
	  int FuelForLaps =                          // Eigene Reichweite
        Team->FuelForLaps[CarDriverIndex] =
	    (int) (CarFuel / FuelNeeded - 1);
	  int MinLaps = Team->GetMinLaps(oCar);      // Mindestreichweite der anderen
	  //GfOut("ID: %d FuelForLaps: %d MinLaps: %d (%s)\n",
	  //  CarDriverIndex,Team->FuelForLaps[CarDriverIndex],MinLaps,oDriver->GetBotName());

	  // Wenn Tanken, dann der, der weniger Runden weit kommen w�rde
      if (FuelForLaps < MinLaps)
      {
        if ((MinLaps < oDriver->GetMinLaps())
  		  && (RemainingLaps > FuelForLaps))
	    {                                        // Nur Tanken, wenn n�tig!
          FuelNeeded = (oRemainingDistance + oReserve) * FuelConsum;
          if (CarFuel < FuelNeeded)              // Wenn der Tankinhalt nicht
	      {                                      // reicht, tanken
            Result = true;
            GfOut("Pitstop by Teammate\n");
	      }
        }
	  }
	}

    if (RepairWanted(cPIT_DAMAGE) > 0)           // Wenn die Sch�den zu hoch
	{                                            //   reparieren lassen
       Result = true;
	}
  };

  // Check the estimated lap time
  double DistToStart = oTrack->length - CarDistFromStart;
  double RemainingLapTime = DistToStart / oTrack->length * CarLastLapTime * 1.5; 
  double PitTime = 2.0 + (oMaxFuel - CarFuel) / 8.0 + RepairWanted(0) * 0.007;
  double LapTime = CarCurLapTime + RemainingLapTime + PitTime;
  if (LapTime > 84.5 + oTrack->length/10.0)
	Result = false;

  if (Result)
  {
	TTeamManager::TTeam* Team = oDriver->GetTeam();
	Team->PitState = CarDriverIndex;             // Box reserviert
  }
  return Result;
};
//==========================================================================*

//==========================================================================*
// Box freigeben
//--------------------------------------------------------------------------*
void TAbstractStrategy::PitRelease()
{
  TTeamManager::TTeam* Team = oDriver->GetTeam();
  if (Team->PitState == CarDriverIndex)          // Box f�r mich reserviert?
  {
    Team->PitState = PIT_IS_FREE;
    oCar->ctrl.raceCmd = 0;
  }
};
//==========================================================================*
/*
//==========================================================================*
// Tanken
//--------------------------------------------------------------------------*
float TSimpleStrategy::PitRefuel()
{
  float FuelConsum;                              // Spritverbrauch kg/m
  float Fuel;                                    // Menge in kg

  if (oFuelPerM == 0.0)                          // Wenn kein Messwert
    FuelConsum = oExpectedFuelPerM;              //   vorliegt, sch�tzen
  else                                           // ansonsten
    FuelConsum = oFuelPerM;                      //   Messwert nehmen

  FuelConsum *= oDriver->oScaleRefuel;           // ggf. ohne Windschatten!

  oRemainingDistance =                           // Restliche Strecke des
    oRaceDistance - DistanceRaced;               //   Rennens ermitteln

  Fuel = oMaxFuel;                               // Bedarf an Treibstoff

  if (Fuel > oMaxFuel - CarFuel)                 // Menge ggf. auf freien
    Fuel = oMaxFuel - CarFuel;                   // Tankinhalt begrenzen
  else                                           // ansonsten Bedarf
    Fuel = Fuel - CarFuel;                       // abz�gl. Tankinhalt

  oLastPitFuel = (float) MAX(Fuel,0.0);          // Wenn genug da ist = 0.0

  return oLastPitFuel;                           // Menge an TORCS melden
};
//==========================================================================*
*/
//==========================================================================*
// Tanken
//--------------------------------------------------------------------------*
float TSimpleStrategy::PitRefuel()
{
  float FuelConsum;                              // Spritverbrauch kg/m
  float Fuel;                                    // Menge in kg

  if (oFuelPerM == 0.0)                          // Wenn kein Messwert
    FuelConsum = oExpectedFuelPerM;              //   vorliegt, sch�tzen
  else                                           // ansonsten
    FuelConsum = oFuelPerM;                      //   Messwert nehmen

  FuelConsum *= oDriver->oScaleRefuel;           // ggf. ohne Windschatten!

  oRemainingDistance =                           // Restliche Strecke des
    oRaceDistance - DistanceRaced;               //   Rennens ermitteln

  Fuel =                                         // Bedarf an Treibstoff
    (oRemainingDistance + oReserve) * FuelConsum;// f�r restliche Strecke

  if (Fuel > oMaxFuel)                           // Wenn mehr als eine Tank-
  {                                              //   f�llung ben�tigt wird
    if (Fuel / 2 < oMaxFuel)                     // Bei zwei Tankf�llungen
      Fuel = Fuel / 2;                           //   die H�lfte tanken
    else if (Fuel / 3 < oMaxFuel)                // Bei drei Tankf�llungen.
      Fuel = Fuel / 3;                           //   ein Drittel tanken
    else if (Fuel / 4 < oMaxFuel)                // Bei vier Tankf�llungen.
      Fuel = Fuel / 4;                           //   ein Viertel tanken
    else                                         // Bei f�nf Tankf�llungen
      Fuel = Fuel / 5;                           //   ein F�nftel tanken
  }

//  if (oRemainingDistance > 100000)               // Corkscrew
//    Fuel = oMaxFuel;

  if (Fuel > oMaxFuel - CarFuel)                 // Menge ggf. auf freien
    Fuel = oMaxFuel - CarFuel;                   // Tankinhalt begrenzen
  else                                           // ansonsten Bedarf
    Fuel = Fuel - CarFuel;                       // abz�gl. Tankinhalt

  oLastPitFuel = (float) MAX(Fuel,0.0);          // Wenn genug da ist = 0.0

  return oLastPitFuel;                           // Menge an TORCS melden
};
//==========================================================================*

//==========================================================================*
// Umfang der Reparaturen festlegen
//--------------------------------------------------------------------------*
int TSimpleStrategy::RepairWanted(int AcceptedDamage)
{
  if (oCar->_dammage < AcceptedDamage)
	return 0;
  else if (oRemainingDistance > 5.5 * oTrackLength)
    return oCar->_dammage;
  else if (oRemainingDistance > 4.5 * oTrackLength)
    return MAX(0,oCar->_dammage - cPIT_DAMAGE);
  else if (oRemainingDistance > 3.5 * oTrackLength)
    return MAX(0,oCar->_dammage - cPIT_DAMAGE - 1000);
  else if (oRemainingDistance > 2.5 * oTrackLength)
    return MAX(0,oCar->_dammage - cPIT_DAMAGE - 2000);
  else
    return MAX(0,oCar->_dammage - cPIT_DAMAGE - 3000);
}
//==========================================================================*

//==========================================================================*
// Umfang der Reparaturen festlegen
//--------------------------------------------------------------------------*
int TSimpleStrategy::PitRepair()
{
  oState = PIT_EXIT;
  oWasInPit = true;
  return RepairWanted(0);
}
//==========================================================================*

//==========================================================================*
// Tankf�llung beim Start bestimmen
//--------------------------------------------------------------------------*
double TSimpleStrategy::SetFuelAtRaceStart
  (PTrack Track, PCarSettings *CarSettings, PSituation Situation, float Fuel)
{
  oTrack = Track;                                // Save TORCS pointer

  cPIT_DAMAGE = (int)
	GfParmGetNum(*CarSettings,TDriver::SECT_PRIV,
	(char *) PRV_MAX_DAMAGE,(char*) NULL, (float) cPIT_DAMAGE);
  GfOut("cPIT_DAMAGE = %d\n",cPIT_DAMAGE);

  oTrackLength = oTrack->length;                 // L�nge der Strecke merken
  oRaceDistance =                                // Gesamtl�nge des Rennens
    oTrackLength * Situation->_totLaps;          //   berechnen
  oRemainingDistance =                           // Restliche Strecke des
    oRaceDistance + oReserve;                    //   Rennens ermitteln
  Fuel = (float)
    (Fuel * oRemainingDistance / 100000.0);      // Gesamtbedarf in kg

  oExpectedFuelPerM = Fuel / oRemainingDistance; // Verbrauch in kg/m

  oMaxFuel =
	GfParmGetNum(*CarSettings,TDriver::SECT_PRIV,// Maximal m�glicher
	(char *) PRV_MAX_FUEL,(char*) NULL,oMaxFuel);//   Tankinhalt
  GfOut("oMaxFuel (private) = %.1f\n",oMaxFuel);

  oStartFuel = 0.0;                              // initialisieren         
  oStartFuel =            
	GfParmGetNum(*CarSettings,TDriver::SECT_PRIV,// Tankinhalt beim Start
	PRV_START_FUEL,(char*) NULL,oStartFuel);     
  if (oStartFuel > 0.0)
    GfOut("oStartFuel (private) = %.1f\n",oStartFuel);

  if (oStartFuel > 0)
  {
    oLastFuel = oStartFuel;                      // volltanken
    GfParmSetNum(*CarSettings,SECT_CAR,PRM_FUEL, // Gew�nschte Tankf�llung
      (char*) NULL, oLastFuel);                  //   an TORCS melden
    return oLastFuel;    
  }

  if (Fuel == 0)                                 // Wenn nichts bekannt ist,
    Fuel = oMaxFuel;                             // Volltanken

  oLastFuel = Fuel;                              // Erforderlicher Treibstoff
  if (Fuel > oMaxFuel)                           // Wenn mehr als eine Tank-
  {                                              //   f�llung ben�tigt wird
    if (Fuel / 2 < oMaxFuel)                     // Bei zwei Tankf�llungen
      oLastFuel = Fuel / 2;                      //   die H�lfte tanken
    else if (Fuel / 3 < oMaxFuel)                // Bei drei Tankf�llungen.
      oLastFuel = Fuel / 3;                      //   ein Drittel tanken
    else if (Fuel / 4 < oMaxFuel)                // Bei vier Tankf�llungen.
      oLastFuel = Fuel / 4;                      //   ein Viertel tanken
    else                                         // Bei f�nf Tankf�llungen
      oLastFuel = Fuel / 5;                      //   ein F�nftel tanken
  };

//  if (!TDriver::Qualification) // Corcscrew
//    oLastFuel = oMaxFuel;      //

  oLastFuel = MIN(oLastFuel, oMaxFuel);          // �berlaufen verhindern
  GfParmSetNum(*CarSettings,
    (char *) SECT_CAR,
    (char *) PRM_FUEL,                           // Gew�nschte Tankf�llung
    (char *) NULL, oLastFuel);                   //   an TORCS melden

  return oLastFuel;
};
//==========================================================================*

//==========================================================================*
// Go to pit
//--------------------------------------------------------------------------*
bool TSimpleStrategy::GoToPit()
{
  return ((oState >= PIT_ENTER) && (oState <= PIT_GONE));
};
//==========================================================================*

//==========================================================================*
// Start entry procedure to pit?
//--------------------------------------------------------------------------*
bool TSimpleStrategy::StartPitEntry(float& Ratio)
{
  float DLong, DLat;                             // Dist to Pit
  RtDistToPit(oCar,oTrack,&DLong,&DLat);
  if (GoToPit() && (DLong < oPit->oPitLane->PitDist()))
  {
    Ratio = (float) (1.0 - MAX(0.0,(DLong-100)/oPit->oPitLane->PitDist()));
    return true;
  }
  else
    return false;
};
//==========================================================================*

//==========================================================================*
// Stop entry procedure to pit?
//--------------------------------------------------------------------------*
bool TSimpleStrategy::StopPitEntry(float Offset)
{
  float DLong, DLat;                             // Dist to Pit
  RtDistToPit(oCar,oTrack,&DLong,&DLat);
  if (oWasInPit && (DLong - oTrackLength) > -Offset)
  {
    return true;
  }
  else
  {
    oWasInPit = false;
    return false;
  }
};
//==========================================================================*

//==========================================================================*
// Update Data
//--------------------------------------------------------------------------*
void TSimpleStrategy::Update(PtCarElt Car)
{
  double CurrentFuelConsum;                      // Current fuel consum
  float DL = 99999.9f;                           // Distance longitudinal
  float DW = 0.0;                                // Distance lateral

  oCar = Car;                                    // Save pointer

  RtDistToPit                                    // Get distance to pit
   (Car,oTrack,&DL,&DW);

  if (DL < 0)                                    // Make DL be >= 0.0
	DL = DL + oTrack->length;                    // to have it to front

  if ((DL < oDistToSwitch) && (DL > 50) && (!oFuelChecked))
  { // We passed the line to check our fuel consume!
    if (CarLaps > 1)                             // Start at lap 2
    {                                            //   to get values
      CurrentFuelConsum =                        // Current consume =
        (oLastFuel                               // Last tank capacity
        + oLastPitFuel                           // + refueled
        - oCar->priv.fuel)                       // - current capacity
        / oTrackLength;                          // related to track length

      if (oFuelPerM == 0.0)                      // At first time we use
        oFuelPerM = (float) CurrentFuelConsum;    //   our initial estimation
      else                                       // later
        oFuelPerM = (float)                      //   we get the mean
          ((oFuelPerM*CarLaps+CurrentFuelConsum) //   of what we needed till now
		  / (CarLaps + 1));

    };

    oLastFuel = oCar->priv.fuel;                 // Capacity at this estimation
    oLastPitFuel = 0.0;                          // Refueled
    oFuelChecked = true;                         // We did the estimation in this lap

    if (!oGoToPit)                               // If decision isn'd made
	{
  	  oGoToPit = NeedPitStop();                  // Check if we sholud have a pitstop
      //if (oGoToPit) GfOut("Drv:%d/%d GoToPit\n",CarDriverIndex,CarLaps);
	}
  }
  else if (DL < 50)                              // I we are out of the window
  {                                              // to estimate
    oFuelChecked = false;                        // reset flag
  };
};
//==========================================================================*

//==========================================================================*
// State (Sequential logic system)
//--------------------------------------------------------------------------*
void TSimpleStrategy::CheckPitState(float PitScaleBrake)
{
  if (oPit == NULL)                              // No Pit no service
    return;
  else if (!oPit->HasPits())
	return;

  double TrackPos = RtGetDistFromStart(oCar);    // Distance to pit

  switch(oState)                                 // Check state
  {
	case PIT_NONE:
      // We are somewhere on the track, nothing has happend yet
	  if ((!oPit->oPitLane[0].InPitSection(TrackPos)) && oGoToPit)
	  { // if we are not parallel to the pits and get the flag,
		// let's stop in the pits.
		oState = PIT_BEFORE;
	  }
	  break;

	case PIT_BEFORE:
      // We are somewhere on the track and got the flag to go to pit
	  if (oPit->oPitLane[0].InPitSection(TrackPos) && oGoToPit)
	  { // If we reache pit entry and flag is still set
	    // switch to the pitlane
		oState = PIT_ENTER;
	  }
	  break;

	case PIT_ENTER:
      // We are on the pitlane and drive to out pit
 	  if (!oPit->oPitLane[0].CanStop(TrackPos))
	  { // We have to wait, till we reached the point to stop
		break;
	  }

	  // We reached the poit to stopp
	  oState = PIT_ASKED;

 	  // falls through...

	case PIT_ASKED:
	  // We are still going to the pit
	  if (oPit->oPitLane[0].CanStop(TrackPos))
	  { // If we can stop a this position we start pit procedure
		oDriver->oStanding = true;               // For motion survey!
        oPitTicker = 0;                          // Start service timer
	    oCar->ctrl.accelCmd = 0;                 // release throttle
	    oCar->ctrl.brakeCmd = 0.5f*PitScaleBrake;// Start braking
	    oCar->ctrl.raceCmd = RM_CMD_PIT_ASKED;   // Tell TORCS to service us! To test oPitTicker comment out
	    oState = PIT_SERVICE;
	  }
	  else
	  { // We can't stop here (to early or to late)
	    if (oPit->oPitLane[0].Overrun(TrackPos))
		{ // if to late
		  GfOut("Overrun 1: %g\n",TrackPos);
		  PitRelease();
	      oState = PIT_EXIT;
	      // pit stop finished, need to exit pits now.
		}
	  }
	  break;

	case PIT_SERVICE:
      // Wait to reach standstill to get service from TORCS
	  oDriver->oStanding = true;                 // Keep motion survey quiet
	  oPitTicker++;                              // Check time to start service
	  if (oPitTicker > 150)                      // Check Timer
	  { // If we have to wait to long
		GfOut("oPitTicker: %d\n",oPitTicker);
	    PitRelease();                            // Something went wrong, we have
	    oState = PIT_EXIT;                       // to leave and release pit for teammate
	  }
	  else if (oPit->oPitLane[0].Overrun(TrackPos))
	  { // If we couldn't stop in place
		GfOut("Overrun 2: %g\n",TrackPos);
	    PitRelease();                            // We have to release the pit
	    oState = PIT_EXIT;                       // for teammate
	  }
	  else
	  { // There is nothing that hampers TORCS to service us
        oCar->ctrl.accelCmd = 0;                 // No throttle
	    oCar->ctrl.brakeCmd = 1.0f*PitScaleBrake;// Still braking
	    oCar->ctrl.raceCmd = RM_CMD_PIT_ASKED;   // Tell TORCS to service us! To test oPitTicker comment out
        // oState is set to next state in PitRepair()!
		// If TORCS doesn't service us, no call to PitRepair() is done!
		// We run into timeout! (oPitTicker)
	  }
	  break;

	case PIT_EXIT:
      // We are still in the box
	  oDriver->oStanding = true;                 // Keep motion survey quiet
  	  oGoToPit = false;                          // Service is finished, lets go
	  oCar->ctrl.accelCmd = 0.5;                 // Press throttle
	  oCar->ctrl.brakeCmd = 0;                   // Release brake
	  PitRelease();                              // Release pit for teammate
	  if (oDriver->oCurrSpeed > 5)
	    oState = PIT_GONE;
	  break;

	case PIT_GONE:
      // We are on our path back to the track
	  if (!oPit->oPitLane[0].InPitSection(TrackPos))
	  { // If we reached the end of the pitlane
		oState = PIT_NONE;                       // Switch to default mode
	  }
 	  break;
  }
}
//==========================================================================*
// end of file unitstrategy.cpp
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
