//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unittrack.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Streckenbeschreibung
// (C++-Portierung der Unit UnitTrack.pas)
//
// Datei    : unittrack.h
// Erstellt : 17.11.2007
// Stand    : 10.06.2012
// Copyright: � 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.04.000 (Championship 2012 Alpine-1)
//--------------------------------------------------------------------------*
// Stellt Funktionen zur Streckenbeschreibung zur Verf�gung
//--------------------------------------------------------------------------*
// Teile diese Unit basieren auf diversen Header-Dateien von TORCS
//
//    Copyright: (C) 2000 by Eric Espie
//    eMail    : torcs@free.fr
//
// dem Robot berniw two
//
//    Copyright: (C) 2000-2002 by Bernhard Wymann
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
//    Copyright: (C) 2006 Tim Foden
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
#ifndef _UNITTRACK_H_
#define _UNITTRACK_H_

// TORCS
#include <track.h>
#include <car.h>

// This robot
#include "unitglobal.h"
#include "unitcommon.h"
#include "unitcarparam.h"
#include "unitsection.h"
#include "unitlanepoint.h"

//==========================================================================*
// Class TTrackDescription
//--------------------------------------------------------------------------*
class TTrackDescription
{
  public:
	struct PitSideMod
	{
		PitSideMod() : side(-1), start(0), end(0) {}

		int		side;                            // side of pitlane
		int		entry;
		int		start;                           // start of pitlane
		int		end;                             // end of pitlane
		int		exit;
	};

  public:
	TTrackDescription();                         // Default constructor
	~TTrackDescription() ;                       // destructor

	double CalcPos                               // Calc position from offset
	  (tTrkLocPos& TrkPos, double Offset = 0) const;
	double CalcPos                               // Calc cars position from offset
	  (tCarElt* Car, double Offset = 0) const;
	double CalcPos                               // Calc position from coordinates
	  (float X, float Y,
	  const TSection* Hint = 0,
	  bool Sides = false) const;

	int Count() const;                           // nbr of section in track
    void Execute();                              // Make description of track
	double Friction(int Index) const;            // Friction of section[Index]
	double ForwardAngle(double TrackPos) const;  // Angle

	int IndexFromPos                             // Get sections index from pos
	  (double TrackPos) const;
	void InitTrack                               // Initialize Track
	  (tTrack* Track, TCarParam& CarParam, float SmoothSide, PitSideMod* PitSideMod = 0);
	double MeanSectionLen() const;               // Mean length of sections
	TVec2d Normale(double TrackPos) const;        // To right
	double NormalizePos(double TrackPos) const;  // Keep pos in 0..Tracklength

	double Length() const;                       // Length of track in m

	const TSection&	operator[](int Index) const; // section of index
	const TSection&	Section(int Index) const;    // section of index
    void SmoothSides(double Delta);              // Smooth width changes
	double LearnFriction
      (int Index, double Delta, double MinFriction) const;

	tTrack* Track();                             // Get TORCS track data
	const tTrack* Track() const;                 // TORCS track data as const
	double Width() const;                        // Const track width

  private:
    void BuildPos2SecIndex();
    int NbrOfSections
	  (double Len, bool PitSection);             // Estimate nbr of sections
	void NormalizeDir                            // Calc Center and ToRight
	  (const tTrackSeg* pSeg, double toStart,
      double& T, TVec3d& Pt, TVec3d& Norm) const;

  private:
	int	oCount;                                  // Number of sections in track
	double oMeanSectionLen;                      // Mean length of sections
	TSection* oSections;                         // Array of sections
	tTrack*	oTrack;                              // TORCS data of track
    double oTrackRes;                            // sampling rate (per m)
    int oPitEntry;                               // Pit entry
    int oPitEntryNext;                           // Pit entry next
    int oPitExitPrev;                            // Pit exit prev
    int oPitExit;                                // Pit exit
    int oPitSide;                                // Pit side
	PitSideMod oPitSideMod;                      // Pit side mode
};
//==========================================================================*
#endif // _UNITTRACK_H_
//--------------------------------------------------------------------------*
// end of file unittrack.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*