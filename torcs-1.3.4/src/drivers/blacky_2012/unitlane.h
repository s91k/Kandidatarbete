//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitlane.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Fahrspur
//
// Datei    : unitlane.h
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
// dem Roboter mouse_2006
//
//    Copyright: (C) 2006-2007 Tim Foden
//
// dem Roboter wdbee_2007
//
//    Copyright: (C) 2006-2007 Wolf-Dieter Beelitz
//    eMail    : wdb@wdbee.de
//
// und dem Roboter delphin
//
//    Copyright: (C) 2006-2007 Wolf-Dieter Beelitz
//    eMail    : wdb@wdbee.de
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
#ifndef _UNITLANE_H_
#define _UNITLANE_H_

#include "unittrack.h"
#include "unitcarparam.h"
#include "unitfixcarparam.h"

//==========================================================================*
// Class TLane
//--------------------------------------------------------------------------*
class TLane
{
  public:
	struct TParamSeg
	{
		double DistFromStart;
		double Length;
		double BorderLeft;
		double BorderRight;
		double DeltaLeft;
		double DeltaRight;
		double Factor;
		double Apex;
		double ScaleFriction;
		double ScaleBraking;
		double ScaleBumps;
		double TargetSpeed;
	};

	struct TPathPt
	{
		const TSection*	Sec;		             // Track sec that contains this Point
		TVec3d Center;                           // Lane specific center
		double Crv;	 		                     // Curvature in xy
		double CrvZ;			                 // Curvature in z direction... e.g. bumps
		double Offset;                           // Offset from centre point
		TVec3d Point;                            // Actual point (same as CalcPt())
		double MaxSpeed;                         // Max speed through this point
		double Speed;                            // Speed through this point (braking only)
		double AccSpd;                           // Speed through this point, with modelled accel
		double FlyHeight;                        // Predicted height of car above track (flying)
		double BufL;		                     // Buffer from left for safety
		double BufR;		                     // Buffer from right for safety
		double NextCrv;                          // Cuvature comming next
		double WToL;                             // Lane specfic width to left
		double WToR;                             // Lane specfic width to right
		double WPitToL;                          // Lane specfic width to left
		double WPitToR;                          // Lane specfic width to right
		double Delta;
		double Factor;
		double Apex;
		double ScaleFriction;
		double ScaleBraking;
		double ScaleBumps;
		double TargetSpeed;
		double SpeedFrictionFactor;
		double BrakeFrictionFactor;
		double LastSpeedFrictionFactor;
		double LastBrakeFrictionFactor;
		bool Fix;

		double Dist() const {return Sec->DistFromStart;}
		double WtoL() const {return WToL;}
		double WtoR() const {return WToR;}
		const TVec3d& Pt() const {return Center;}
		const TVec3d& Norm() const {return Sec->ToRight;}
		TVec3d CalcPt() const {return Center + Sec->ToRight * Offset;}
	};

  private:
    int oLastIndex;

  public:
	int oCount;
	int oCountSeg;

	TLane();
	virtual ~TLane();

	virtual TLane& operator= (const TLane& Lane);

	virtual bool ContainsPos
	  (double TrackPos) const;
	virtual bool GetLanePoint
	  (double TrackPos, TLanePoint& LanePoint) const;
    virtual TVec2d GetLaneTarget
	  (double TrackPos) const;
    virtual TVec2d GetAvoidLaneTarget
	  (double AvoidSide, double TrackPos) const;

	void SetLane(const TLane& Lane);
	void Initialise
	  (TTrackDescription* pTrack,
      const TFixCarParam& FixCarParam,
	  const TCarParam& CarParam,
	  double MaxLeft = FLT_MAX,
	  double MaxRight = FLT_MAX);

	const TPathPt& PathPoints(int Index) const;

	void CalcCurvaturesXY
	  (int Start, int Step = 1);
	void CalcCurvaturesZ
	  (int Start, int Step = 1);
	void CalcMaxSpeeds
	  (int Start, int Len, int Step = 1);
	void PropagateBreaking
	  (int Start, int Len, int Step = 1);
	void PropagatePitBreaking
	  (int Start, int Len, float PitStopPos, float ScaleMu);
	void PropagateAcceleration
	  (int Start, int Len, int Step = 1);

	void CalcCurvaturesXY
	  (int Step = 1);
	void CalcCurvaturesZ
	  (int Step = 1);
	void CalcMaxSpeeds
	  (int Step = 1);
	void PropagateBreaking
	  (int Step = 1);
	void PropagatePitBreaking
	  (float PitStopPos, float ScaleMu);
	void PropagateAcceleration
	  (int Step = 1);
	void CalcFwdAbsCrv
	  (int Range, int Step = 1);

	double CalcEstimatedTime
	  (int Start, int Len) const;
	double CalcEstimatedLapTime() const;
	void ReduceSpeed(double TrackPos, double Delta);

	void CreateParamSeg(int Count);
    TParamSeg* GetParamSeg(int Index);
	void UpdateInfo(int Index, int Side);


  public:
	TTrackDescription* oTrack;                   // TORCS track data
	TPathPt* oPathPoints;                        // Points in this lane
	TFixCarParam oFixCarParam;                   // Copy of car params
	TCarParam oCarParam;                         // Copy of car params
    TParamSeg* oParamSegs;                       // Parameter segments
	int LapsLearned;						     // Laps used for learning

    int Dummy;
};
//==========================================================================*
#endif // _UNITLANE_H_
//--------------------------------------------------------------------------*
// end of file unitlane.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
