//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitlane.cpp
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter für TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Fahrspur
//
// Datei    : unitlane.cpp
// Erstellt : 25.11.2007
// Stand    : 10.06.2012
// Copyright: © 2007-2012 Wolf-Dieter Beelitz
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
#include <robottools.h>

#include "unitglobal.h"
#include "unitcommon.h"

#include "unitdriver.h"
#include "unitlane.h"
#include "unitlinalg.h"
#include "unitvec3d.h"
#include "unittmpcarparam.h"

//==========================================================================*
// Default constructor
//--------------------------------------------------------------------------*
TLane::TLane():
  oLastIndex(0),
  oCount(0),
  oCountSeg(0),
  oTrack(NULL),
  LapsLearned(0),
  oPathPoints(NULL),
  oParamSegs(NULL)
{
}
//==========================================================================*

//==========================================================================*
// Destructor
//--------------------------------------------------------------------------*
TLane::~TLane()
{
  delete [] oParamSegs;
  delete [] oPathPoints;
}
//==========================================================================*

//==========================================================================*
// Set operator (Sets lane)
//--------------------------------------------------------------------------*
TLane& TLane::operator= (const TLane& Lane)
{
  SetLane(Lane);
  return *this;
}
//==========================================================================*

//==========================================================================*
// Set lane
//--------------------------------------------------------------------------*
void TLane::SetLane(const TLane& Lane)
{
  oTrack = Lane.oTrack;
  oFixCarParam = Lane.oFixCarParam;
  oCarParam = Lane.oCarParam;
  LapsLearned = Lane.LapsLearned;

  oCount = Lane.oCount;
  oCountSeg = Lane.oCountSeg;

  delete [] oPathPoints;
  oPathPoints = new TPathPt[oCount];
  memcpy(oPathPoints, Lane.oPathPoints, oCount * sizeof(*oPathPoints));

  delete [] oParamSegs;
  oParamSegs = new TParamSeg[oCountSeg];
  memcpy(oParamSegs, Lane.oParamSegs, oCountSeg * sizeof(*oParamSegs));
}
//==========================================================================*

//==========================================================================*
// Check wether position is in lane
//--------------------------------------------------------------------------*
bool TLane::ContainsPos(double TrackPos) const
{
  if (TrackPos > 0.0)
    return true;                                 // Allways true because
  else                                           // this lane type
    return true;                                 // contains all points
}
//==========================================================================*

//==========================================================================*
// Reduce Speed to the points nearest the given position
//--------------------------------------------------------------------------*
void TLane::ReduceSpeed(double TrackPos, double Delta)
{
  int Idx1 = oTrack->IndexFromPos(TrackPos);
  int Idx0 = (Idx1 - 1 + oCount) % oCount;
  int Idx2 = (Idx1 + 1) % oCount;

  oPathPoints[Idx0].Delta = MAX(Delta,oPathPoints[Idx0].Delta);
  oPathPoints[Idx1].Delta = MAX(Delta,oPathPoints[Idx1].Delta);
  oPathPoints[Idx2].Delta = MAX(Delta,oPathPoints[Idx1].Delta);
}
//==========================================================================*

//==========================================================================*
// Get the point nearest the given position
//--------------------------------------------------------------------------*
TVec2d TLane::GetLaneTarget(double TrackPos) const
{
  TVec2d Target;
  int Idx = oTrack->IndexFromPos(TrackPos);
  TVec3d P = oPathPoints[Idx].CalcPt();
  Target.x = P.x;
  Target.y = P.y;
  return Target;
}
//==========================================================================*

//==========================================================================*
// Get the point nearest the given position
//--------------------------------------------------------------------------*
TVec2d TLane::GetAvoidLaneTarget(double AvoidSide, double TrackPos) const
{
  TVec2d Target;
  int Idx = oTrack->IndexFromPos(TrackPos);
  TVec3d P = oPathPoints[Idx].CalcPt();
  if (AvoidSide > 0)
  {
	P = P + (AvoidSide * oPathPoints[Idx].WToL) * oPathPoints[Idx].Sec->ToRight;
	//GfOut("AVS>0\n");
  }
  else
  {
    P = P + (AvoidSide * oPathPoints[Idx].WToR) * oPathPoints[Idx].Sec->ToRight;
	//GfOut("AVS<0\n");
  }
  Target.x = P.x;
  Target.y = P.y;
  return Target;
}
//==========================================================================*

//==========================================================================*
// Get information to the point nearest the given position
//--------------------------------------------------------------------------*
bool TLane::GetLanePoint(double TrackPos, TLanePoint& LanePoint) const
{
  int Idx0 = oTrack->IndexFromPos(TrackPos);
  int Idxp = (Idx0 - 1 + oCount) % oCount;
  int Idx1 = (Idx0 + 1) % oCount;
  int Idx2 = (Idx0 + 2) % oCount;

  double Dist0 = oPathPoints[Idx0].Dist();
  double Dist1 = oPathPoints[Idx1].Dist();
  if (Idx1 == 0)
    Dist1 = oTrack->Length();

  TVec3d P0 = oPathPoints[Idxp].CalcPt();
  TVec3d P1 = oPathPoints[Idx0].CalcPt();
  TVec3d P2 = oPathPoints[Idx1].CalcPt();
  TVec3d P3 = oPathPoints[Idx2].CalcPt();

  double Crv1 = TUtils::CalcCurvatureXY(P0, P1, P2);
  double Crv2 = TUtils::CalcCurvatureXY(P1, P2, P3);
  double Crvz1 = TUtils::CalcCurvatureZ(P0, P1, P2);
  double Crvz2 = TUtils::CalcCurvatureZ(P1, P2, P3);

  double Tx = (TrackPos - Dist0) / (Dist1 - Dist0);

  LanePoint.Index = Idx0;
  LanePoint.Crv = (1.0 - Tx) * Crv1 + Tx * Crv2;
  LanePoint.Crvz = (1.0 - Tx) * Crvz1 + Tx * Crvz2;
  LanePoint.T = Tx;
  LanePoint.Offset =
	(oPathPoints[Idx0].Offset)
	+ Tx * (oPathPoints[Idx1].Offset - oPathPoints[Idx0].Offset);

  double Ang0 = TUtils::VecAngXY(oPathPoints[Idx1].CalcPt() -
	oPathPoints[Idx0].CalcPt());
  double Ang1 = TUtils::VecAngXY(oPathPoints[Idx2].CalcPt() -
	oPathPoints[Idx1].CalcPt());

  double DeltaAng = Ang1 - Ang0;
  DOUBLE_NORM_PI_PI(DeltaAng);
  LanePoint.Angle = Ang0 + LanePoint.T * DeltaAng;

  TVec2d Tang1, Tang2;
  TUtils::CalcTangent(P0.GetXY(), P1.GetXY(), P2.GetXY(), Tang1);
  TUtils::CalcTangent(P1.GetXY(), P2.GetXY(), P3.GetXY(), Tang2);
  TVec2d Dir = TUtils::VecUnit(Tang1) * (1 - Tx) + TUtils::VecUnit(Tang2) * Tx;

  Ang0 = TUtils::VecAngle(Tang1);
  Ang1 = TUtils::VecAngle(Tang2);
  DeltaAng = Ang1 - Ang0;
  DOUBLE_NORM_PI_PI(DeltaAng);

  LanePoint.Speed = oPathPoints[LanePoint.Index].Speed + (oPathPoints[Idx1].Speed
	- oPathPoints[LanePoint.Index].Speed) * LanePoint.T;
  LanePoint.Delta = oPathPoints[LanePoint.Index].Delta + (oPathPoints[Idx1].Delta
	- oPathPoints[LanePoint.Index].Delta) * LanePoint.T;
  LanePoint.AccSpd = oPathPoints[LanePoint.Index].AccSpd + (oPathPoints[Idx1].AccSpd
	- oPathPoints[LanePoint.Index].AccSpd) * LanePoint.T;

  return true;
}
//==========================================================================*

//==========================================================================*
// Initialize lane from track limiting width to left and right
//--------------------------------------------------------------------------*
void TLane::Initialise
  (TTrackDescription* Track,
  const TFixCarParam& FixCarParam,
  const TCarParam& CarParam,
  double MaxLeft, double MaxRight)
{
  delete [] oPathPoints;
  oTrack = Track;
  oCount = oTrack->Count();
  oPathPoints = new TPathPt[oCount];
  oCarParam = CarParam;                          // Copy car params
  oFixCarParam = FixCarParam;                    // Copy car params
  oLastIndex = 0;                                // initialize to start

  if (MaxLeft < 999.0) 
  { // Right lane
    for (int I = 0; I < oCount; I++)
    {
	  const TSection& Sec = (*oTrack)[I];
	  oPathPoints[I].Sec = &Sec;
	  oPathPoints[I].Center = Sec.Center;
	  oPathPoints[I].Crv = 0;
	  oPathPoints[I].CrvZ	= 0;
	  oPathPoints[I].Offset = 0.0;
	  oPathPoints[I].Point = oPathPoints[I].CalcPt();
	  oPathPoints[I].MaxSpeed	= 10;
	  oPathPoints[I].Speed = 10;
	  oPathPoints[I].AccSpd = 10;
	  oPathPoints[I].FlyHeight = 0;
	  oPathPoints[I].BufL	= 0;
	  oPathPoints[I].BufR	= 0;
	  oPathPoints[I].NextCrv = 0.0;
	  oPathPoints[I].WToL = MaxLeft;
      oPathPoints[I].WToR = Sec.WidthToRight;
	  oPathPoints[I].WPitToL = Sec.PitWidthToLeft;
      oPathPoints[I].WPitToR = Sec.PitWidthToRight;
	  oPathPoints[I].Delta = 0.0;
	  oPathPoints[I].Factor = TDriver::RLFactor;
	  oPathPoints[I].Apex = TDriver::Apex;
      oPathPoints[I].ScaleFriction = 1.0;
      oPathPoints[I].ScaleBraking = 1.0;
      oPathPoints[I].ScaleBumps = 1.0;
      oPathPoints[I].TargetSpeed = 100.0;
	  oPathPoints[I].SpeedFrictionFactor = 1.0;
	  oPathPoints[I].BrakeFrictionFactor = 1.0;
	  oPathPoints[I].LastSpeedFrictionFactor = 1.0;
	  oPathPoints[I].LastBrakeFrictionFactor = 1.0;
	  oPathPoints[I].Fix = false;
	  UpdateInfo(I,1);
    }
    oPathPoints[0].WToL = oPathPoints[1].WToL;
    oPathPoints[0].WToR = oPathPoints[1].WToR;;
  }
  else if (MaxRight < 999.0)
  {
    for (int I = 0; I < oCount; I++)
    { // Left lane
	  const TSection& Sec = (*oTrack)[I];
	  oPathPoints[I].Sec = &Sec;
	  oPathPoints[I].Center	= Sec.Center;
	  oPathPoints[I].Crv = 0;
	  oPathPoints[I].CrvZ = 0;
	  oPathPoints[I].Offset = 0.0;
	  oPathPoints[I].Point = oPathPoints[I].CalcPt();
	  oPathPoints[I].MaxSpeed = 10;
	  oPathPoints[I].Speed = 10;
	  oPathPoints[I].AccSpd	= 10;
	  oPathPoints[I].FlyHeight = 0;
	  oPathPoints[I].BufL = 0;
	  oPathPoints[I].BufR = 0;
	  oPathPoints[I].NextCrv = 0.0;
      oPathPoints[I].WToL = Sec.WidthToLeft;
	  oPathPoints[I].WToR = MaxRight;
      oPathPoints[I].WPitToL = Sec.PitWidthToLeft;
      oPathPoints[I].WPitToR = Sec.PitWidthToRight;
	  oPathPoints[I].Delta = 0.0;
	  oPathPoints[I].Factor = TDriver::RLFactor;
	  oPathPoints[I].Apex = TDriver::Apex;
      oPathPoints[I].ScaleFriction = 1.0;
      oPathPoints[I].ScaleBraking = 1.0;
      oPathPoints[I].ScaleBumps = 1.0;
      oPathPoints[I].TargetSpeed = 100.0;
	  oPathPoints[I].SpeedFrictionFactor = 1.0;
	  oPathPoints[I].BrakeFrictionFactor = 1.0;
	  oPathPoints[I].LastSpeedFrictionFactor = 1.0;
	  oPathPoints[I].LastBrakeFrictionFactor = 1.0;
	  oPathPoints[I].Fix = false;
	  UpdateInfo(I,-1);
    }
    oPathPoints[0].WToL = oPathPoints[1].WToL;
    oPathPoints[0].WToR = oPathPoints[1].WToR;;
  }
  else
  {
    for (int I = 0; I < oCount; I++)
    { // Main lane
	  const TSection& Sec = (*oTrack)[I];
	  oPathPoints[I].Sec = &Sec;
	  oPathPoints[I].Center = Sec.Center;
	  oPathPoints[I].Crv = 0;
	  oPathPoints[I].CrvZ	= 0;
	  oPathPoints[I].Offset = 0.0;
	  oPathPoints[I].Point = oPathPoints[I].CalcPt();
	  oPathPoints[I].MaxSpeed	= 10;
	  oPathPoints[I].Speed = 10;
	  oPathPoints[I].AccSpd = 10;
	  oPathPoints[I].FlyHeight = 0;
	  oPathPoints[I].BufL	= 0;
	  oPathPoints[I].BufR	= 0;
	  oPathPoints[I].NextCrv = 0.0;
	  oPathPoints[I].WToL = Sec.WidthToLeft;
      oPathPoints[I].WToR = Sec.WidthToRight;
      oPathPoints[I].WPitToL = Sec.PitWidthToLeft;
      oPathPoints[I].WPitToR = Sec.PitWidthToRight;
	  oPathPoints[I].Delta = 0.0;
	  oPathPoints[I].Factor = TDriver::RLFactor;
	  oPathPoints[I].Apex = TDriver::Apex;
      oPathPoints[I].ScaleFriction = 1.0;
      oPathPoints[I].ScaleBraking = 1.0;
      oPathPoints[I].ScaleBumps = 1.0;
      oPathPoints[I].TargetSpeed = 100.0;
	  oPathPoints[I].SpeedFrictionFactor = 1.0;
	  oPathPoints[I].BrakeFrictionFactor = 1.0;
	  oPathPoints[I].LastSpeedFrictionFactor = 1.0;
	  oPathPoints[I].LastBrakeFrictionFactor = 1.0;
	  oPathPoints[I].Fix = false;
	  UpdateInfo(I,0);
    }
    oPathPoints[0].WToL = oPathPoints[1].WToL;
    oPathPoints[0].WToR = oPathPoints[1].WToR;;
  }
  CalcCurvaturesXY();
  CalcCurvaturesZ();
}
//==========================================================================*

//==========================================================================*
// Get path point from index
//--------------------------------------------------------------------------*
const TLane::TPathPt& TLane::PathPoints(int Index) const
{
  return oPathPoints[Index];
}
//==========================================================================*

//==========================================================================*
// Calc curvature in XY
//--------------------------------------------------------------------------*
void TLane::CalcCurvaturesXY(int Start, int Step)
{
  for (int I = 0; I < oCount; I++)
  {
	int	P  = (Start + I) % oCount;               // Point
	int	Pp = (P - Step + oCount) % oCount;       // Prev Point
	int	Pn = (P + Step) % oCount;                // Next Point

	oPathPoints[P].Crv =
	  TUtils::CalcCurvatureXY(
	    oPathPoints[Pp].CalcPt(),
	    oPathPoints[P].CalcPt(),
 	    oPathPoints[Pn].CalcPt());
  }

  // Overwrite values at start to avoid slowdown caused by track errors
  for (int I = 0; I <= Step; I++)
  {
    oPathPoints[I].Crv = 0.0;
    oPathPoints[oCount-1-I].Crv = 0.0;
  }
}
//==========================================================================*

//==========================================================================*
// Calc curvature in Z
//--------------------------------------------------------------------------*
void TLane::CalcCurvaturesZ(int Start, int Step)
{
  Step *= 3;

  for (int I = 0; I < oCount; I++)
  {
	int	P  = (Start + I) % oCount;                    // Point
	int	Pp = (P - Step + oCount) % oCount;                 // Prev Point
	int	Pn = (P + Step) % oCount;                     // Next Point

	oPathPoints[P].CrvZ = 6 * TUtils::CalcCurvatureZ(
	  oPathPoints[Pp].CalcPt(),
      oPathPoints[P].CalcPt(),
	  oPathPoints[Pn].CalcPt());
  }

  // Overwrite values at start to avoid slowdown caused by track errors
  for (int I = 0; I <= Step; I++)
  {
    oPathPoints[I].CrvZ = 0.0;
    oPathPoints[oCount-1-I].CrvZ = 0.0;
  }
}
//==========================================================================*

//==========================================================================*
// Calc max possible speed depending on car modell
//--------------------------------------------------------------------------*
void TLane::CalcMaxSpeeds
  (int Start, int Len, int Step)
{
  //double SpeedCmp = 0.0;
  //int Counter = 0;

  for (int I = 0; I < Len; I += Step)
  {
	int P = (Start + I) % oCount;
	int Q = (P + 1) % oCount;
	int R = (P + 30) % oCount;

	double TrackRollAngle = atan2(oPathPoints[P].Norm().z, 1);
	const TVec3d* V = &oPathPoints[P].Norm();
	const TVec3d* W = &oPathPoints[R].Norm();
	float TrackCrvAngle = V->CosPhi(V,W);
	TVec3d Delta = oPathPoints[P].Point - oPathPoints[Q].Point;
	double Distance = Length2D(Delta);
	double TrackTiltAngle = atan2(Delta.z, Distance);
	double ScaleBumps = 1.0;
	double ScaleSpeed = 1.0;
	double Speed = oFixCarParam.EstimateSpeed(
		oPathPoints[P].Crv,							
		oPathPoints[P].CrvZ,							
		oTrack->Friction(P) * oPathPoints[P].ScaleFriction,						
		TrackRollAngle,					
		TrackTiltAngle,
		oPathPoints[P].SpeedFrictionFactor,
		oPathPoints[P].ScaleBumps);
/**/
	if (TrackCrvAngle > TDriver::MaxCrvAngle) 
	{
		Speed = MIN(100.0,Speed *= 2.0);
    }
/**/
	if (Speed < oPathPoints[P].TargetSpeed)
		Speed = oPathPoints[P].TargetSpeed;

	oPathPoints[P].MaxSpeed = Speed;
	oPathPoints[P].Speed = Speed;
	oPathPoints[P].AccSpd = Speed;
  }
  //GfOut("\n\n\nN: %d S: %f\n\n\n",Counter,SpeedCmp/Counter);
}
//==========================================================================*

//==========================================================================*
// Propagate braking
//--------------------------------------------------------------------------*
void TLane::PropagateBreaking
  (int Start, int Len, int Step)
{
  oFixCarParam.oXXX = 1.0;

  for (int I = Step * ((2*Len - 1) / Step); I >= 0; I -= Step )
  {
	int	P = (Start + I) % oCount;
	int Q = (P + Step) % oCount;

	if (oPathPoints[P].Speed > oPathPoints[Q].Speed)
	{
	  // see if we need to adjust spd[i] to make it possible
	  //   to slow to spd[j] by the next seg.
      TVec3d Delta = oPathPoints[P].CalcPt() - oPathPoints[Q].CalcPt();
      double Dist = TUtils::VecLenXY(Delta);
      double K = (oPathPoints[P].Crv + oPathPoints[Q].Crv) * 0.5;
	  if (fabs(K) > 0.0001)
	    Dist = 2 * asin(0.5 * Dist * K) / K;
	  double TrackRollAngle = atan2(oPathPoints[P].Norm().z, 1);
	  double TrackTiltAngle = atan2(Delta.z, Dist);
	  double U = oFixCarParam.EstimateBraking(
		oPathPoints[P].Crv,							// Curvature in xy at P0
		oPathPoints[P].CrvZ,						// Curvature in z at P0
		oPathPoints[Q].Crv,							// Curvature in xy at P1
		oPathPoints[Q].CrvZ,						// Curvature in z at P0
		oPathPoints[Q].Speed,						// Speed
		Dist,										// Distance P0 P1
		oTrack->Friction(P),						// Friction
		TrackRollAngle,								// Track roll angle
		TrackTiltAngle,  					        // Track tilt angle
		oPathPoints[P].BrakeFrictionFactor,
		oPathPoints[P].ScaleBumps);

	  if (oPathPoints[P].Speed > U)
		oPathPoints[P].Speed = oPathPoints[P].AccSpd = U;

	  if (oPathPoints[P].FlyHeight > 0.1)
		oPathPoints[P].Speed = oPathPoints[Q].Speed;

	}
  }
}
//==========================================================================*

//==========================================================================*
// Propagate braking
//--------------------------------------------------------------------------*
void TLane::PropagatePitBreaking
  (int Start, int Len, float PitStopPos, float ScaleMu)
{
  //const float base = 0.5f;
  int Step = 1;

  for (int I = Step * ((2*Len - 1) / Step); I >= 0; I -= Step )
  {
	int	P = (Start + I) % oCount;
	int Q = (P + Step) % oCount;

	if (oPathPoints[P].Speed > oPathPoints[Q].Speed)
	{
	  // see if we need to adjust spd[i] to make it possible
	  //   to slow to spd[j] by the next seg.
      TVec3d Delta = oPathPoints[P].CalcPt() - oPathPoints[Q].CalcPt();
      double Dist = TUtils::VecLenXY(Delta);
      double K = (oPathPoints[P].Crv + oPathPoints[Q].Crv) * 0.5;
	  if (fabs(K) > 0.0001)
	    Dist = 2 * asin(0.5 * Dist * K) / K;
	  double TrackRollAngle = atan2(oPathPoints[P].Norm().z, 1);
	  double TrackTiltAngle = atan2(Delta.z, Dist);

	  double Factor = 1.0 - MIN(1.0,fabs(oPathPoints[Q].Dist() - PitStopPos) / oFixCarParam.oPitBrakeDist);
	  double Friction = oTrack->Friction(P) * (Factor * ScaleMu + (1 - Factor) * oCarParam.oScaleBrakePit);

	  double U = oFixCarParam.CalcBraking(
        oCarParam,
  		oPathPoints[P].Crv,
		oPathPoints[P].CrvZ,
		oPathPoints[Q].Crv,
		oPathPoints[Q].CrvZ,
		oPathPoints[Q].FlyHeight,
		oPathPoints[Q].Speed,
		Dist,
		Friction,
		TrackRollAngle,
		TrackTiltAngle,
		oPathPoints[P].ScaleBraking);

	  if (oPathPoints[P].Speed > U)
		oPathPoints[P].Speed = oPathPoints[P].AccSpd = U;

	  //GfOut("I:%d P:%d Q:%d ID:%d F:%g U:%g S:%g\n",I,P,Q,ID,Factor,U,oPathPoints[P].Speed);

	  if (oPathPoints[P].FlyHeight > 0.1)
		oPathPoints[P].Speed = oPathPoints[Q].Speed;
	}
  }
}
//==========================================================================*

//==========================================================================*
// Propagate acceleration
//--------------------------------------------------------------------------*
void TLane::PropagateAcceleration
  (int Start, int Len, int Step)
{
  for (int I = 0; I < 2*Len; I += Step )
  {
	int Q = (Start + I + oCount) % oCount;
	int	P = (Q - Step + oCount) % oCount;

	if (Q == 0)
	  P = (oCount - 3);

	if (oPathPoints[P].AccSpd < oPathPoints[Q].AccSpd)
	{
	  // see if we need to adjust spd[Q] to make it possible
	  //   to speed up to spd[P] from spd[Q].
  	  double Dist = TUtils::VecLenXY(
		oPathPoints[P].CalcPt() - oPathPoints[Q].CalcPt());

	  double K = (oPathPoints[P].Crv + oPathPoints[Q].Crv) * 0.5;
	  if (fabs(K) > 0.0001)
	    Dist = 2 * asin(0.5 * Dist * K) / K;

	  double TrackRollAngle = atan2(oPathPoints[P].Norm().z, 1);

	  double V = oFixCarParam.CalcAcceleration(
	    oPathPoints[P].Crv,
		oPathPoints[P].CrvZ,
		oPathPoints[Q].Crv,
		oPathPoints[Q].CrvZ,
		oPathPoints[P].AccSpd,
		Dist,
		oTrack->Friction(P),
		TrackRollAngle);

		//if (oPathPoints[Q].AccSpd > V)
		oPathPoints[Q].AccSpd = MIN(V,oPathPoints[Q].Speed);
	}
  }
}
//==========================================================================*

//==========================================================================*
// Calculate curvature in XY
//--------------------------------------------------------------------------*
void TLane::CalcCurvaturesXY(int Step)
{
  CalcCurvaturesXY(0, Step);
}
//==========================================================================*

//==========================================================================*
// Calculate curvature in Z
//--------------------------------------------------------------------------*
void TLane::CalcCurvaturesZ(int Step)
{
  CalcCurvaturesZ( 0, Step);
}
//==========================================================================*

//==========================================================================*
// Calculate max possible speed
//--------------------------------------------------------------------------*
void TLane::CalcMaxSpeeds(int Step)
{
  CalcMaxSpeeds(0, oCount, Step);
}
//==========================================================================*

//==========================================================================*
// Propagate breaking
//--------------------------------------------------------------------------*
void TLane::PropagateBreaking
  (int Step)
{
  PropagateBreaking( 0, oCount, Step);
}
//==========================================================================*

//==========================================================================*
// Propagate breaking
//--------------------------------------------------------------------------*
void TLane::PropagatePitBreaking
  (float PitStopPos, float ScaleMu)
{
  PropagatePitBreaking( 0, oCount, PitStopPos, ScaleMu);
}
//==========================================================================*

//==========================================================================*
// Propagate acceleration
//--------------------------------------------------------------------------*
void TLane::PropagateAcceleration
  (int Step)
{
  PropagateAcceleration( 0, oCount, Step);
}
//==========================================================================*

//==========================================================================*
// Calculate forward absolute curvature
//--------------------------------------------------------------------------*
void TLane::CalcFwdAbsCrv(int Range, int Step)
{
  const int	N = oCount - 1;

  int Count = Range / Step;
  int P = Count * Step;
  int Q = P;
  double TotalCrv = 0;

  while (P > 0)
  {
	TotalCrv += oPathPoints[P].Crv;
	P -= Step;
  }

  oPathPoints[0].NextCrv = TotalCrv / Count;
  TotalCrv += fabs(oPathPoints[0].Crv);
  TotalCrv -= fabs(oPathPoints[Q].Crv);

  P = (N / Step) * Step;
  Q -= Step;
  if (Q < 0)
    Q = (N / Step) * Step;

  while (P > 0)
  {
	oPathPoints[P].NextCrv = TotalCrv / Count;
	TotalCrv += fabs(oPathPoints[P].Crv);
	TotalCrv -= fabs(oPathPoints[Q].Crv);

	P -= Step;
	Q -= Step;
	if (Q < 0)
	  Q = (N / Step) * Step;
  }
}
//==========================================================================*

//==========================================================================*
// Calculate estimated time
//--------------------------------------------------------------------------*
double TLane::CalcEstimatedTime(int Start, int Len) const
{
  double TotalTime = 0;

  for (int I = 0; I < Len; I++)
  {
	int	P = (Start + I) % oCount;
	int	Q = (P + 1) % oCount;
	double Dist = TUtils::VecLenXY(
	  oPathPoints[P].CalcPt() - oPathPoints[Q].CalcPt());

	TotalTime += Dist / ((oPathPoints[P].AccSpd + oPathPoints[Q].AccSpd) * 0.5);
  }

  return TotalTime;
}
//==========================================================================*

//==========================================================================*
// Calculate estimated lap time
//--------------------------------------------------------------------------*
double TLane::CalcEstimatedLapTime() const
{
  double LapTime = 0;

  for (int I = 0; I < oCount; I++)
  {
	int	Q = (I + 1) % oCount;
	double Dist = TUtils::VecLenXY(
	  oPathPoints[I].CalcPt() - oPathPoints[Q].CalcPt());
	LapTime += Dist / ((oPathPoints[I].AccSpd + oPathPoints[Q].AccSpd) * 0.5);
  }

  return LapTime;
}
//==========================================================================*

//==========================================================================*
// Create Parameter Segments
//--------------------------------------------------------------------------*
void TLane::CreateParamSeg(int Count)
{
  oCountSeg = Count;
  oParamSegs = new TParamSeg[Count];
}
//==========================================================================*

//==========================================================================*
// Get Parameter Segment
//--------------------------------------------------------------------------*
TLane::TParamSeg* TLane::GetParamSeg(int Index)
{
  return &oParamSegs[Index];
}
//==========================================================================*

//==========================================================================*
// Update track info
//--------------------------------------------------------------------------*
void TLane::UpdateInfo(int Index, int Side)
{
  if (oLastIndex > oCountSeg)
    return;

  TParamSeg* ParSeg = &oParamSegs[oLastIndex];
  double DistFromStart = oPathPoints[Index].Sec->DistFromStart;

  if (DistFromStart < ParSeg->DistFromStart)
	return;

  while (DistFromStart > ParSeg->DistFromStart + ParSeg->Length)
  {
	if (++oLastIndex > oCountSeg)
	  return;
    ParSeg = &oParamSegs[oLastIndex];
  }
  double T = (DistFromStart - ParSeg->DistFromStart)/ParSeg->Length;
  if (Side == -1)
    oPathPoints[Index].WToL += ParSeg->BorderLeft + T * ParSeg->DeltaLeft;
  else if (Side == 1)
    oPathPoints[Index].WToR += ParSeg->BorderRight + T * ParSeg->DeltaRight;
  else
  {
    oPathPoints[Index].WToL += ParSeg->BorderLeft + T * ParSeg->DeltaLeft;
    oPathPoints[Index].WToR += ParSeg->BorderRight + T * ParSeg->DeltaRight;
  }
  oPathPoints[Index].Factor = ParSeg->Factor;
  oPathPoints[Index].Apex = ParSeg->Apex;
  oPathPoints[Index].ScaleFriction = ParSeg->ScaleFriction;
  oPathPoints[Index].ScaleBraking = ParSeg->ScaleBraking;
  oPathPoints[Index].ScaleBumps = ParSeg->ScaleBumps;
  oPathPoints[Index].TargetSpeed = ParSeg->TargetSpeed;

}
//==========================================================================*

//--------------------------------------------------------------------------*
// end of file unitlane.cpp
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
