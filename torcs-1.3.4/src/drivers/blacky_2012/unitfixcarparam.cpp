//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitfixcarparam.cpp
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter für TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Unveränderliche Parameter des Fahrzeugs und Nebenrechnungen
//
// Datei    : unitfixcarparam.cpp
// Erstellt : 25.11.2007
// Stand    : 10.06.2012
// Copyright: © 2007-2012 Wolf-Dieter Beelitz
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
#include <math.h>

#include "unitglobal.h"
#include "unitcommon.h"

#include "unitparabel.h"
#include "unitfixcarparam.h"

#include "unitdriver.h"

//==========================================================================*
// Default constructor
//--------------------------------------------------------------------------*
TFixCarParam::TFixCarParam():
  oCar(NULL),
  oTmpCarParam(NULL),
  oBorderInner(0.0f),
  oBorderOuter(0.0f),
  oMaxBorderInner(0.31f),
  oBorderScale(120.0f),
  oCa(0),
  oCaFrontWing(0),
  oCaGroundEffect(0),
  oCaRearWing(0),
  oCdBody(0),
  oCdWing(0),
  oEmptyMass(0),
  oLength(4.5),
  oTyreMu(0),
  oTyreMuFront(0),
  oTyreMuRear(0),
  oWidth(2.0),
  oAeroModel(2.0),
  oPitBrakeDist(150.0f),
  oXXX(1.0)
{
}
//==========================================================================*

//==========================================================================*
// Destructor
//--------------------------------------------------------------------------*
TFixCarParam::~TFixCarParam()
{
}
//===========================================================================

//==========================================================================*
// Initialize
//--------------------------------------------------------------------------*
void TFixCarParam::Initialize(PtCarElt Car, PDriver Driver)
{
  oCar = Car;
  oDriver = Driver;
}
//==========================================================================*

//==========================================================================*
// Calculate accelleration
//--------------------------------------------------------------------------*
double TFixCarParam::CalcAcceleration(
  double Crv0,                                   // Curvature in xy at P0
  double Crvz0,                                  // Curvature in z at P0
  double Crv1,                                   // Curvature in xy at P1
  double Crvz1,                                  // Curvature in z at P0
  double Speed,                                  // Speed
  double Dist,                                   // Distance P0 P1
  double Friction,                               // Friction
  double TrackRollAngle) const                   // Track roll angle
{
  double MU = Friction * oTyreMu;
  double CD = oCdBody *
	(1.0 + oTmpCarParam->oDamage / 10000.0) + oCdWing;

//  double Crv = (Crv0  + Crv1) * 0.5;
//  double Crvz = -(Crvz0 + Crvz1) * 0.5;
  double Crv = (0.25*Crv0  + 0.75*Crv1);
  double Crvz = -(0.25*Crvz0 + 0.75*Crvz1);
  if (Crvz > 0)
    Crvz = 0;

  double Gdown = G * cos(TrackRollAngle);
  double Glat = G * sin(TrackRollAngle);
  double Gtan = 0;	// TODO: track pitch angle.

  double U = Speed;
  double V = U;

  TParabel AccFromSpd(PAR_A, PAR_B, PAR_C);      // approx. carX-trb1
  double OldV = 0.0;
  for (int Count = 0; Count < 10; Count++)
  {
    double AvgV = (U + V) * 0.5;
    double AvgV2 = AvgV * AvgV;

    double Fdown = oTmpCarParam->oMass * Gdown
	  + (oTmpCarParam->oMass * Crvz + oCa) * AvgV2;
    double Froad = Fdown * MU;
    double Flat = oTmpCarParam->oMass * Glat;
    double Ftan = oTmpCarParam->oMass * Gtan - CD * AvgV2;

    double Flatroad = fabs(oTmpCarParam->oMass * AvgV2 * Crv - Flat);
    if (Flatroad > Froad)
  	  Flatroad = Froad;

	double Ftanroad = sqrt(Froad * Froad - Flatroad * Flatroad) + Ftan;

	double Acc = Ftanroad / oTmpCarParam->oMass;
	double MaxAcc = MIN(11.5,AccFromSpd.CalcY(AvgV));
    if (Acc > MaxAcc)
	  Acc = MaxAcc;

	double Inner = MAX(0, U * U + 2 * Acc * Dist);
	V = sqrt(Inner);
	if (fabs(V - OldV) < 0.001)
	  break;
	OldV = V;
  }
  return V;
}
//==========================================================================*

//==========================================================================*
// Calculate decceleration
//--------------------------------------------------------------------------*
double	TFixCarParam::CalcBraking
  (TCarParam& CarParam,                          // Lane specific parameters
  double Crv0,                                   // Curvature in xy at P0
  double Crvz0,                                  // Curvature in z at P0
  double Crv1,                                   // Curvature in xy at P1
  double Crvz1,                                  // Curvature in z at P0
  double FlyHeight,
  double Speed,                                  // Speed
  double Dist,                                   // Distance P0 P1
  double Friction,                               // Friction
  double TrackRollAngle,                         // Track roll angle
  double TrackTiltAngle,                         // Track tilt angle
  double ScaleBraking)                           // Part of Track
{
  double Crv = (Crv0 + Crv1) * 0.5; 
  double Crvz = -(Crvz0 + Crvz1) * 0.5;
  if (Crvz > 0)
    Crvz = 0;

  if (fabs(Crv) > 1/20.0)
	oXXX = 0.85;
  else
	oXXX = MIN(1.0,oXXX+0.003);

  if (fabs(Crv) > 0.02)
    Friction *= 0.6;         // Wheel-2
  else if (fabs(Crv) > 0.01)
    Friction *= 0.75;
  else if (fabs(Crv) > 0.005)
    Friction *= 0.85;
  else if (fabs(Crv) > 0.002)
    Friction *= 0.9;
  else 
    Friction *= 0.95;

  Friction *= oXXX;

  double Mu = Friction * oTyreMu;
  double Mu_F = Mu;
  double Mu_R = Mu;

  if (oAeroModel == 1)
  {
    Mu_F = Friction * oTyreMuFront;
	Mu_R = Friction * oTyreMuRear;
	Mu = (Mu_F + Mu_R) / 2;
  }
  else if (oAeroModel == 2)
  {
    Mu_F = Friction * oTyreMuFront;
	Mu_R = Friction * oTyreMuRear;
	Mu = MIN(Mu_F,Mu_R);
  }

  double Cd = 0.0;
  // From TORCS:
  //double Cd = oCdBody *
  //  (1.0 + oTmpCarParam->oDamage / 10000.0) + oCdWing;

  TrackTiltAngle *= 1.1;
  //TrackTiltAngle *= 0.0;
  //GfOut("Crv: %f Roll: %f  Tilt: %f\n",Crv,sin(TrackRollAngle),cos(TrackTiltAngle));
  double Gdown = G * cos(TrackRollAngle) * cos(TrackTiltAngle);
  double Glat  = fabs(G * sin(TrackRollAngle));
  double Gtan  = G * sin(TrackTiltAngle);

  double V = Speed;
  double U = V;

  double Factor = MAX(0.0,MIN(1.0,1.0 - FlyHeight / 0.2));
  //double Factor = 1.0;
  Factor *= CarParam.oScaleBrake * ScaleBraking;

  for (int I = 0; I < 10; I++)
  {
	double AvgV = (U + V) * 0.5;
	double AvgV2 = AvgV * AvgV;

	double Froad;
	if (oAeroModel > 0)
	{
	  double Fdown = oTmpCarParam->oMass * Gdown + (oTmpCarParam->oMass * Crvz + oCaGroundEffect) * AvgV2;
	  double Ffrnt = oCaFrontWing * AvgV2;
	  double Frear = oCaRearWing * AvgV2;

	  Froad = 0.95 * Fdown * Mu + Ffrnt * Mu_F + Frear * Mu_R;
	}
	else
	{
	  double Fdown = oTmpCarParam->oMass * Gdown + (oTmpCarParam->oMass * Crvz + oCa) * AvgV2;

	  Froad = Fdown * Mu;
	}

	double Flat  = oTmpCarParam->oMass * Glat;
	double Ftan  = oTmpCarParam->oMass * Gtan - Cd * AvgV2;

	double Flatroad = MAX(0.0,oTmpCarParam->oMass * AvgV2 * fabs(Crv) - Flat);
	if (Flatroad > Froad)
	  Flatroad = Froad;

	double Ftanroad = -sqrt(Froad * Froad - Flatroad * Flatroad) + Ftan;

	double Acc = Factor * Ftanroad / oTmpCarParam->oMass;

	double Inner = MAX(0, V * V - 2 * Acc * Dist);
	double OldU = U;
	U = sqrt(Inner);
	if (fabs(U - OldU) < 0.001)
	  break;
  }

  return U;
}
//==========================================================================*

//==========================================================================*
// Calculate decceleration in pitlane
//--------------------------------------------------------------------------*
double	TFixCarParam::CalcBrakingPit
  (TCarParam& CarParam,                          // Lane specific parameters
  double Crv0,                                   // Curvature in xy at P0
  double Crvz0,                                  // Curvature in z at P0
  double Crv1,                                   // Curvature in xy at P1
  double Crvz1,                                  // Curvature in z at P0
  double FlyHeight,
  double Speed,                                  // Speed
  double Dist,                                   // Distance P0 P1
  double Friction,                               // Friction
  double TrackRollAngle                          // Track roll angle
  /* int ID)*/) const                           // Part of Track
{
  double Crv = (Crv0 + Crv1) * 0.5; 
  double Crvz = -(Crvz0 + Crvz1) * 0.5;
  if (Crvz > 0)
    Crvz = 0;

  if (fabs(Crv) > 0.02)
    Friction *= 0.6;         // Wheel-2
  else if (fabs(Crv) > 0.01)
    Friction *= 0.75;
  else if (fabs(Crv) > 0.005)
    Friction *= 0.85;
  else if (fabs(Crv) > 0.002)
    Friction *= 0.9;
  else 
    Friction *= 0.95;

  double Mu = Friction * oTyreMu;
  double Mu_F = Mu;
  double Mu_R = Mu;

  if (oAeroModel == 1)
  {
    Mu_F = Friction * oTyreMuFront;
	Mu_R = Friction * oTyreMuRear;
	Mu = (Mu_F + Mu_R) / 2;
  }
  else if (oAeroModel == 2)
  {
    Mu_F = Friction * oTyreMuFront;
	Mu_R = Friction * oTyreMuRear;
	Mu = MIN(Mu_F,Mu_R);
  }

  // double Cd = 0.0;
  // From TORCS:
  double Cd = oCdBody *
    (1.0 + oTmpCarParam->oDamage / 10000.0) + oCdWing;

  double Gdown = G * cos(TrackRollAngle);
  double Glat  = G * sin(TrackRollAngle);
  double Gtan  = 0;

  double V = Speed;
  double U = V;

  for (int I = 0; I < 10; I++)
  {
	double AvgV = (U + V) * 0.5;
	double AvgV2 = AvgV * AvgV;

	double Froad;
	if (oAeroModel > 0)
	{
	  double Fdown = oTmpCarParam->oMass * Gdown + (oTmpCarParam->oMass * Crvz + oCaGroundEffect) * AvgV2;
	  double Ffrnt = oCaFrontWing * AvgV2;
	  double Frear = oCaRearWing * AvgV2;

	  Froad = Fdown * Mu + Ffrnt * Mu_F + Frear * Mu_R;
	}
	else
	{
	  double Fdown = oTmpCarParam->oMass * Gdown + (oTmpCarParam->oMass * Crvz + oCa) * AvgV2;

	  Froad = Fdown * Mu;
	}

	double Flat  = oTmpCarParam->oMass * Glat;
	double Ftan  = oTmpCarParam->oMass * Gtan - Cd * AvgV2;

	double Flatroad = MAX(0.0,oTmpCarParam->oMass * AvgV2 * fabs(Crv) - Flat);
	if (Flatroad > Froad)
	  Flatroad = Froad;

	double Ftanroad = -sqrt(Froad * Froad - Flatroad * Flatroad) + Ftan;

	double Acc = CarParam.oScaleBrakePit * Ftanroad
	  / oTmpCarParam->oMass;

	double Inner = MAX(0, V * V - 2 * Acc * Dist);
	double OldU = U;
	U = sqrt(Inner);
	if (fabs(U - OldU) < 0.001)
	  break;
  }

  return U;
}
//==========================================================================*

//==========================================================================*
// Calculate maximum of speed
//--------------------------------------------------------------------------*
double TFixCarParam::CalcMaxSpeed
  (TCarParam& CarParam,                          // Lane specific parameters
  double Crv0,                                   // Curvature in xy at P
  double Crv1,                                   // Curvature in xy at P
  double CrvZ,                                   // Curvature in z at P
  double Friction,                               // Friction
  double FlyHeight,                              
  double TrackRollAngle,                         // Track roll angle
  double ScaleFriction,                          // Part of Track
  double TargetSpeed) const                      // Max target speed
{
  // Here we calculate the theoretical maximum speed at a point on the
  // path. This takes into account the curvature of the path (crv), the
  // grip on the road (mu), the downforce from the wings and the ground
  // effect (CA), the tilt of the road (left to right slope) (sin)
  // and the curvature of the road in z (crvz).
  //
  // There are still a few silly fudge factors to make the theory match
  // with the reality (the car goes too slowly otherwise, aarrgh!).

  double Mu;

  double Cos = cos(TrackRollAngle);
  double Sin = sin(TrackRollAngle); 

  double AbsCrv0 = MAX(0.001, fabs(Crv0));
  double AbsCrv1 = MAX(0.001, fabs(Crv1));
  double AbsCrv = AbsCrv0;

  double factor = 1.000;
  //double factor = MAX(0.9,MIN(1.0,1.0 - FlyHeight / 0.2));
  if (AbsCrv > AbsCrv1)
	factor *= 1.00;
  else
	factor *= 0.985;

  //if (AbsCrv < 0.0250)
  //	CrvZ = CrvZ / 3;
  //if (AbsCrv < 0.005)
  //	CrvZ = CrvZ / 3;
  if (AbsCrv < 0.005)
  	CrvZ = CrvZ / 3;

  double Den;

  double ScaleBump;
  if (Crv0 > 0)
    ScaleBump = CarParam.oScaleBumpLeft;
  else
    ScaleBump = CarParam.oScaleBumpRight;

  double Factor = CarParam.oScaleMu * ScaleFriction;

  if (oAeroModel == 1)
  {
	double MuF = Friction * oTyreMuFront * Factor;
	double MuR = Friction * oTyreMuRear * Factor;
	Mu = (MuF + MuR) / 2;
    Den = (AbsCrv - ScaleBump * CrvZ)
	  - (oCaFrontWing * MuF + oCaRearWing * MuR + oCaGroundEffect * Mu) / oTmpCarParam->oMass;
  }
  else if (oAeroModel == 2)
  {
	double MuF = Friction * oTyreMuFront * Factor;
	double MuR = Friction * oTyreMuRear * Factor;
	Mu = MIN(MuF,MuR);
    Den = (AbsCrv - ScaleBump * CrvZ)
	  - (oCaFrontWing * MuF + oCaRearWing * MuR + oCaGroundEffect * Mu) / oTmpCarParam->oMass;
  }
  else
  {
	Mu = Friction * oTyreMu * Factor;
    Den = (AbsCrv - ScaleBump * CrvZ) - oCa * Mu / oTmpCarParam->oMass;
  }

  if (Den < 0.00001)
   Den = 0.00001;

  double Speed = factor * sqrt((Cos * G * Mu + Sin * G * SGN(Crv0)) / Den);
  //double Speed = factor * sqrt((Cos * G * Mu) / Den);
  //GfOut("A:%f Sin:%f Sign:%d F:%f S:%f\n",TrackRollAngle,Sin,SGN(Crv0),Sin * SGN(Crv0),Speed); 
  Speed = MIN(Speed,TargetSpeed);

  if (fabs(AbsCrv) > 1/50.0)
    Speed *= 0.93;                               // Filter hairpins
  else if (Speed > 112)                          // (111,11 m/s = 400 km/h)
    Speed = 112;                                 

  if (Speed < 11.0)
	  Speed =  11.0;

  return Speed;
}
//==========================================================================*

//==========================================================================*
// Calculate maximum of speed
//--------------------------------------------------------------------------*
double TFixCarParam::CalcMaxLateralF
  (double Speed, double Friction, double Crvz) const
{
  double Fdown = oTmpCarParam->oMass * G
	- (oTmpCarParam->oMass * Crvz + oCa) * Speed * Speed;
  return Fdown * Friction * oTyreMu;
}
//==========================================================================*

//==========================================================================*
// Calculate maximum of speed
//--------------------------------------------------------------------------*
double TFixCarParam::CalcMaxSpeedCrv() const
{
  const double MAX_SPD = 100; // 360 km/h
//  const double MAX_SPD = 150; // 540 km/h
  return G * oTyreMu / (MAX_SPD * MAX_SPD);
}
//==========================================================================*

//==========================================================================*
// Estimate braking deceleration based on the target speed of the next point
//--------------------------------------------------------------------------*
float TFixCarParam::EstimateBraking(
	double Crv0,								// Curvature in xy at P0
	double Crvz0,								// Curvature in z at P0
	double Crv1,								// Curvature in xy at P1
	double Crvz1,								// Curvature in z at P0
	double Speed,								// Speed
	double Dist,								// Distance P0 P1
	double Friction,							// Friction
	double TrackRollAngle,						// Track roll angle
	double TrackTiltAngle,                      // Track tilt angle
	double ScaleBrake,
	double ScaleBumps)
{
	ScaleBrake = ScaleBrake * oScaleBrake;
	ScaleBumps = ScaleBumps * oScaleBumps;

	// We have to respect the limit at front and rear wheels
	double Mu = Friction * ScaleBrake
		* MIN(oTyreMuFront,oTyreMuRear);

	// Some part of the friction is used for steering
	double MuSteer = Speed * Speed * fabs(Crv1) / G;

	// The remaining we can use for braking
	double MuBrake2 = MAX(0.01,Mu * Mu - MuSteer * MuSteer);

	// Estimate the part of bumps
	double AccelDown = G *
		MAX(0.1,MIN(1.0, (1 + 25 * ScaleBumps * (Crvz0 + Crvz1))));

	// Estimate the part of the aerodynamic downforce
	double AeroDown = sqrt(MuBrake2)
		* MIN(oCaFrontWing + oCaGroundEffect,
		oCaRearWing + oCaGroundEffect);

	// Estimate the part of the aerodynamic drag
	double AeroDrag = 
		(1.0 + oCar->_dammage / 10000.0)// SD specific factor
		* oCdBody				         // Drag of body of car
		+ oCdWing;                      // Drag of wings

	// Estimate the part of the aerodynamic
	double AeroScale = oTmpCarParam->oMass/(AeroDown + AeroDrag);

	// Estimate angle between down and track normal
	double Cos = cos(TrackRollAngle) * cos(TrackTiltAngle);

	// Estimate the part of the usable force
	double Down = MuBrake2 * AccelDown * AeroScale 
		* (Cos - sin(TrackTiltAngle));

	// Calculate the exponent
	double Ex = exp(Dist * 2 / AeroScale);

	// Top secrete formula, do not publish to racers ;-D
	double TargetSpeed = sqrt(Down * ((1 + Speed*Speed/Down)*Ex - 1));

	// Estimate speed in the middle of the section
	double MidSpeed = (TargetSpeed + Speed)/2;

	// Check brake
	double BrakeDecel = ScaleBrake * oBrakeForce / oTmpCarParam->oMass;
	double BrakeTargetSpeed = sqrt(MidSpeed * MidSpeed + 2 * BrakeDecel * Dist);
	double ResultTargetSpeed = MIN(TargetSpeed,BrakeTargetSpeed);

	// Sanity check
	return (float) MAX(ResultTargetSpeed,Speed);
}
//==========================================================================*

//==========================================================================*
// Estimate maximum of speed based on the local curvature and slope
//--------------------------------------------------------------------------*
float TFixCarParam::EstimateSpeed(
	double Crv0,								// Curvature in xy at P
	double CrvZ,								// Curvature in z at P
	double Friction,							// Friction
	double TrackRollAngle,						// Track roll angle
	double TrackTiltAngle,                      // Track tilt angle
	double ScaleSpeed,
	double ScaleBumps)
{
    double ScaleBump;
    if (Crv0 > 0)
      ScaleBump = oDriver->Param.oCarParam2.oScaleBumpLeft;
    else
      ScaleBump = oDriver->Param.oCarParam2.oScaleBumpRight;

	ScaleSpeed = ScaleSpeed * oScaleSpeed;
	ScaleBumps = ScaleBumps * oScaleBumps;

	// Estimate part of friction we can use for steering, because we
	// have to brake into the curve as well!
	double SteerFriction = Friction * ScaleSpeed;   

	double AbsCrv = MAX(0.0001,fabs(Crv0));      // Sanity check
	
	double Cos = cos(TrackRollAngle);
	double Sin = sin(TrackRollAngle);

	// Estimate the part of bumps
	double Accel = (1 + 50 * (oScaleBumps + ScaleBump) * CrvZ) * Cos * cos(TrackTiltAngle);
	Accel = G * MAX(0.5,MIN(1.0, Accel));
	Accel *= MIN(1.0,200.0 * AbsCrv);

	// First calculation for front tyres:
	double Mu = SteerFriction * oTyreMuFront;
	double Den = MAX(0.00001,AbsCrv - 
		(oCaFrontWing + oCaGroundEffect/2) 
		* Mu / oTmpCarParam->oMass);

	double SpeedSqrFront = (Cos * Mu + Sin * SGN(Crv0)) * Accel / Den;

	// Next calculation for rear tyres:
	Mu = SteerFriction * oTyreMuRear;
	Den = MAX(0.00001,AbsCrv - 
		(oCaRearWing + oCaGroundEffect/2) 
		* Mu / oTmpCarParam->oMass);

	double SpeedSqrRear = (Cos * Mu + Sin * SGN(Crv0)) * Accel / Den;

	return (float) sqrt(MIN(SpeedSqrFront,SpeedSqrRear));
}
//==========================================================================*

//--------------------------------------------------------------------------*
// end of file unitfixcarparam.cpp
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
