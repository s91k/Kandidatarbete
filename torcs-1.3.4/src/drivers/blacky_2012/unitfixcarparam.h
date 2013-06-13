//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitfixcarparam.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter für TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Unveränderliche Parameter des Fahrzeugs
//
// Datei    : unitfixcarparam.h
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
#ifndef _UNITFIXCARPARAM_H_
#define _UNITFIXCARPARAM_H_

#include "unitglobal.h"
#include "unitcommon.h"

#include "unittmpcarparam.h"
#include "unitcarparam.h"

//==========================================================================*
// Deklaration der Klasse TFixCarParam
//--------------------------------------------------------------------------*
class TFixCarParam
{
  private:

  public:
    PtCarElt oCar;                               // Pointer to TORCS data of car
	PDriver oDriver;							 // Driver
    TTmpCarParam* oTmpCarParam;                  // Variable car parameters

	TFixCarParam();                              // Default constructor
	~TFixCarParam();                             // Destructor

	double CalcAcceleration
	  (
	   double Crv0, double Crvz0,
	   double Crv1, double Crvz1,
	   double Speed, double Dist,
	   double Friction,
	   double TrackRollAngle) const;

	double CalcBraking
	  (TCarParam& CarParam,
	   double Crv0, double Crvz0,
	   double Crv1, double Crvz1,
	   double FlyHeight,
	   double Speed, double Dist,
	   double Friction,
	   double TrackRollAngle,
	   double TrackTiltAngle,
       double ScaleBraking);  

	double CalcBrakingPit
	  (TCarParam& CarParam,
	   double Crv0, double Crvz0,
	   double Crv1, double Crvz1,
	   double FlyHeight,
	   double Speed, double Dist,
	   double Friction,
	   double TrackRollAngle
	   /* int ID*/ ) const;

	double CalcMaxSpeed
	  (TCarParam& CarParam,
	   double Crv0,
	   double Crv1,
	   double Crvz,
	   double Friction,
       double FlyHeight,                              
	   double TrackRollAngle,
       double ScaleFriction,
       double TargetSpeed) const;

	double CalcMaxLateralF
	  (
       double Speed,
	   double Friction,
	   double Crvz = 0.0) const;

	double CalcMaxSpeedCrv() const;

    float EstimateBraking(
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
		double ScaleBumps);

	float EstimateSpeed(
		double Crv0,								// Curvature in xy at P
		double CrvZ,								// Curvature in z at P
		double Friction,							// Friction
		double TrackRollAngle,						// Track roll angle
		double TrackTiltAngle,                      // Track tilt angle
		double ScaleSpeed,
		double ScaleBumps);

	void Initialize(
	  PtCarElt Car,
	  PDriver Driver);

  public:
	tdble oBorderInner;                          // Const. Buffer to inner
	tdble oBorderOuter;                          // Const. Buffer to outer
	tdble oMaxBorderInner;                       // Max var. Buffer to inner
	tdble oBorderScale;                          // Scale var. Buffer to inner
	double oBrakeForce;
	double oCa;                                  // Aerodynamic downforce constant
	double oCaFrontWing;                         // Aerod. d. const. front wing
	double oCaGroundEffect;                      // Aerod. d. const. ground effect
	double oCaRearWing;                          // Aerod. d. const. rear wing
	double oCdBody;                              // Aerodynamic drag constant car body
	double oCdWing;	                             // Aerod. drag const. wings
	double oEmptyMass;                           // Mass of car.without fuel
	double oLength;                              // Length of car (m)
	double oTyreMu;	                             // Mu of tyres
	double oTyreMuFront;                         // Mu of front tyres
	double oTyreMuRear;                          // Mu of rear tyres
	double oWidth;                               // Width of car (m)

	double oAeroModel;                           // 0: def; 1: sep; 2: min
	double oPitBrakeDist;
	double oXXX;

	float oScaleBrake;
	float oScaleBumps;
	float oScaleSpeed;


};
//==========================================================================*
#endif // _UNITFIXCARPARAM_H_
//--------------------------------------------------------------------------*
// end of file unitfixcarparam.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
