//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitdriver.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter für TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Zentrale Klasse für das Fahren bzw. den Fahrer/Roboter
//
// Datei    : unitdriver.h
// Erstellt : 17.11.2007
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
//    Copyright: (C) 2006-2007 Tim Foden.
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
#ifndef _UNITDRIVER_H_
#define _UNITDRIVER_H_

#include <track.h>
#include <car.h>
#include <robot.h>

#include "unitglobal.h"
#include "unitcommon.h"
#include "unitcommondata.h"

#include "unitcharacteristic.h"
#include "unitcollision.h"
#include "unitclothoid.h"
#include "unitpit.h"
#include "unitstrategy.h"
#include "unittrack.h"
#include "unitopponent.h"
#include "unitparam.h"                           // TParam
#include "unitpidctrl.h"
#include "unitsysfoo.h"
#include "unitteammanager.h"

//==========================================================================*
// Deklaration der Klasse TDriver
//--------------------------------------------------------------------------*
class TDriver
{
  public:
    double CurrSimTime;                          // Current simulation time

	TDriver(int Index);                          // Constructor
	~TDriver();                                  // Destructor

	// TORCS-Interface:
	void InitTrack                               // Initialize Track
	  (PTrack track, PCarHandle CarHandle,
	  PCarSettings *CarParmHandle,
	  PSituation Situation);
	void NewRace                                 // Start new Race
	  (PtCarElt Car, PSituation Situation);
	void InitDrive();                            // Initialize driving
	void SetupRacingLines();                     // Initialize racing lines
	void Drive();                                // Drive while racing
	int	PitCmd();                                // Handle pitstop
	void EndRace();                              // Stop race
	void Shutdown();                             // Cleanup

    void Update                                  // Update data
	  (PtCarElt Car, PSituation Situation);
    void InitAdaptiveShiftLevels();              // Calculate shifting levels
    float BetterSteering();                      // Steering while unstucking
	double StartAutomatic();                     // Clutch controller for start
	void Clutching();                            // Clutch controller
	float DetectFlight();                        // Detect flight
    void FindRacinglines();                      // Find racing lines
	void FlightControl();                        // Prepare landing
	double GearRatio();                          // Get gear ratio
    void GearTronic();                           // GearTronic
    void InitCarModells();                       // Initialize car modells
	void InitBrake();                            // Initialize brake
	void InitCa();                               // Initialize Ca
	void InitCw();                               // Initialize Cw
    void InitDriveTrain();                       // Initialize drive train
	void InitTireMu();                           // Initailize tire mu
    void InitWheelRadius();                      // Calculate mean wheel radius
    bool IsStuck();                              // Stehen wir vor einem Hindernis
	double NextGearRatio();                      // Get next gear ratio
    bool TargetReached                           // Target reached
      (double Target, double AvoidTarget);
	int PitSide();                               // Side of pitlane
	double PrevGearRatio();                      // Get prev gear ratio
	void Propagation();                          // Propagation
	double Steering();                           // Steering
	void TeamInfo();                             // Get team infos
	void Turning();                              // Turn if needed
    void Unstuck();                              // Rangieren
    void YawControl();                           // Controll yawing
    void SetPathAndFilenameForRacinglines();     // Adjust pathes

    inline PTrack Track();                       // Get Pointer to TORCS track data
    inline PtCarElt Car();                       // Get Pointer to TORCS car data

	void GetLanePoint                            // Interpolate Lanepoint
      (int Path, double Pos, TLanePoint& LanePoint);
	void GetPosInfo                              // Get Info to position
	  (double pos, TLanePoint& pi, double u, double v);
	void GetPosInfo                              // Get Info to position
	  (double pos, TLanePoint& pi);
	double CalcPathTarget                        // Get target
	  (double pos, double offs);
	TVec2d CalcPathTarget2                       // Get target
	  (double pos, double offs);
	void GetPathToLeftAndRight                   // Get width to sides
	  (const PCarElt Car, double& toL, double& toR);
    void OwnCarOppIndex();                       // Get own index

	double SteerAngle                            // Get steer angle
	  (TLanePoint& AheadLanePoint);

	void BrakingForceRegulator();                // Regulate braking
    void BrakingForceRegulatorAvoid();           // R. b. while avoiding
    void BrakingForceRegulatorTraffic();         // R. b. in trafic

//	inline void SetBotName                       // Set name of bot
//	  (char* Value);
	void SetBotName                              // Set name of bot
	  (char* Value);
	inline void	SetCommonData                    // Set pointer to common data
	  (TCommonData* CommonData);
	inline TTeamManager::TTeam* GetTeam();
    inline char* GetBotName();
    inline int GetMinLaps();
    bool SaveToFile();
    bool SaveBrakeCoeffsToFile();
    bool LoadBrakeCoeffsFromFile();

private:
	void AvoidOtherCars                          // Avoiding
	  (double K, bool& IsClose, bool& IsLapper);
    void EvaluateCollisionFlags(                 // Check flags
      int I,
      TCollision::TCollInfo& Coll,
      double Crv,
      double& MinCatchTime,
      double& MinCatchAccTime,
      double& MinVCatTime,
	  bool& IsLapper);

	double FilterRelax(double TargetSpeed);      // Relax mode
	double FilterSteerSpeed(double Steer);       // Limit steer speed
	double FilterABS(double Brake);              // ABS filter
    double FilterBrake(double Brake);            //

	double FilterCrv(double Accel);              // Curves
	double FilterDiff(double Accel);             // Different segments
	double FilterDrifting(double Accel);         // Drifting
	double FilterLetPass(double Accel);          // Reduce accel
	double FilterTCL(double Accel);              // Tracktion control
    double FilterTrack(double Accel);            // Keep on track

    void NextCurvature                           // Get next crv
	  (TCollision::TCollInfo& Coll, PtCarElt Car);
    void Runaround                               // run around obstacles
	  (double Scale, double Target, bool DoAvoid);
    double UnstuckSteerAngle();                  // steer while unstucking
//      (TLanePoint& PointInfo, TLanePoint& AheadPointInfo);
    double TargetDist(double Speed);
    bool OppInFrontBraking();
	TVec2d GetTargetPoint(float LookAhead);
	TVec2d GetAvoidTargetPoint(double AvoidSide, float LookAhead);

    void InterpolatePointInfo                    // Interpolation
	  (TLanePoint& P0, const TLanePoint& P1, double Q);
	
	void CheckTolerance();
	void DriveFaster(double Scale);
//	void DriveSlower(double Scale, int Length = 20);
	void DriveSlower(double Scale, int Length = 8);
	void SpeedController();
	void BrakeEarlier(double Scale);
	void BrakeLater(double Scale);
    void UpdateTargetSpeed();

private:
	enum // drive types
	{
		cDT_RWD, cDT_FWD, cDT_4WD,
	};

	TCommonData* oCommonData;                    // Pointer to common data
	TTrackDescription oTrackDesc;                // Track description
	TClothoidLane oRacingLine[gNBR_RL];          // Racinglines

	TCarParam* oCarParams[3];                    // Array of pointers to parameter sets

    bool oFirst;                                 // To initialize
    bool oSecond;                                // Second starter of the team
    double oFlyHeight;                           // fly height
	tdble oBumpMod;                              // bump mode
	double oScaleSteer;                          // scale steering
	double oStayTogether;			             // Dist in m.
	double oAvoidScale;			                 // scale avoiding
	double oAvoidWidth;			                 // In m.
	bool oGoToPit;                               // Enter pit flag
	TTeamManager::TTeam* oTeam;                  // Team

	int	oDriveTrainType;                         // Drive train type
	TPidController oPIDCLine;      	             // Controller for line error.
	int	oFlying;				                 // Flag prepare landing
    float oLastRideHeight;                       // Last ride height
    int oNbrCars;                                // Nbr of cars in race
	int	oOwnOppIdx;                              // Index of own car in list of opponents
	TOpponent* oOpponents;						 // Infos about other cars.

	double oAvoidRange;				             // Where we are T->LR (0..1).
	double oAvoidRangeDelta;                     // Delta to change range
	double oAvoidOffset;				         // Where we are L->R (-1..1).
	double oAvoidOffsetDelta;                    // Delta to change offset

	TCharacteristic oMaxAccel;                   // Cars accelleration characteristic

	tdble oBrakeCoeff[NBR_BRAKECOEFF];           // Brake coefficients
	double oBrakeMaxTqFront;
	double oBrakeMaxTqRear;
	double oBrakeForce;
	int	oLastBrakeCoefIndex;                     // Index of last brake coef.
	double oMinBrakeFrictionFactor;
	double oMaxBrakeFrictionFactor;
	double oMinSpeedFrictionFactor;
	double oMaxSpeedFrictionFactor;
	int oExcludeFrom;
	int oExcludeTill;
	double oLastBrake;                           // Last brake command
	double oLastTargetSpeed;                     // Last target speed

    // State values to update commands
	double oAccel;                               // Accelleration
	double oLastAccel;                           // Last accelleration
	double oBrake;                               // Braking
	double oClutch;                              // Clutching
	int oGear;                                   // Gear
	double oSteer;                               // Steering

	double oClutchDelta;
	double oClutchMax;
	double oClutchRange;
	double oClutchRelease;
	double oAbsDelta;
	double oAbsScale;
    bool oAlone;                                 // No opponent near
	double oAngle;                               // Actual Angle
    double oAngleSpeed;                          // Angle of speed
	char* oBotName;                              // Name of driver
	char* oTeamName;                             // Name of team
	int oRaceNumber;                             // Race number
	double oBrakeDiffInitial;                    // Initial difference to brake
	double oBrakeForceMax;                       // Maximum braking force
	double oBrakeScale;                          // Brake force scaling
	tdble oInitialBrakeCoeff;
	PtCarElt oCar;                               // TORCS data for own car
    float oSteerAngle;                           // Angle to steer
    char* oCarType;                              // Type name of own car
	double oGearEff[MAX_GEARS];                  // Efficiency of gears
	int oLastGear;                               // Last gear
    bool oLetPass;                               // Let opoonent pass
	double oLookAhead;                           // Look ahead base value
	double oLookAheadFactor;                     // Look ahead factor
	double oLookScale;                           // Actual scale
	double oLookBase;                            // Actual base
	double oOmegaBase;                           // Actual base
	double oOmegaScale;                          // Actual scale
	double oOmegaAheadFactor;                    // Omega ahead factor
	double oOmegaAhead;                          // Omega ahead base value
    double oDistFromStart;                       // Position along Track
    double oShift[MAX_GEARS];                    // Shift levels
    double oShiftMargin;                         // Shift back margin
	PSituation oSituation;                       // TORCS data fpr situation
	double oStartDistance;                       // max Dist. raced while starting
    int oStuckCounter;                           // Tick counter
    PSysFoo oSysFooStuckX;                       // Positionsüberwachung in X
    PSysFoo oSysFooStuckY;                       // und Y
    int oTestPitStop;                            // Test pit stop
	float oTrackAngle;                           // Direction of track
    double oTargetSpeed;                         // Target speed for speed controller
    double oTargetDelta;                         // Delta target speed for speed controller
	double oTclRange;                            // TCL range
	double oTclSlip;                             // Max TCL slip
	char* oTrackName;                            // Name of track to drive on
	char* oPathToWriteTo;                        // Path to write to
	char* oTrackLoad;                            // Name of track to drive on
	char* oTrackLoadLearned;                     // Name of track to drive on
	char* oTrackLoadLearnedXML;                  // Name of track to drive on
	char* oTrackLoadQualify;                     // Name of track to drive on
	char* oTrackLoadLeft;                        // Name of track to drive on
	char* oTrackLoadRight;                       // Name of track to drive on
	char* oPitLoad[3];                           // Name of track to drive on
	PTrack oTrack;                               // TORCS data fpr track
	double oTolerance;                           // Tolerable offset difference
	float oSmoothSide;                           // Smmoth width changes
    TLanePoint oLanePoint;                       // Information to Point
	bool oUnstucking;                            // Surmounting himdrance
	double oWheelRadius;                         // Mean wheel radius
    double oDeltaOffset;                         // Delta to planned
    double oDriftAngle;                          // Drifting angle
	int oLetPassSide;                            // Go to side to let pass
	double oOldTarget;
    bool oReduced;
	bool oShowIndex;
	bool oSavePlot;
	bool oSaveSections;
	double oNoAvoidLength;
    double oStartRPM;
	double oStartSide;
    double oFuelNeeded;
	double oRepairNeeded;
	float oSideReduction;
    float oMinDistLong;
	int oLastCarLaps;

	int NBRRL;
	int oRL_FREE;
	int oRL_LEFT;
	int oRL_RIGHT;

	PCarHandle oCarHandle;                       // Handle of car parameter file
    PSimpleStrategy oStrategy;                   // Pit strategy
	bool oSaveStatistics;                        // Save statistics
    bool oDoAvoid;                               // Do avoid

	bool oLoadLearned;
	bool oLoadedLearned;
	int oLapsLearned;
	int oSecIndex;
	double oDifference;
	int oLastLap;
	double oCorrFactor;

  public:
	static float FlyLevel;                       // Racingline parameter
	static float FlySection;                     // Racingline parameter
	static float RLFactor;                       // Racingline parameter
	static float MaxCrvAngle;                    // Racingline parameter 
	static double Apex;                          // Racingline parameter 
	static double LengthMargin;                  // Length margin
	static bool Qualifying;                      // Flag qualifying

	int oIndex;                                  // index of own driver
	bool oStanding;                              // Fahrzeug steht#
	TParam Param;                                // Parameters
    //bool KiloIsInFront;                          // Project Laurin
	//double BlinkTimer;                           // Project Laurin
    //bool Blinking();                             // Project Laurin
	double oFuelPer100km;                        //
	float oMaxFuel;                              // tank capacity
	int oMinLaps;
	double oBestLapTime;
	double oBestFuelPer100km;                    //
	double oSpeedScale;                          //
    bool oTreatTeamMateAsLapper;
    bool oBrakeForTeamMate;
    bool oBrakeTeamMateAside;
	int oCarsPerPit;                             // Pit sharing
	int oTurnCounter;
    float oLastSteer;
	float oMinScaleBrake;
	float oMinScaleMu;
	float oScaleMuTraffic;
	float oScaleBrakeTraffic;
	float oDistToTraffic;
	float oSlowSpeed;
	float oScaleRefuel;
    double oLastTarget;
    int oSideCounter;
	int oOppsNearBehind;
	int oOppsNearInFront;
	int oLastNearInFront;
    double oAvoidSide;
	bool oPreviewSlow;
	bool oUseFilterDrifting;
	bool oUseFilterTrack;
	bool oUseFilterTCL;
	bool oUseFilterDiff;
	bool oUseFilterRelax;
	double oCurrSpeed;                           // Currend speed

	double oStartSteerFactor;                    // Steer speed at start

	static int NBBOTS;                           // Nbr of cars
	static char* MyBotName;                      // Name of this bot
	static char* ROBOT_DIR;                      // Sub path to dll
	static char* SECT_PRIV;                      // Private section
	static char* DEFAULTCARTYPE;                 // Default car type

	static bool AdvancedParameters;

};
//==========================================================================*

//==========================================================================*
// Get name of robot
//--------------------------------------------------------------------------*
char* TDriver::GetBotName()
  {return oBotName;};
//==========================================================================*

//==========================================================================*
// Get name of robot
//--------------------------------------------------------------------------*
int TDriver::GetMinLaps()
  {return oMinLaps;};
//==========================================================================*

//==========================================================================*
// Set pointer to common data
//--------------------------------------------------------------------------*
void TDriver::SetCommonData
  (TCommonData* CommonData){oCommonData = CommonData;};
//==========================================================================*

//==========================================================================*
// Get Team
//--------------------------------------------------------------------------*
TTeamManager::TTeam* TDriver::GetTeam()
{
  return oTeam;
}
//==========================================================================*

//==========================================================================*
// Get Pointer to TORCS data of track
//--------------------------------------------------------------------------*
PTrack TDriver::Track()
  {return oTrack;};
//==========================================================================*

//==========================================================================*
// Get Pointer to TORCS data of car
//--------------------------------------------------------------------------*
PtCarElt TDriver::Car()
  {return oCar;};
//==========================================================================*
#endif // _UNITDRIVER_H_
//--------------------------------------------------------------------------*
// end of file unitdriver.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*

