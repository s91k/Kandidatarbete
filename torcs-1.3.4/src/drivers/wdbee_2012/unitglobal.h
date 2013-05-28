//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitglobal.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Globale Datentypen und Definitionen
//
// Datei    : unitglobal.h
// Erstellt : 17.11.2007
// Stand    : 11.08.2012
// Copyright: � 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.06.000 (Championship 2012 Ruudskogen)
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
#ifndef _UNITGOBAL_H_
#define _UNITGOBAL_H_

// The great question, ...
#if defined(WIN32) || defined(_WIN32)
#include <windows.h>     // The rich world of windows and gates, ...
#define mysecure           // Use _fopen_s
#define myhypot _hypot     // Use _hypot instead of hypot
#define myfopen _fopen_s   // Use _fopen_s instead of fopen
#else                    // but in a poor world without walls and fences, ...
#define myhypot hypot      // Use hypot
#define myfopen fopen      // Use fopen
#endif                   // ... who needs windows and gates?
// ... but the answer is just 42!

//#include <portability.h> do not use this, vc++ 2005 warnings ...

// VC++ 2005 ...
#if defined(_CRT_SECURE_NO_DEPRECATE) // used with vc++ 2005
#define snprintf _snprintf_s
#endif
// ... VC++ 2005

// VC++ 6.0 ...
#if defined(WIN32) && !defined(snprintf_s)
#undef snprintf
#define snprintf _snprintf
#endif

#if defined(WIN32) && !defined(fopen_s)
#undef mysecure
#endif
// ... VC++ 6.0

#include <tgf.h>     // TORCS
#include <track.h>   // TORCS
#include <car.h>     // TORCS
#include <raceman.h> // TORCS

//#include "identity.txt"

//==========================================================================*
// Global constants, to be changed for different wdbee-bots
//--------------------------------------------------------------------------*
#define MAX_NBBOTS 10                            // Number of drivers/robots

// Estimation of acceleration: depends on car types
// carX-trb1:
const double PAR_A = 0.001852;                   // Parameters of a quadratic
const double PAR_B = -0.35;                      // of X = speed in [m/s]
const double PAR_C = 17.7;                       // Acc = (A*X+B)*X+C
//==========================================================================*

//==========================================================================*
// Other global constants
//--------------------------------------------------------------------------*
const float TRACKRES = 2.5;                      // Digitising sampling rate
const int FLY_COUNT = 20;                        // Fly counter
const int NEW_VERSION = 307;                     // Version Street-1
//==========================================================================*

//==========================================================================*
// Forewarding for Classes and pointers to
//--------------------------------------------------------------------------*
class TAbstractStrategy;
typedef TAbstractStrategy* PAbstractStrategy;

class TAdjustedCharacteristic;
typedef TAdjustedCharacteristic* PAdjustedCharacteristic;

class TCarParam;
typedef TCarParam* PCarParam;

class TClothoidLane;
typedef TClothoidLane* PClothoidLane;

class TCollision;
typedef TCollision* PCollision;

class TCommonData;
typedef TCommonData* PCommonData;

class TDriver;
typedef TDriver* PDriver;

class TGenericAvoidance;
typedef TGenericAvoidance* PGenericAvoidance;

class TLane;
typedef TLane* PLane;

class TLanePoint;
typedef TLanePoint* PLanePoint;

class TLinAttractor;
typedef TLinAttractor* PLinAttractor;

class TLinearRegression;
typedef TLinearRegression* PLinearRegression;

class TOpponent;
typedef TOpponent* POpponent;

class TParam;
typedef TParam* PParam;

class TPath;
typedef TPath* PPath;

class TPit;
typedef TPit* PPit;

class TQuadratic;
typedef TQuadratic* PQuadratic;

class TSimpleStrategy;
typedef TSimpleStrategy* PSimpleStrategy;

class TSection;
typedef TSection* PSection;

class TTeamManager;
typedef TTeamManager* PTeamManager;

class TTrackDescription;
typedef TTrackDescription* PTrackDescription;
//==========================================================================*

//==========================================================================*
// Type definitions for TORCS pointers
//--------------------------------------------------------------------------*
typedef tTrack* PTrack;                            // TORCS track
typedef tCarElt* PtCarElt;                         // TORCS car
typedef CarElt* PCarElt;                           // TORCS car
typedef void* PCarHandle;                          // TORCS file handle
typedef void* PCarSettings;                        // TORCS file handle
typedef tSituation* PSituation;                    // TORCS situation
typedef tTrackSeg* PTrackSeg;                      // TORCS segment of track

//==========================================================================*

//==========================================================================*
// Indices for the different racinglines
//--------------------------------------------------------------------------*
enum
{
  RL_FREE,                                       // Racing untroubled
  RL_LEFT,                                       // Racing on left side
  RL_RIGHT,                                      // Racing on right side

  gNBR_RL                                        // Nbr of racinglines
};
//==========================================================================*

//==========================================================================*
// Collision flags
//--------------------------------------------------------------------------*
enum
{
  F_LEFT			= 0x000001, // You are at my left side
  F_RIGHT			= 0x000002, // or my right side
  F_FRONT			= 0x000004, // in front of me
  F_REAR			= 0x000008, // behind me

  F_AHEAD			= 0x000010, // I see you in front of me
  F_AT_SIDE			= 0x000020, // or looking to a side
  F_BEHIND			= 0x000040, // or in the mirror

  F_TRK_LEFT		= 0x000100, // British
  F_TRK_RIGHT		= 0x000200, // European

  F_CATCHING		= 0x001000, // Tom and
  F_CATCHING_ACC	= 0x002000, //   Jerry
  F_COLLIDE			= 0x004000, // My assurance nbr is ...
  F_TRAFFIC			= 0x008000, // Business as usual
  F_CLOSE			= 0x010000, // You are too close to me!
  F_TEAMMATE		= 0x020000, // Not like Alonso and Hamilton
  F_LAPPER			= 0x040000,	// it's lapping us
  F_BEING_LAPPED	= 0x080000,	// we're lapping it
  F_DANGEROUS		= 0x100000, // ugly!
  F_PREVIEWSLOW		= 0x1000000  // ugly!
};
//==========================================================================*

//==========================================================================*
// #defines
//--------------------------------------------------------------------------*
// Array sizes ...
#define NBR_BRAKECOEFF 50                        // Number of brake coeffs
// ... Array sizes

// my own planet ...
#define G 9.81                                   // Hello earth gravity
// ... my own planet

// pit states ...
#define PIT_IS_FREE (-1)
// ... pit states

const float HALFFRICTION = (float) sqrt(0.5); // 1=Sqrt(2*HALFFRICTION**2) 

// Shortcuts for TORCS commands ...
#define CarAccelCmd (oCar->_accelCmd)
#define CarBrakeCmd (oCar->_brakeCmd)
#define CarClutchCmd (oCar->_clutchCmd)
#define CarGearCmd (oCar->_gearCmd)
#define CarSteerCmd (oCar->_steerCmd)
#define CarRaceCmd (oCar->_raceCmd)
// ... Shortcuts for TORCS

// Shortcuts for TORCS commands ...
#define CarAccelLat (oCar->_accel_y)
#define CarAccelLong (oCar->_accel_x)
#define CarBestLapTime (oCar->_bestLapTime)
#define CarCurLapTime (oCar->_curLapTime)
#define CarDamage (oCar->_dammage)
#define CarDistFromStart (oCar->race.distFromStartLine)
#define CarDriverIndex (oCar->_driverIndex)
#define CarFriction (oCar->_trkPos.seg->surface->kFriction)
#define CarFuel (oCar->_fuel)
#define CarGearNbr (oCar->_gearNb)
#define CarGearOffset (oCar->_gearOffset)
#define CarGearRatio oCar->_gearRatio
#define CarCarHandle (oCar->_carHandle)
#define CarIndex (oCar->index)
#define CarLaps (oCar->_laps)
#define CarLapsBehindLeader (oCar->_lapsBehindLeader)
#define CarLastLapTime (oCar->_lastLapTime)
#define CarLength (oCar->_dimension_x)
#define CarPit (oCar->_pit)
#define CarPosX (oCar->_pos_X)
#define CarPosY (oCar->_pos_Y)
#define CarPosZ (oCar->_pos_Z)
#define CarPubGlobPosX (oCar->pub.DynGCg.pos.x)
#define CarPubGlobPosY (oCar->pub.DynGCg.pos.y)
#define CarPubGlobVelX (oCar->pub.DynGCg.vel.x)
#define CarPubGlobVelY (oCar->pub.DynGCg.vel.y)
#define CarPubGlobAccX (oCar->pub.DynGCg.acc.x)
#define CarPubGlobAccY (oCar->pub.DynGCg.acc.y)
#define CarRpm (oCar->_enginerpm)
#define CarRpmLimit (oCar->_enginerpmRedLine)
#define CarSeg (oCar->_trkPos.seg)
#define CarSpeedLat (oCar->_speed_y)
#define CarSpeedLong (oCar->_speed_x)
#define CarSpeedX (oCar->_speed_X)
#define CarSpeedY (oCar->_speed_Y)
#define CarSteerLock (oCar->_steerLock)
#define CarState (oCar->_state)
#define CarTeamname (oCar->_teamname)
#define CarToMiddle (oCar->_trkPos.toMiddle)
#define CarTrackPos (oCar->_trkPos)
#define CarWidth (oCar->_dimension_y)
#define CarYaw (oCar->_yaw)
#define CarYawRate (oCar->_yaw_rate)
#define DistanceFromStartLine (oCar->_distFromStartLine)
#define DistanceRaced (oCar->_distRaced)
#define HasDriveTrainFront (oDriveTrainType == cDT_FWD || oDriveTrainType == cDT_4WD)
#define HasDriveTrainRear (oDriveTrainType == cDT_RWD || oDriveTrainType == cDT_4WD)
#define NextGear (oCar->_gear + 1)
#define PrevGear (oCar->_gear - 1)
#define RemainingLaps (oCar->_remainingLaps)
#define TreadClutch (oClutch = 0.5)
#define NextRpm (oShift[oCar->_gear])
#define IsFullThrottle ((oAccel >= 1.0) && (oBrake <= 0.0))
#define SteerLock (oCar->_steerLock)
#define WheelRad(x) (oCar->_wheelRadius(x))
#define WheelSpinVel(x) (oCar->_wheelSpinVel(x))
#define WheelSeg(x) (oCar->_wheelSeg(x))
#define WheelSegFriction(x) (oCar->_wheelSeg(x)->surface->kFriction)
#define WheelSegRoughness(x) (oCar->_wheelSeg(x)->surface->kRoughness)
#define WheelSegRollRes(x) (oCar->_wheelSeg(x)->surface->kRollRes)
#define UsedGear (oCar->_gear)
// ... Shortcuts for TORCS

// Shortcuts for this robot ...
#define IsTickover (oCar->_gear <= 0)
#define FixRange (((AvoidTarget < OldAvoidRange) && (AvoidTarget >= oAvoidRange)) || ((AvoidTarget > OldAvoidRange) && (AvoidTarget <= oAvoidRange)) || (fabs(oAvoidRange - AvoidTarget) < 0.0005))
#define FixOffset (((OldAvoidOffset < Target) && (oAvoidOffset >= Target)) || ((OldAvoidOffset > Target) && (oAvoidOffset <= Target)))
#define	DEG_TO_RAD(x)	((x) * PI / 180.0)
#define	STEER_SPD_IDX(x)	(int(floor((x) / 5)))
#define	STEER_K_IDX(k)		(MAX(0, MIN(int(20 + floor((k) * 500 + 0.5)), 40)))
#define	SGN(X) ((X) < 0 ? -1 : (X) > 0 ? 1 : 0)
#define MINMAX(X,Y) (MAX(-X,MIN(X,Y)))
#define DIRCHANGED(X,Y,Z) ((X < Z) && (Y >= Z) || (X > Z) && (Y <= Z))
#define XX2Y(X,Y) (X*X/(2*Y))

#ifndef FLOAT_NORM_PI_PI
#define FLOAT_NORM_PI_PI(x) 				\
{ \
	while ((x) > PI) { (x) -= (float)(2*PI); } \
	while ((x) < -PI) { (x) += (float)(2*PI); } \
}
#endif

#define DOUBLE_NORM_PI_PI(x) 				\
{ \
	while ((x) > PI) { (x) -= 2*PI; } \
	while ((x) < -PI) { (x) += 2*PI; } \
}
// ... Shortcuts for this robot

// Parameters of this robot ...
#define PRV_OPTIMIZATION	 "optimization"
#define PRV_RESERVE          "reserve"            // Reserve in m
#define PRV_SCALE_REFUEL     "scale refuel"       // Factor used while refuel
#define PRV_SCALE_BRAKE2     "scale brakes"       // Scale brake
#define PRV_SCALE_BUMPS2     "scale bumps"        // Scale bumps
#define PRV_SCALE_SPEED2     "scale speed"        // Scale speed

#define PRV_TRKPIT_END       "trkpit end"
#define PRV_TRKPIT_START	 "trkpit start"

// Still in work ...
#define PRV_PIT_ENTRY_LONG	 "pit entry long"
#define PRV_PIT_EXIT_LONG	 "pit exit long"
#define PRV_PIT_EXIT_LEN	 "pit exit length"
#define PRV_PIT_LAT_OFFS	 "pit lat offset"
#define PRV_PIT_LONG_OFFS	 "pit long offset"
#define PRV_PIT_SCALE_BRAKE  "pit scale brake"
#define PRV_PIT_STOP_DIST    "pit stop dist"
#define PRV_PIT_BRAKE_DIST   "pit brake dist"
#define PRV_PIT_TEST_STOP    "pit test stop"
#define PRV_PIT_USE_FIRST    "pit use first"
#define PRV_PIT_USE_SMOOTH   "pit use smooth"
#define PRV_PITLANE_ENTRY    "pitlane entry offset"
#define PRV_PITLANE_EXIT     "pitlane exit offset"
// ... Still in work

#define PRV_QUALIFICATION    "qualification"      // Practice as qualifying

#define PRV_AERO_MOD		 "aero mod"

#define PRV_BORDER_INNER     "border inner"
#define PRV_BORDER_OUTER     "border outer"
#define PRV_MAX_BORDER_INNER "border inner max"
#define PRV_BORDER_SCALE     "border scale"
#define PRV_BUMP_MOD		 "bump mod"

#define PRV_LOAD_LEARNED	 "load learned"

#define PRV_MIN_BRAKEFF		 "min brake friction factor"
#define PRV_MAX_BRAKEFF		 "max brake friction factor"

#define PRV_MIN_SPEEDFF		 "min speed friction factor"
#define PRV_MAX_SPEEDFF		 "max speed friction factor"

#define PRV_EX_FROM			 "exclude from"
#define PRV_EX_TILL			 "exclude till"

#define PRV_FLY_LEVEL		 "fly level"
#define PRV_FLY_SECTION		 "fly section"
#define PRV_MAX_CRVANGLE	 "max crvangle"
#define PRV_APEX	         "apex"
#define PRV_FACTOR			 "rlfactor"
#define PRV_FUELPER100KM     "fuelper100km"       // Spritverbrauch pro 100 km

#define PRV_LOOKAHEAD        "lookahead"
#define PRV_LOOKAHEADFACTOR  "lookaheadfactor"

#define PRV_OMEGAAHEAD       "omegaahead"
#define PRV_OMEGAAHEADFACTOR "omegaaheadfactor"

#define PRV_INIT_BRAKE       "initial brake"      // Scale brake coeff

#define PRV_SMOOTH_SIDE      "smooth side"        // Smooth width changes

#define PRV_SCALE_BRAKE      "scale brake"        // Scale brake force
#define PRV_SCALE_BUMP       "scale bump"         // Scale bump detection inside
#define PRV_SCALE_BUMPOUTER  "scale bump outer"   // Scale bump detection outside
#define PRV_SCALE_MU         "scale mu"           // Scale friction calculation
#define PRV_SCALE_MU_TRAFFIC "scale mu traffic"   // Scale friction calculation
#define PRV_SCALE_BRAKE_TRAFFIC "scale brake traffic"   // Scale brake calculation
#define PRV_SCALE_FRICTION	 "scale friction"     // Scale friction calculation
#define PRV_SCALE_BRAKING	 "scale braking"      // Scale brake calculation
#define PRV_MAX_BRAKING	     "max braking"        // Max brake

#define PRV_SCALE_MIN_MU     "scale min mu"
#define PRV_SCALE_STEER	     "scale steer"

#define PRV_MIN_SCALE_BRAKE  "min scale brake"
#define PRV_MIN_SCALE_MU     "min scale mu"

#define PRV_SLOW_SPEED		 "slow speed"
#define PRV_FLY_HEIGHT		 "fly height"
#define PRV_AVOID_SCALE		 "avoid scale"
#define PRV_AVOID_WIDTH		 "avoid width"
#define PRV_STAY_TOGETHER	 "stay together"
#define PRV_LENGTH_MARGIN	 "length margin"

#define PRV_DIST_TO_TRAFFIC  "dist to traffic"
#define PRV_NO_AVOIDLENGTH	 "no avoid"
#define PRV_START_SIDE	     "start side"
#define PRV_SHOW_INDEX	     "show index"
#define PRV_SAVE_PLOT	     "save plot"
#define PRV_SAVE_SECTIONS    "save sect"
#define PRV_SAVE_STAT        "save stat"
#define PRV_MIN_LAPS         "min laps"
#define PRV_MAX_FUEL         "max fuel"
#define PRV_MAX_DAMAGE       "max damage"
#define PRV_START_FUEL       "start fuel"
#define PRV_NEXT_FUEL        "next fuel"

#define PRV_BORDER_INNER     "border inner"
#define PRV_BORDER_OUTER     "border outer"
#define PRV_MAX_BORDER_INNER "border inner max"
#define PRV_BORDER_SCALE     "border scale"
#define PRV_FLY_HEIGHT		 "fly height"
#define PRV_LOOKAHEAD        "lookahead"
#define PRV_LOOKAHEADFACTOR  "lookaheadfactor"
#define PRV_OMEGAAHEAD       "omegaahead"
#define PRV_OMEGAAHEADFACTOR "omegaaheadfactor"
#define PRV_INIT_BRAKE       "initial brake"     // Scale brake coeff

#define PRV_TCL_RANGE        "tcl range"         // default 10.0
#define PRV_TCL_SLIP         "tcl slip"          // default 1.6

#define PRV_ABS_DELTA        "abs delta"         // default 1.1
#define PRV_ABS_SCALE        "abs scale"         // default 0.5

#define PRV_CLUTCH_MAX       "clutch max"        // default 0.5
#define PRV_CLUTCH_DELTA     "clutch delta"      // default 0.05
#define PRV_CLUTCH_RANGE     "clutch range"      // default 0.82
#define PRV_CLUTCH_RELEASE   "clutch release"    // default 0.4

#define PRV_TEAM_ENABLE      "team enable"       // default 1

#define PRV_USE_FILTERDRIFT  "filter drifting"   // default 1
#define PRV_USE_FILTERTRACK  "filter track"      // default 1
#define PRV_USE_FILTERTCL    "filter tcl"        // default 1
#define PRV_USE_FILTERDIFF   "filter diff"       // default 0
#define PRV_USE_FILTERRELAX  "filter relax"      // default 0

#define PRV_TRACK_PARAMCOUNT "track param count" // default 0
#define PRV_SEC_PARAMCOUNT   "section param count" // default 0
#define PRV_ARR_PARAMSEG     "param seg" 
#define PRV_ARR_QPARAMSEG    "qualify param seg" 
#define PRV_ARR_PARAMSEC     "param sec" 
#define PRV_ARR_QPARAMSEC    "qualify param sec" 
#define PRM_DIST_FROM_START  "dist from start"
#define PRM_BORDER_LEFT      "border left"
#define PRM_BORDER_RIGHT     "border right"
#define PRM_FACTOR           "factor"
#define PRM_APEX             "apex"
#define PRM_SCALE_FRICTION   "scale friction"
#define PRM_SCALE_BRAKING    "scale braking"
#define PRM_SCALE_BUMPS      "scale bumps"
#define PRM_TARGET_SPEED     "target speed"
// ... Parameters of this robot

// Parameter candidates ...
#define AVG_KEEP (0.75)
#define AVG_CHANGE (1 - AVG_KEEP)

#define LENGTH_MARGIN (1.0f)
#define SIDE_MARGIN (0.5)

#define MAX_SPEED_CRV 0.00175 // R = 571,428 m

#define DELTA_T 0.0001

#define ANALYSE_STEPS 2

#define UNSTUCK_COUNTER 90
//#define UNSTUCK_COUNTER 60
#define NOAVOIDLENGTH -2000

//#define MAXBLOCKED 9
#define MAXBLOCKED 5
// ... Parameter candidates
//==========================================================================*
#endif // _UNITGOBAL_H_
//--------------------------------------------------------------------------*
// end of file unitglobal.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
