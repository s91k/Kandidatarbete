//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitdriver.cpp
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter für TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Zentrale Klasse für das Fahren bzw. den Fahrer/Roboter
//
// Datei    : unitdriver.cpp
// Erstellt : 25.11.2007
// Stand    : 26.08.2012
// Copyright: © 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.07.000 (Championship 2012 Steet-1)
//--------------------------------------------------------------------------*
// Ein erweiterter TORCS-Roboters
//--------------------------------------------------------------------------*
// Teile dieser Unit basieren auf diversen Header-Dateien von TORCS
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
#include <tmath/v2_t.h>
//#include <v2_t.h>
#include <tgf.h>
#include <robottools.h>

#include "unitglobal.h"
#include "unitcommon.h"

#include "unitcollision.h"
#include "unitdriver.h"
#include "unitlinalg.h"
#include "unitparabel.h"
#include "unitpit.h"
#include "unitstrategy.h"
#include "unittrack.h"

//==========================================================================*
// Statics
//--------------------------------------------------------------------------*
int TDriver::NBBOTS = MAX_NBBOTS;                    // Nbr of drivers/robots
char* TDriver::MyBotName = (char *) "wdbee_2012";    // Name of this bot
char* TDriver::ROBOT_DIR = (char *) "drivers/wdbee_2012"; // Sub path to dll
char* TDriver::SECT_PRIV = (char *) "wdbee private"; // Private section
char* TDriver::DEFAULTCARTYPE  = (char *) "car6-trb1"; // Default car type
bool  TDriver::AdvancedParameters = false;           // Advanced parameters

float TDriver::FlyLevel;
float TDriver::FlySection;
float TDriver::RLFactor;
float TDriver::MaxCrvAngle;
double TDriver::Apex;
double TDriver::LengthMargin;
double BasicLengthMargin;

bool TDriver::Qualifying;

static char *WheelSect[4] =
{(char *) SECT_FRNTRGTWHEEL,
 (char *) SECT_FRNTLFTWHEEL,
 (char *) SECT_REARRGTWHEEL,
 (char *) SECT_REARLFTWHEEL};

const int NSteps = 50;
float DistLong[NSteps];
//==========================================================================*

//==========================================================================*
// Buffers
//--------------------------------------------------------------------------*
#define BUFLEN 256
static char PathToWriteToBuffer[BUFLEN];         // for path we write to
static char PathFilenameBuffer[BUFLEN];          // for path and filename
static char PathStatFilenameBuffer[BUFLEN];      // for path and filename
static char TrackNameBuffer[BUFLEN];             // for track name
static char TrackLoadBuffer[BUFLEN];             // for track filename F
static char TrackLoadBufferLearned[BUFLEN];      // for track filename lrn
static char TrackLoadBufferLearnedXML[BUFLEN];   // for track filename xml
static char TrackLoadQualifyBuffer[BUFLEN];      // for track filename F
static char TrackLoadLeftBuffer[BUFLEN];         // for track filename L
static char TrackLoadRightBuffer[BUFLEN];        // for track filename R
static char PitLoadBuffer[BUFLEN];               // for pit track filename F
static char PitLoadLeftBuffer[BUFLEN];           // for pit track filename L
static char PitLoadRightBuffer[BUFLEN];          // for pit track filename R
static char PathLocalRobotDir[BUFLEN];           // for path and filename
//==========================================================================*

//==========================================================================*
// Interpolation for point Q between two points P0 and P1 (Result in P0)
//--------------------------------------------------------------------------*
void TDriver::InterpolatePointInfo
  (TLanePoint& P0, const TLanePoint& P1, double Q)
{
  double DeltaOAngle = P1.Angle - P0.Angle;

  P0.Crv = TUtils::InterpCurvature(P0.Crv, P1.Crv, (1 - Q));
  DOUBLE_NORM_PI_PI(DeltaOAngle);
  P0.Angle = P0.Angle + DeltaOAngle * (1 - Q);
  P0.Offset = Q * P0.Offset + (1 - Q) * P1.Offset;
  P0.Speed = Q * P0.Speed + (1 - Q) * P1.Speed;
}
//==========================================================================*

//==========================================================================*
// Constructor
//--------------------------------------------------------------------------*
TDriver::TDriver(int Index):
  CurrSimTime(0.0),
  oCommonData(NULL),
  oFirst(true),
  oSecond(false),
  oFlyHeight(0.06),
  oBumpMod(0.35f),
  oScaleSteer(0.0),
  oStayTogether(0.0),
  oAvoidScale(0.0),
  oAvoidWidth(0.0),
  oGoToPit(false),
  oTeam(NULL),

  oDriveTrainType(cDT_RWD),
  oFlying(0),
  oLastRideHeight(0.1f),
  oNbrCars(0),
  oOwnOppIdx(0),

  oAvoidRange(0.5),
  oAvoidRangeDelta(0.0),
  oAvoidOffset(0.0),
  oAvoidOffsetDelta(0.0),
  oMaxAccel(0, 100, 101, 1),
  oLastBrakeCoefIndex(0),
  oLastBrake(0.0),
  oLastTargetSpeed(0.0),

  oAccel(0),
  oLastAccel(0),
  oBrake(0),
  oClutch(0.5),
  oGear(0),
  oSteer(0),
  oClutchDelta(0.05),
  oClutchMax(0.5),
  oClutchRange(0.82),
  oClutchRelease(0.345),
  oAbsDelta(1.1),
  oAbsScale(0.5),
  oAlone(true),
  oAngle(0),
  oAngleSpeed(0),
  oBotName(NULL),
  oBrakeDiffInitial(2.0),
  oBrakeForceMax(0.5),
  oBrakeScale(72.0),
  oInitialBrakeCoeff(0.5),
  oCar(NULL),
  oSteerAngle(0.0f),
  oCarType(NULL),
  oCurrSpeed(0),
  oIndex(0),
  oLastGear(0),
  oLetPass(false),
  oLookAhead(5.0),
  oLookAheadFactor(0.2),
  oLookScale(0.2),
  oLookBase(5.0),
  oOmegaBase(5.0),
  oOmegaScale(0.2),
  oOmegaAheadFactor(2.0),
  oOmegaAhead(5.0),
  oDistFromStart(0.0),
  oShiftMargin(0),
  oSituation(NULL),
  oStartDistance(50.0),
  oStuckCounter(0),
  oSysFooStuckX(NULL),
  oSysFooStuckY(NULL),
  oTestPitStop(0),
  oTrackAngle(0.0),
  oTargetSpeed(100.0),
  oTargetDelta(0.0),
  oTclRange(10.0),
  oTclSlip(1.6),
  oTrackName(NULL),
  oTrack(NULL),
  oTolerance(2.0),
  oSmoothSide(0.22f),
  oUnstucking(false),
  oWheelRadius(0),
  oDeltaOffset(0.0),
  oDriftAngle(0.0),
  oLetPassSide(0),
  oOldTarget(0.0),
  oReduced(false),
  oShowIndex(false),
  oSavePlot(false),
  oSaveSections(false),
  oNoAvoidLength(NOAVOIDLENGTH),
  oStartRPM(100.0),
  oStartSide(0.0),
  oFuelNeeded(0.0),
  oRepairNeeded(0.0),
  oSideReduction(1.0),
  oMinDistLong(FLT_MAX),
  oLastCarLaps(1),
  oCarHandle(NULL),
  oStrategy(NULL),
  oStanding(true),
  oMaxFuel(100.0),
  oMinLaps(3),
  oSpeedScale(0.0),
  oTreatTeamMateAsLapper(false),
  oBrakeForTeamMate(false),
  oBrakeTeamMateAside(false),
  oTurnCounter(0),
  oLastSteer(0.0),
  oMinScaleBrake(0.95f),
  oMinScaleMu(0.97f),
  oScaleMuTraffic(1.0),
  oScaleBrakeTraffic(1.0),
  oDistToTraffic(0.0),
  oSlowSpeed(5.0f),
  oScaleRefuel(1.10f),
  oLastTarget(0.0),
  oSideCounter(0),
  oOppsNearBehind(0),
  oOppsNearInFront(0),
  oLastNearInFront(0),
  oAvoidSide(0),
  oPreviewSlow(false),
  oUseFilterDrifting(true),
  oUseFilterTrack(true),
  oUseFilterTCL(true),
// oUseFilterDiff(false),
  oUseFilterDiff(true),
  oUseFilterRelax(false)
{
//  GfOut("TDriver::TDriver() >>>\n");
  int I;
  oIndex = Index;                                // Save own index

  // Motion survey
//  oSysFooStuckX = new TSysFoo(1,128);            // Ringbuffer for X
//  oSysFooStuckY = new TSysFoo(1,128);            // and Y coordinates
  oSysFooStuckX = new TSysFoo(1,32);             // Ringbuffer for X
  oSysFooStuckY = new TSysFoo(1,32);             // and Y coordinates

  for (I = 0; I < NBR_BRAKECOEFF; I++)           // Initialize braking
    oBrakeCoeff[I] = oInitialBrakeCoeff;

  NBRRL = gNBR_RL;                               // Setup number
  oRL_FREE = RL_FREE;                            // and index for
  oRL_LEFT = RL_LEFT;                            // normal races
  oRL_RIGHT = RL_RIGHT;

  TDriver::FlyLevel = 0.031f;
  TDriver::FlySection = 0.0f;
  TDriver::RLFactor = 1.015f;
  TDriver::MaxCrvAngle = 0.6f;
  TDriver::Apex = 1.0;
  TDriver::LengthMargin = LENGTH_MARGIN;
  BasicLengthMargin = LENGTH_MARGIN;

  Param.Fix.oScaleBrake = HALFFRICTION;
  Param.Fix.oScaleBumps = 1.00f;
  Param.Fix.oScaleSpeed = HALFFRICTION;

  oLastLap = 1;
  oLapsLearned = 0;
  oCorrFactor = 1.0;
  oMinBrakeFrictionFactor = 0.95;
  oMaxBrakeFrictionFactor = 2.0;
  oMinSpeedFrictionFactor = 0.95;
  oMaxSpeedFrictionFactor = 3.0;
  oExcludeFrom = -1;
  oExcludeTill = -1;

  oLoadLearned = false;
  oLoadedLearned = false;

  oStartSteerFactor = 0.0;                       // Steer speed at start

//  GfOut("<<< TDriver::TDriver()\n");
}
//==========================================================================*

//==========================================================================*
// destructor
//--------------------------------------------------------------------------*
TDriver::~TDriver()
{
//  GfOut("TDriver::~TDriver() >>>\n");
  delete [] oOpponents; 

  free(oCarType);
  free(oTeamName);

  if (oStrategy != NULL)
    delete oStrategy;
  if (oSysFooStuckX != NULL)
    delete oSysFooStuckX;
  if (oSysFooStuckY != NULL)
    delete oSysFooStuckY;
//  GfOut("<<< TDriver::~TDriver()\n");
}
//==========================================================================*

//==========================================================================*
// Set name of robot (an other appendant features)
//--------------------------------------------------------------------------*
void TDriver::SetBotName(char* Value)
{
  // At this point TORCS gives us no information
  // about the name of the driver, the team and
  // our own car type!
  // Because we want it to set the name as defined
  // in the teams xml file and to load depending
  // setup files we have to find it out:

  // Initialize the base param path.
  char* PathFilename = PathFilenameBuffer;

  snprintf(PathFilenameBuffer,BUFLEN,            // Build path to
    "drivers/%s/%s.xml"                          // own robot dll from
	,MyBotName,MyBotName);                       // name of own robot

  char SectionBuffer[256];                       // Buffer
  snprintf(SectionBuffer,BUFLEN,                 // Build name of
    "%s/%s/%d"                                   // section from
	,ROB_SECT_ROBOTS,ROB_LIST_INDEX,oIndex);     // Index of own driver
  char* Section = SectionBuffer;

  void* RobotSettings = GfParmReadFile           // Open team setup file
    (PathFilename,GFPARM_RMODE_STD);

  GfOut("\n\n\nPathFilename: %s\n",PathFilename);
  if (RobotSettings)
  {
	oCarType = strdup(GfParmGetStr               // Get pointer to
      (RobotSettings                             // car type
      , Section                                  // defined in corresponding
      , ROB_ATTR_CAR                             // section,
      , DEFAULTCARTYPE));                        // default car type

	oBotName = Value;                            // Get pointer to drivers name

	oTeamName = strdup(GfParmGetStr              // Get pointer to
      (RobotSettings                             // drivers team name
      , Section                                  // defined in corresponding
      , (char *) ROB_ATTR_TEAM, oCarType));      // section, car type as default

	oRaceNumber = (int) GfParmGetNum             // Get pointer to
      (RobotSettings                             // race number
      , Section, (char *) ROB_ATTR_RACENUM       // defined in corresponding
      , (char *) NULL, (tdble) oIndex + 1);      // section, index as default
  }
  else
  {
	GfOut("\n\n FATAL ERROR: File '%s' not found\n\n",PathFilename);
	oCarType = strdup(DEFAULTCARTYPE);
	oBotName = Value;
	oTeamName = strdup(oCarType);
	oRaceNumber = oIndex + 1;
  }
  GfOut("Bot name: %s\n",oBotName);
  GfOut("Team name: %s\n",oTeamName);
  GfOut("Car type: %s\n",oCarType);
  GfOut("Race number: %d\n",oRaceNumber);

};
//==========================================================================*

//==========================================================================*
// Set path and filenames for racinglines
//--------------------------------------------------------------------------*
void TDriver::SetPathAndFilenameForRacinglines()
{
  const char* PathToWriteTo = GetLocalDir();

  snprintf(PathToWriteToBuffer,sizeof(TrackLoadBuffer),
	"%sdrivers/%s/tracks",
    PathToWriteTo,MyBotName);
  oPathToWriteTo = PathToWriteToBuffer;
  GfOut("\n\n\n");
  GfOut("#oPathToWriteTo: >%s<\n",oPathToWriteTo);
  if (GfCreateDir(oPathToWriteTo) == GF_DIR_CREATION_FAILED)
  {
	  GfOut("#Unable to create path for racinglines: >%s<",oPathToWriteTo);
  };
  GfOut("\n\n\n");

  snprintf(TrackLoadBufferLearned,sizeof(TrackLoadBufferLearned),"%s/%s.lrn",
    oPathToWriteTo,oTrackName);
  oTrackLoadLearned = TrackLoadBufferLearned;    // Set pointer to buffer

  snprintf(TrackLoadBufferLearnedXML,sizeof(TrackLoadBufferLearnedXML),"%s/%s.lrn",
    oPathToWriteTo,oTrackName);
  oTrackLoadLearnedXML = TrackLoadBufferLearnedXML;    // Set pointer to buffer

  snprintf(TrackLoadBuffer,sizeof(TrackLoadBuffer),"%s/%s.trk",
    oPathToWriteTo,oTrackName);
  oTrackLoad = TrackLoadBuffer;                  // Set pointer to buffer

  snprintf(TrackLoadQualifyBuffer,sizeof(TrackLoadQualifyBuffer),
	"%s/%s.trq",oPathToWriteTo,oTrackName);
  oTrackLoadQualify = TrackLoadQualifyBuffer;    // Set pointer to buffer

  snprintf(TrackLoadLeftBuffer,sizeof(TrackLoadLeftBuffer),"%s/%s.trl",
    oPathToWriteTo,oTrackName);
  oTrackLoadLeft = TrackLoadLeftBuffer;          // Set pointer to buffer

  snprintf(TrackLoadRightBuffer,sizeof(TrackLoadRightBuffer),"%s/%s.trr",
    oPathToWriteTo,oTrackName);
  oTrackLoadRight = TrackLoadRightBuffer;        // Set pointer to buffer

  snprintf(PitLoadBuffer,sizeof(PitLoadBuffer),"%s/%s.tpk",
    oPathToWriteTo,oTrackName);
  oPitLoad[0] = PitLoadBuffer;                   // Set pointer to buffer

  snprintf(PitLoadLeftBuffer,sizeof(PitLoadLeftBuffer),"%s/%s.tpl",
    oPathToWriteTo,oTrackName);
  oPitLoad[1] = PitLoadLeftBuffer;               // Set pointer to buffer

  snprintf(PitLoadRightBuffer,sizeof(PitLoadRightBuffer),"%s/%s.tpr",
    oPathToWriteTo,oTrackName);
  oPitLoad[2] = PitLoadRightBuffer;              // Set pointer to buffer
};
//==========================================================================*

//==========================================================================*
// Called for every track change or new race.
// CarHandle is a file handle to the merged file of car category and cartype
// CarSettings points to a a variable for a file handle with the car setup
//--------------------------------------------------------------------------*
void TDriver::InitTrack
  (PTrack Track, PCarHandle CarHandle,
  PCarSettings *CarSettings, PSituation Situation)
{
  oTrack = Track;                                // save pointers
  oSituation = Situation;

  // Initialize race type array
  char* RaceType[] =
    {(char*)"practice", (char*)"qualify", (char*)"race"};

  // Initialize the base param path.
  char* BaseParamPath = (char*) TDriver::ROBOT_DIR;
  //char* PathFilename = (char*) PathFilenameBuffer;

  // Get the name of the track
  strncpy(TrackNameBuffer,                       // Copy name of track file
    strrchr(oTrack->filename, '/') + 1,          // from path and filename
	sizeof(TrackNameBuffer));                    // regarding length of buffer
  *strrchr(TrackNameBuffer, '.') = '\0';         // Truncate at point
  oTrackName = TrackNameBuffer;                  // Set pointer to buffer

  // Read/merge car parms
  // First all params out of the common files
  oMaxFuel = GfParmGetNum(CarHandle              // Tank capacity of car type
    , (char*) SECT_CAR, (char*) PRM_TANK
    , (char*) NULL, 100.0);
  GfOut("#oMaxFuel (TORCS)   = %.1f\n",oMaxFuel);

  PCarHandle Handle = NULL;                      // initialize it!
  char Buf[1024];                                // Multi purpose buffer

  // Default params for car type (e.g. .../ROBOT_DIR/sc-petrol/default.xml)
  snprintf(Buf,sizeof(Buf),"%s/%s/default.xml",
    BaseParamPath,oCarType);
  Handle = TUtils::MergeParamFile(Handle,Buf);

  SetPathAndFilenameForRacinglines();

  // Override params for track (Pitting)
  snprintf(Buf,sizeof(Buf),"%s/tracks/%s.xml",
    BaseParamPath,oTrackName);
  Handle = TUtils::MergeParamFile(Handle,Buf);

  // Override params for car type with params of track
  snprintf(Buf,sizeof(Buf),"%s/%s/%s.xml",
    BaseParamPath,oCarType,oTrackName);
  Handle = TUtils::MergeParamFile(Handle,Buf);

  // Override params for car type on track with params of specific race type
  snprintf(Buf,sizeof(Buf),"%s/%s/%s-%s.xml",
    BaseParamPath,oCarType,oTrackName,RaceType[oSituation->_raceType]);
  Handle = TUtils::MergeParamFile(Handle,Buf);

  // Override params for car type on track with params of specific race type and driver
  snprintf(Buf,sizeof(Buf),"%s/%s/%s-%d-%s.xml",
    BaseParamPath,oCarType,oTrackName,oIndex,RaceType[oSituation->_raceType]);
  Handle = TUtils::MergeParamFile(Handle,Buf);

  // Setup the car param handle to be returned
  *CarSettings = Handle;

  // Get the private parameters now.
  oLoadLearned = 
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_LOAD_LEARNED,0,oLoadLearned) == 1;
  if (oLoadLearned)
    GfOut("#LoadLearned = true\n");
  else
    GfOut("#LoadLearned = false\n");

  oMinBrakeFrictionFactor = 0.95;
  oMaxBrakeFrictionFactor = 2.0;
  oMinSpeedFrictionFactor = 0.95;
  oMaxSpeedFrictionFactor = 3.0;

  oMinBrakeFrictionFactor = 
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_MIN_BRAKEFF,0,(float) oMinBrakeFrictionFactor);
  GfOut("#MinBrakeFrictionFactor %f\n",oMinBrakeFrictionFactor);

  oMaxBrakeFrictionFactor = 
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_MAX_BRAKEFF,0,(float) oMaxBrakeFrictionFactor);
  GfOut("#MaxBrakeFrictionFactor %f\n",oMaxBrakeFrictionFactor);

  oMinSpeedFrictionFactor = 
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_MIN_SPEEDFF,0,(float) oMinSpeedFrictionFactor);
  GfOut("#MinSpeedFrictionFactor %f\n",oMinSpeedFrictionFactor);

  oMaxSpeedFrictionFactor = 
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_MAX_SPEEDFF,0,(float) oMaxSpeedFrictionFactor);
  GfOut("#MaxSpeedFrictionFactor %f\n",oMaxSpeedFrictionFactor);

  oExcludeFrom = (int)
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_EX_FROM,0,(float) oExcludeFrom);
  GfOut("#Exclude from %d\n",oExcludeFrom);

  oExcludeTill = (int)
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_EX_TILL,0,(float) oExcludeTill);
  GfOut("#Exclude till %d\n",oExcludeTill);

  TDriver::FlyLevel = 
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_FLY_LEVEL,0,TDriver::FlyLevel);
  GfOut("#FlyLevel %f\n",TDriver::FlyLevel);

  TDriver::FlySection = 
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_FLY_SECTION,0,TDriver::FlySection);
  GfOut("#FlySection %f\n",TDriver::FlySection);

  TDriver::RLFactor =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_FACTOR,0,TDriver::RLFactor);
  GfOut("#RLFactor %f\n",TDriver::RLFactor);

  TDriver::MaxCrvAngle =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_MAX_CRVANGLE,0,TDriver::MaxCrvAngle);
  GfOut("#MaxCrvAngle %f\n",TDriver::MaxCrvAngle);

  TDriver::Apex =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_APEX,0,(float) TDriver::Apex);
  GfOut("#Apex %f\n",TDriver::Apex);

  TDriver::LengthMargin =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_LENGTH_MARGIN,0,LENGTH_MARGIN);
  BasicLengthMargin = TDriver::LengthMargin;

  int TestQualification =
	(int) GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_QUALIFICATION,0,0);
  if ((oSituation->_raceType == RM_TYPE_QUALIF)
	|| (TestQualification > 0))
  {
	Qualifying = true;
	GfOut("Qualifying = True\n");
	NBRRL = 1;
  }

  Param.Fix.oLength =
	GfParmGetNum(Handle,(char*)SECT_CAR,(char*)PRM_LEN,(char*)NULL,4.5);

  // Adjust pitting ...
  Param.Pit.oUseFirstPit = (int)
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_USE_FIRST,0,1);
  //GfOut("oUseFirstPit %d\n",Param.Pit.oUseFirstPit);

  Param.Pit.oUseSmoothPit = (int)
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_USE_SMOOTH,0,1);
  //GfOut("oUseSmoothPit %d\n",Param.Pit.oUseSmoothPit);

  Param.Pit.oLaneEntryOffset =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PITLANE_ENTRY,0,3.0);
  //GfOut("oLaneEntryOffset %g\n",Param.Pit.oLaneEntryOffset);

  Param.Pit.oLaneExitOffset =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PITLANE_EXIT,0,5.0);
  //GfOut("oLaneExitOffset %g\n",Param.Pit.oLaneExitOffset);

  Param.Pit.oEntryLong =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_ENTRY_LONG,0,0);
  //GfOut("oEntryLong %g\n",Param.Pit.oEntryLong);

  Param.Pit.oExitLong =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_EXIT_LONG,0,0);
  //GfOut("oExitLong %g\n",Param.Pit.oExitLong);

  Param.Pit.oExitLength =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_EXIT_LEN,0,0);
  //GfOut("oExitLength %g\n",Param.Pit.oExitLength);

  Param.Pit.oLatOffset =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_LAT_OFFS,0,0.0);
  //GfOut("Lateral Pit Offset %f\n",Param.Pit.oLatOffset);

  Param.Pit.oLongOffset =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_LONG_OFFS,0,0.0);
  //GfOut("Longitudinal Pit  Offset %f\n",Param.Pit.oLongOffset);

  Param.oCarParam.oScaleBrakePit =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_SCALE_BRAKE,0,
	(float) MIN(1.0,Param.oCarParam.oScaleBrake));
  //GfOut("PitScaleBrake %g\n",Param.oCarParam.oScaleBrakePit);

  Param.Pit.oStoppingDist =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_STOP_DIST,0,1.5);
  //GfOut("oStoppingDist %g\n",Param.Pit.oStoppingDist);

  Param.Fix.oPitBrakeDist =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_BRAKE_DIST,0,150.0);
  //GfOut("oPitBrakeDist %g\n",Param.Fix.oPitBrakeDist);

  oTestPitStop = (int)
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_PIT_TEST_STOP,0,0);
  //GfOut("TestPitStop %d\n",oTestPitStop);
  // ... Adjust pitting

  // Adjust driving ...
  oSmoothSide =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SMOOTH_SIDE,NULL,oSmoothSide);
  //GfOut("Smooth side: %g\n",oSmoothSide);

  Param.oCarParam.oScaleBrake =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_BRAKE,NULL,0.85f);
  //GfOut("Scale Brake: %g\n",Param.oCarParam.oScaleBrake);

  Param.oCarParam.oScaleBump =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_BUMP,NULL,0.10f);
  Param.oCarParam.oScaleBumpLeft =
    Param.oCarParam.oScaleBump;
  Param.oCarParam.oScaleBumpRight =
    Param.oCarParam.oScaleBump;
  //GfOut("Scale Bump: %g\n",Param.oCarParam.oScaleBump);

  Param.oCarParam.oScaleBumpOuter =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_BUMPOUTER,NULL,(float) Param.oCarParam.oScaleBump);
  //GfOut("Scale Bump Outer: %g\n",Param.oCarParam.oScaleBumpOuter);

  Param.oCarParam.oScaleMu =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_MU,NULL,1.00f);
  //GfOut("Scale Mu: %g\n",Param.oCarParam.oScaleMu);

  oScaleMuTraffic =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_MU_TRAFFIC,NULL,Param.oCarParam.oScaleMu);
  //GfOut("Scale Mu Traffic: %g\n",oScaleMuTraffic);

  oScaleBrakeTraffic =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_BRAKE_TRAFFIC,NULL,(float) Param.oCarParam.oScaleBrake);
  //GfOut("Scale Brake Traffic: %g\n",oScaleBrakeTraffic);

  oDistToTraffic =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_DIST_TO_TRAFFIC,NULL,0.0);
  //GfOut("Dist to traffic: %g\n",oDistToTraffic);

  Param.oCarParam.oScaleMinMu =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_MIN_MU,NULL,0.8f);
  //GfOut("Scale Min Mu %g\n",Param.oCarParam.oScaleMinMu);

  Param.Fix.oAeroModel =
	(int) GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_AERO_MOD,0,0);
  //GfOut("Aero Model %d\n",Param.Fix.oAeroModel);

  oAvoidScale =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_AVOID_SCALE,0,10.0);
  //GfOut("oAvoidScale %g\n",oAvoidScale);

  oAvoidWidth =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_AVOID_WIDTH,0,1.5);
  //GfOut("oAvoidWidth %g\n",oAvoidWidth);

  oMinScaleBrake =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_MIN_SCALE_BRAKE,0,oMinScaleBrake);
  //GfOut("oMinScaleBrake %g\n",oMinScaleBrake);

  oMinScaleMu =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_MIN_SCALE_MU,0,oMinScaleMu);
  //GfOut("oMinScaleMu %g\n",oMinScaleMu);


  Param.Fix.oBorderOuter =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_BORDER_OUTER,0,Param.Fix.oBorderOuter);
  //GfOut("Border Outer: %g\n",Param.Fix.oBorderOuter);

  Param.Fix.oMaxBorderInner =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_MAX_BORDER_INNER,0,Param.Fix.oMaxBorderInner);
  //GfOut("Max Border Inner: %g\n",Param.Fix.oMaxBorderInner);

  Param.Fix.oBorderScale =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_BORDER_SCALE,0,Param.Fix.oBorderScale);
  //GfOut("Border Scale: %g\n",Param.Fix.oBorderScale);

  Param.Fix.oBorderInner =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_BORDER_INNER,0,Param.Fix.oBorderInner);
  //GfOut("Border Inner: %g\n",Param.Fix.oBorderInner);

  oBumpMod =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_BUMP_MOD,0,oBumpMod);
  //GfOut("oBumpMod %g\n",oBumpMod);

  oFlyHeight =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_FLY_HEIGHT,(char*)"m",0.06f);
  //GfOut("FLY_HEIGHT %g\n",oFlyHeight);

  oLookAhead =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_LOOKAHEAD,0,(float) Param.Fix.oLength);
  //GfOut("LookAhead %g\n",oLookAhead);

  oLookAheadFactor =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_LOOKAHEADFACTOR,0,(float)oLookAheadFactor);
  //GfOut("LookAheadFactor %g\n",oLookAheadFactor);

  oOmegaAhead =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_OMEGAAHEAD,0,(float) Param.Fix.oLength);
  //GfOut("OmegaAhead %g\n",oOmegaAhead);

  oOmegaAheadFactor =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_OMEGAAHEADFACTOR,0,(float)(oLookAheadFactor * 10));
  //GfOut("OmegaAheadFactor %g\n",oOmegaAheadFactor);

  oScaleSteer =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_STEER,0,0.8f);
  //GfOut("oScaleSteer %g\n",oScaleSteer);

  oStayTogether =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_STAY_TOGETHER,0,0);
  //GfOut("oStayTogether %g\n",oStayTogether);

  oInitialBrakeCoeff =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_INIT_BRAKE,0,oBrakeCoeff[0]);
  //GfOut("oInitialBrakeCoeff %g\n",oInitialBrakeCoeff);

  oSlowSpeed =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SLOW_SPEED,0,oSlowSpeed);
  //GfOut("oSlowSpeed %g\n",oSlowSpeed);

  oScaleRefuel =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_REFUEL,0,oScaleRefuel);
  //GfOut("oScaleRefuel %g\n",oScaleRefuel);

  oMinLaps = (int)
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_MIN_LAPS,0, (float) oMinLaps);
  //GfOut("#oMinLaps %d\n",oMinLaps);

  Param.Fix.oScaleBrake =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_BRAKE2,0,Param.Fix.oScaleBrake);
  GfOut("# Scale brake         = %.3f\n",Param.Fix.oScaleBrake);

  Param.Fix.oScaleBumps =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_BUMPS2,0,Param.Fix.oScaleBumps);
  GfOut("# Scale bumps         = %.3f\n",Param.Fix.oScaleBumps);

  Param.Fix.oScaleSpeed =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SCALE_SPEED2,0,Param.Fix.oScaleSpeed);
  GfOut("# Scale speed         = %.3f\n",Param.Fix.oScaleSpeed);

  // Initialize braking
  //if (!LoadBrakeCoeffsFromFile())
    for (int I = 0; I < NBR_BRAKECOEFF; I++)
      oBrakeCoeff[I] = oInitialBrakeCoeff;

  oNoAvoidLength =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_NO_AVOIDLENGTH,0,(float)oNoAvoidLength);
  //GfOut("oNoAvoidLength %g\n",oNoAvoidLength);

  oStartSide =
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_START_SIDE,0,(float)oStartSide);
  //GfOut("oStartSide %g\n",oStartSide);

  oShowIndex = (1 == (int)
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SHOW_INDEX,0,(float) oShowIndex));
  if (oShowIndex)
    GfOut("oShowIndex True\n");

  oSavePlot = (1 == (int)
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SAVE_PLOT,0,(float) oSavePlot));
  if (oSavePlot)
    GfOut("oSavePlot True\n");

  oSaveSections = (1 == (int)
	GfParmGetNum(Handle,SECT_PRIV,(char*)PRV_SAVE_SECTIONS,0,(float) oSavePlot));
  if (oSaveSections)
    GfOut("oSaveSections True\n");

  oTclRange =
	GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_TCL_RANGE,0,(float)oTclRange);
  //GfOut("oTclRange %g\n",oTclRange);

  oTclSlip =
	GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_TCL_SLIP,0,(float)oTclSlip);
  //GfOut("oTclSlip %g\n",oTclSlip);

  oAbsDelta =
	GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_ABS_DELTA,0,(float)oAbsDelta);
  //GfOut("oAbsDelta %g\n",oAbsDelta);

  oAbsScale =
	GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_ABS_SCALE,0,(float)oAbsScale);
  //GfOut("oAbsScale %g\n",oAbsScale);

  oClutchDelta =
	GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_CLUTCH_DELTA,0,(float)oClutchDelta);
  //GfOut("oClutchDelta %g\n",oClutchDelta);

  oClutchMax =
	GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_CLUTCH_MAX,0,(float)oClutchMax);
  //GfOut("oClutchMax %g\n",oClutchMax);

  oClutchRange =
	GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_CLUTCH_RANGE,0,(float)oClutchRange);
  //GfOut("oClutchRange %g\n",oClutchRange);

  oClutchRelease =
	GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_CLUTCH_RELEASE,0,(float)oClutchRelease);
  //GfOut("oClutchRelease %g\n",oClutchRelease);

  oUseFilterDrifting = 
    GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_USE_FILTERDRIFT,0,(float)oUseFilterDrifting) != 0;
  //GfOut("UseFilterDrifting %d\n",oUseFilterDrifting);

  oUseFilterTrack =
    GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_USE_FILTERTRACK,0,(float)oUseFilterTrack) != 0;
  //GfOut("UseFilterTrack %d\n",oUseFilterTrack);

  oUseFilterTCL =
    GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_USE_FILTERTCL,0,(float)oUseFilterTCL) != 0;
  //GfOut("UseFilterTCL %d\n",oUseFilterTCL);

  oUseFilterDiff =
    GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_USE_FILTERDIFF,0,(float)oUseFilterDiff) != 0;
  //GfOut("UseFilterDiff %d\n",oUseFilterDiff);

  oUseFilterRelax =
    GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_USE_FILTERRELAX,0,(float)oUseFilterRelax) != 0;
  GfOut("UseFilterRelax %d\n",oUseFilterRelax);
  // ... Adjust driving

  // Adjust track width ...
  int Count = (int)
    GfParmGetNum(Handle,TDriver::SECT_PRIV,(char*)PRV_TRACK_PARAMCOUNT,0,0.0f);
  //GfOut("Count %d\n",Count);

  bool UseSeparateParameters = true;

  for (int L = 0; L < gNBR_RL; L++)
  {
    oRacingLine[L].CreateParamSeg(Count);

    TLane::TParamSeg* ParSeg = NULL;
    TLane::TParamSeg* LastParSeg = NULL;
    char idx[64];

	if (Count == -1)
	{
	  UseSeparateParameters = false;
	  Count = 0;
	}

	if (Count == 0)
	{
      oRacingLine[L].CreateParamSeg(1);
	  ParSeg = oRacingLine[L].GetParamSeg(0);
	  ParSeg->DistFromStart= 0.0;
	  ParSeg->BorderLeft = 0.0;
	  ParSeg->BorderRight = 0.0;
	  ParSeg->Factor = TDriver::RLFactor;
	  ParSeg->Apex = TDriver::Apex;
	  ParSeg->ScaleFriction = 1.0;
	  ParSeg->ScaleBraking = 1.0;
	  ParSeg->ScaleBumps = 1.0;
	  ParSeg->TargetSpeed = 0.0;
 	  ParSeg->Length = 1.0;
      ParSeg->DeltaLeft = 0.0;
      ParSeg->DeltaRight = 0.0;
	}
	else for (int I = 0; I < Count; I++)
    {
	  ParSeg = oRacingLine[L].GetParamSeg(I);

	  if ((Qualifying) && (UseSeparateParameters))
	    sprintf(idx, "%s/%s/%d", TDriver::SECT_PRIV, PRV_ARR_QPARAMSEG, I+1);
	  else
  	    sprintf(idx, "%s/%s/%d", TDriver::SECT_PRIV, PRV_ARR_PARAMSEG, I+1);

	  if (LastParSeg == NULL)
	  {
	    ParSeg->DistFromStart = GfParmGetNum(Handle, idx,
		    (char*) PRM_DIST_FROM_START, (char*) NULL, 0);
		ParSeg->BorderLeft = GfParmGetNum(Handle, idx,
		    (char*) PRM_BORDER_LEFT, (char*) NULL, 0);
	    ParSeg->BorderRight = GfParmGetNum(Handle, idx,
		    (char*) PRM_BORDER_RIGHT, (char*) NULL, 0);
		ParSeg->Factor = GfParmGetNum(Handle, idx,
		    (char*) PRM_FACTOR, (char*) NULL, TDriver::RLFactor);
	    ParSeg->Apex = GfParmGetNum(Handle, idx,
	        (char*) PRM_APEX, (char*) NULL, (float) TDriver::Apex);
	    ParSeg->ScaleFriction = GfParmGetNum(Handle, idx,
		    (char*) PRM_SCALE_FRICTION, (char*) NULL, 1.0f);
	    ParSeg->ScaleBraking = GfParmGetNum(Handle, idx,
		    (char*) PRM_SCALE_BRAKING, (char*) NULL, 1.0f);
		ParSeg->ScaleBumps = GfParmGetNum(Handle, idx,
		    (char*) PRM_SCALE_BUMPS, (char*) NULL, 1.0f);
	    ParSeg->TargetSpeed = GfParmGetNum(Handle, idx,
		    (char*) PRM_TARGET_SPEED, (char*) NULL, 100.0f);
//		if (ParSeg->TargetSpeed == 0)
//			ParSeg->TargetSpeed = 100;
	  }
	  else // if (LastParSeg != NULL)
  	  {
	    ParSeg->DistFromStart = GfParmGetNum(Handle, idx,
		    (char*) PRM_DIST_FROM_START, (char*) NULL, (float) LastParSeg->DistFromStart);
		ParSeg->BorderLeft = GfParmGetNum(Handle, idx,
			(char*) PRM_BORDER_LEFT, (char*) NULL, (float) LastParSeg->BorderLeft);
	      ParSeg->BorderRight = GfParmGetNum(Handle, idx,
		    (char*) PRM_BORDER_RIGHT, (char*) NULL, (float) LastParSeg->BorderRight);
		ParSeg->Factor = GfParmGetNum(Handle, idx,
			(char*) PRM_FACTOR, (char*) NULL, (float) LastParSeg->Factor);
	    ParSeg->Apex = GfParmGetNum(Handle, idx,
	        (char*) PRM_APEX, (char*) NULL, (float) TDriver::Apex);
		ParSeg->ScaleFriction = GfParmGetNum(Handle, idx,
			(char*) PRM_SCALE_FRICTION, (char*) NULL, (float) LastParSeg->ScaleFriction);
	    ParSeg->ScaleBraking = GfParmGetNum(Handle, idx,
		    (char*) PRM_SCALE_BRAKING, (char*) NULL, (float) LastParSeg->ScaleBraking);
		ParSeg->ScaleBumps = GfParmGetNum(Handle, idx,
			(char*) PRM_SCALE_BUMPS, (char*) NULL, (float) LastParSeg->ScaleBumps);
		ParSeg->TargetSpeed = GfParmGetNum(Handle, idx,
			(char*) PRM_TARGET_SPEED, (char*) NULL, (float) LastParSeg->TargetSpeed);

	    LastParSeg->Length = ParSeg->DistFromStart - LastParSeg->DistFromStart;
	    LastParSeg->DeltaLeft = ParSeg->BorderLeft - LastParSeg->BorderLeft;
        LastParSeg->DeltaRight = ParSeg->BorderRight - LastParSeg->BorderRight;
 	    //GfOut("D:%.0f L:%.0f BL:%0.2f BR:%0.2f DL:%0.2f DR:%0.2f F:%0.4f SF:%0.3f SB:%0.3f\n",
	    //  LastParSeg->DistFromStart,LastParSeg->Length,LastParSeg->BorderLeft,LastParSeg->BorderRight,LastParSeg->DeltaLeft,LastParSeg->DeltaRight,LastParSeg->Factor,LastParSeg->ScaleFriction,LastParSeg->ScaleBraking);
	  }
 	  ParSeg->Length = 1.0;
      ParSeg->DeltaLeft = 0.0;
      ParSeg->DeltaRight = 0.0;

	  LastParSeg = ParSeg;
    }

	// Close the loop
    LastParSeg = oRacingLine[L].GetParamSeg(0);
    ParSeg->Length = oTrack->length + LastParSeg->DistFromStart - ParSeg->DistFromStart;
    ParSeg->DeltaLeft = LastParSeg->BorderLeft - ParSeg->BorderLeft;
    ParSeg->DeltaRight = LastParSeg->BorderRight - ParSeg->BorderRight;
    //GfOut("D:%.0f L:%.0f BL:%0.2f BR:%0.2f DL:%0.2f DR:%0.2f F:%0.4f SF:%0.3f SB:%0.3f\n",
    //  ParSeg->DistFromStart,ParSeg->Length,ParSeg->BorderLeft,ParSeg->BorderRight,ParSeg->DeltaLeft,ParSeg->DeltaRight,ParSeg->Factor,ParSeg->ScaleFriction,ParSeg->ScaleBraking);
  }
  // ... Adjust track width

  int SaveStatistics = (int)
	GfParmGetNum(Handle,(char*)SECT_PRIV,(char*)PRV_SAVE_STAT,0,0.0);
  if (SaveStatistics > 0)
    oSaveStatistics = true;
  else
    oSaveStatistics = false;

  // Find side and sections of pits ...
  TTrackDescription::PitSideMod PitSideMod;      // Data for track description
  PitSideMod.side = PitSide();                   // Get side of pitlane
  PitSideMod.start =                             // Exclude pits
	int(GfParmGetNum(Handle,                     //   while getting extra width
	(char*)SECT_PRIV,(char*)PRV_TRKPIT_START,0,0)); //   starting here
  PitSideMod.end =                               //   and stopping here
	int(GfParmGetNum(Handle,
	(char*)SECT_PRIV,(char*)PRV_TRKPIT_END,0,0));
  //GfOut("SIDE MOD %d %d %d\n",PitSideMod.side,PitSideMod.start,PitSideMod.end);
  //GfOut("\n");
  // ... Find side and sections of pits

  // Create track description
  oTrackDesc.InitTrack(oTrack,Param.oCarParam, oSmoothSide,&PitSideMod);
  // Create pitting strategy
  oStrategy = new TSimpleStrategy();
  oStrategy->oDriver = this;

  // Setup initial fuel for race ...
  float Fuel = GfParmGetNum(Handle               // Estimate fuel consum
    , (char*) SECT_PRIV
    , (char*) PRV_FUELPER100KM                   // based on kg per 100 km
    , (char*) NULL
	, TSimpleStrategy::cMAX_FUEL_PER_METER * 100000);

  float Reserve = GfParmGetNum(Handle            // Reserve in m
    , (char*) SECT_PRIV, (char*) PRV_RESERVE
    , (char*) NULL, 5000);
  oStrategy->oReserve = Reserve;
  oStrategy->oMaxFuel = oMaxFuel;
  oFuelNeeded =
    oStrategy->SetFuelAtRaceStart                // Fueling and pitting
	  (oTrack,CarSettings,oSituation,Fuel);      //   strategy
  GfOut("oFuelNeeded: %.1f\n",oFuelNeeded);
  // ... Setup initial fuel for race
}
//==========================================================================*

//==========================================================================*
// Setup Racinglines
//--------------------------------------------------------------------------*
void TDriver::SetupRacingLines()
{
  double SpeedFFactor;
  double SpeedFFactorLSide;
  double SpeedFFactorRSide;
  double BrakeFFactor;
  double BrakeFFactorLSide;
  double BrakeFFactorRSide;

  const int N = oTrackDesc.Count();

  for (int I = 0; I < N; I++)
  {
    SpeedFFactor = oScaleMuTraffic 
		* oRacingLine[oRL_FREE].oPathPoints[I].SpeedFrictionFactor;
    BrakeFFactor = oScaleBrakeTraffic
		* oRacingLine[oRL_FREE].oPathPoints[I].BrakeFrictionFactor;

    SpeedFFactorLSide = oRacingLine[oRL_LEFT].oPathPoints[I].SpeedFrictionFactor;
    BrakeFFactorLSide = oRacingLine[oRL_LEFT].oPathPoints[I].BrakeFrictionFactor;

    SpeedFFactorRSide = oRacingLine[oRL_RIGHT].oPathPoints[I].SpeedFrictionFactor;
    BrakeFFactorRSide = oRacingLine[oRL_RIGHT].oPathPoints[I].BrakeFrictionFactor;

	SpeedFFactor = 
		MIN(SpeedFFactor,MIN(SpeedFFactorLSide,SpeedFFactorRSide));
	BrakeFFactor = 
		MIN(BrakeFFactor,MIN(BrakeFFactorLSide,BrakeFFactorRSide));
/*
	if (!Qualifying)
    {
        if ((I > 170) && (I < 212))
	    {
		  SpeedFFactor *= 0.90f;
		  BrakeFFactor *= 0.95f;
	    }
        else if ((I > 245) && (I < 275))
	    {
		  SpeedFFactor *= 0.90f;
		  BrakeFFactor *= 0.95f;
	    }
        else if ((I > 500) && (I < 550))
	    {
		  SpeedFFactor *= 0.90f;
		  BrakeFFactor *= 0.95f;
	    }
        //else if ((I > 590) && (I < 625))
        else if ((I > 590) && (I < 680))
	    {
		  SpeedFFactor *= 0.90f;
		  BrakeFFactor *= 0.80f;
	    }
    }
*/

	oRacingLine[oRL_LEFT].oPathPoints[I].SpeedFrictionFactor = SpeedFFactor;
    oRacingLine[oRL_LEFT].oPathPoints[I].BrakeFrictionFactor = BrakeFFactor;

    oRacingLine[oRL_RIGHT].oPathPoints[I].SpeedFrictionFactor = SpeedFFactor;
    oRacingLine[oRL_RIGHT].oPathPoints[I].BrakeFrictionFactor = BrakeFFactor;
  }
}
//==========================================================================*

//==========================================================================*
// Start a new race.
//--------------------------------------------------------------------------*
void TDriver::NewRace(PtCarElt Car, PSituation Situation)
{
  //GfOut(">>> TDriver::NewRace()\n");
  oCar = Car;                                    // Save pointers to TORCS
  oCarHandle = CarCarHandle;                     // data of car, car param
  oSituation = Situation;                        // file and situation
  oLastGear = CarGearNbr - 1;                    // Save index of last gear

  OwnCarOppIndex();                              // Find own opponent index

  InitCarModells();                              // Initilize Car modells
  Param.Fix.oBrakeForce = oBrakeForce;

  oStrategy->Init(this);                         // Init strategy
  FindRacinglines();                             // Find a good racingline
  TeamInfo();                                    // Find team info

  if (oLoadLearned)                              // If wanted
  {
	oLoadedLearned = oRacingLine[oRL_FREE].LoadLearnedFromXMLFile(oCarHandle);
	if (oLoadedLearned)
		SetupRacingLines();
  }
  else
    oLoadedLearned = true;

  oFlying = 0;                                   // Initialize Flags
  oAvoidRange = 1.0;                             // Relative avoiding offset
  oAvoidRangeDelta = 0.0;                        // Avoiding range change
  oAvoidOffsetDelta = 0.0;                       // Avoiding speed
  
  oPIDCLine.oP = 1.0;
  oPIDCLine.oD = 10;

  float MinDist = (float) BasicLengthMargin;
  float MaxDist = (float) (3 * BasicLengthMargin);
  float Range = (float)(MaxDist - MinDist);
  for (int I = NSteps - 1; I >= 0; I--)
	DistLong[I] = MinDist + I * Range / NSteps;

  //GfOut("<<< TDriver::NewRace()\n");
}
//==========================================================================*

//==========================================================================*
// Distance from speed
//--------------------------------------------------------------------------*
bool TDriver::OppInFrontBraking()
{
  bool Result = false;

  if (DistanceRaced < 2080)
	  return Result;

  for (int I = 0; I < oNbrCars; I++)
  {
    if (I == oOwnOppIdx)
	  continue;

    PCarElt OppCar = oOpponents[I].Car();    

    TOpponent::TInfo& OppInfo =          
      oOpponents[I].Info();        

	if (fabs(OppInfo.State.CarDistLat) > OppInfo.State.MinDistLat)
	  continue;

    if ((OppInfo.State.RelPos > 15.0) || (OppInfo.State.RelPos < 0.0))
	  continue;

    Result = true;
  }

  return Result;
}
//==========================================================================*

//==========================================================================*
// Distance from speed
//--------------------------------------------------------------------------*
double TDriver::TargetDist(double Speed)
{
//    if  (DistanceRaced < 500.0)
    if  (DistanceRaced < 2080.0)
		return BasicLengthMargin;

    if (Speed < 0)
		return DistLong[0];
	else if (Speed > NSteps)
		return BasicLengthMargin;
	else if (OppInFrontBraking())
		return DistLong[(int) Speed] + 5;
	else 
		return DistLong[(int) Speed];
//	else 
//		return BasicLengthMargin;
}
//==========================================================================*

//==========================================================================*
// Initialize Drive
//--------------------------------------------------------------------------*
void TDriver::InitDrive()
{
  oAvoidOffset = CalcPathTarget                // Get initial offset
    (oTrackDesc.CalcPos(oCar), -CarToMiddle);  // from start grid
  if (oStartSide > 0)
    oStartSide = SIGN(oAvoidOffset);
  else if (oStartSide < 0)
    oStartSide = -SIGN(oAvoidOffset);
  GfOut("#oIndex: %d oAvoidOffset: %f oStartSide: %f\n",
    oIndex,oAvoidOffset,oStartSide);

  int TeamMateIndex = -1;
  for (int I = 0; I < oNbrCars; I++)
  {
    if (I == oOwnOppIdx)
	  continue;

    PCarElt OppCar = oOpponents[I].Car();    
    if (oCommonData->TeamManager.IsTeamMate(oCar,OppCar))      // If Opp. is teammate
    {
	  TeamMateIndex = OppCar->index;               // Save his index
	  break;
	}
  }

  if (TeamMateIndex > -1)
  {
    if (oOwnOppIdx > TeamMateIndex) 
    {
	  oSecond = true;
      GfOut("#oIndex: %d (%d) (%s) Second=True\n",oIndex,oOwnOppIdx,oBotName);
    }
    else
    {
	  oSecond = false;
      GfOut("#oIndex: %d (%d) (%s) Second=False\n",oIndex,oOwnOppIdx,oBotName);
	}
  }
  else
  {
    oSecond = false;
    GfOut("#oIndex: %d (%d) (%s) Second=False\n",oIndex,oOwnOppIdx,oBotName);
  }

  oFirst = false;
}
//==========================================================================*

//==========================================================================*
// Drive
//--------------------------------------------------------------------------*
void TDriver::Drive()
{
  if (oFirst)
	  InitDrive();

  if (oTestPitStop)                              // If defined, try
    oStrategy->TestPitStop();                    //   to stop in pit

  Propagation();                                 // Propagation

  oAlone = true;                                 // Assume free way to race
  bool Close = false;                            // Assume free way to race
  oLetPass = false;                              // Assume no Lappers

  oAccel = 1.0;                                  // Assume full throttle
  oBrake = 0.0;                                  // Assume no braking

  double Pos = oTrackDesc.CalcPos(oCar);         // Get current pos on track
  oSecIndex = oTrackDesc.IndexFromPos(Pos);
  GetPosInfo(Pos,oLanePoint);                    // Get info about pts on track
  oTargetSpeed = oLanePoint.Speed;               // Target for speed control
/*
  if ((CarLaps < 3) && Qualifying)
  {
	  oTargetSpeed *= 0.98f;
  }

  if ((CarLaps == 1) && (oSecond) && (!Qualifying))
  {
    if ((Pos  > 670) && (Pos  < 827))
	  oTargetSpeed *= 0.8f;
    else if ((Pos  > 2600) && (Pos  < 2900))
      oTargetSpeed *= 0.75f;
    else if ((Pos  > 3500) && (Pos  < 3650))
	  oTargetSpeed *= 0.9f;
  }
  else if ((CarLaps == 1) && (!oSecond) && (!Qualifying))
  {
    if ((Pos  > 690) && (Pos  < 827))
	  oTargetSpeed *= 0.96f;
  }

  if (fabs(oAvoidOffset) > 0.0)
  {
    if ((Pos  > 265) && (Pos  < 675))
	{
      double w = MAX(0.75,1.0 - oAvoidOffset/2);
	  oTargetSpeed *= w;
	}
	else if ((Pos  > 1278) && (Pos  < 1470))
	{
      double w = MAX(0.75,1.0 - oAvoidOffset/2);
	  oTargetSpeed *= w;
	}
	else if ((Pos  > 2950) && (Pos  < 3156))
	{
      double w = MAX(0.80,1.0 - oAvoidOffset/2);
	  oTargetSpeed *= w;
	}
  }
*/

  oTargetDelta = oLanePoint.Delta;               // Delta for speed control

  if (TDriver::LengthMargin < TargetDist(oCurrSpeed))
	TDriver::LengthMargin += 0.05;
  else
	TDriver::LengthMargin -= 0.02;

  if ((oUnstucking) && (oCurrSpeed > 7.5))
	oUnstucking = false;
  else if ((DistanceRaced > 1000.0) && (oCurrSpeed < 2.5) && (!oStrategy->GoToPit()))
	oUnstucking = true;

  if (oUseFilterRelax)
    oTargetSpeed = FilterRelax(oTargetSpeed);

  AvoidOtherCars(oLanePoint.Crv,Close,oLetPass); // General avoiding

/*
  if ((oSecond) && (Pos  > 5230) && (Pos  < 5400) && Close)
    oTargetSpeed = 0.97f;
//  else if ((Pos  > 5270) && (Pos  < 5400) && Close)
  else if ((Pos  > 5250) && (Pos  < 5400) && Close)
    oTargetSpeed = 0.98f;
*/
  
  if (IsStuck())
  {
	  Unstuck();
	  return;
  }

  oSteer = Steering();                           // Steering

  if(Close)                                      // If opponents are close
  {
	BrakingForceRegulatorTraffic();              // Control breaking force
  }
  else if(oStrategy->GoToPit())                  // Going to pitlane
  {
	BrakingForceRegulatorTraffic();              // Control breaking force
  }
  else
  {
	if (oAvoidRange == 0.0)                      // Still avoiding?
      BrakingForceRegulator();                   // Control breaking force
	else
	  BrakingForceRegulatorAvoid();              // Control breaking force
  }

/*
  if ((oExcludeFrom < oSecIndex) && (oSecIndex < oExcludeTill))
  {
    oGear = UsedGear;
  }
  else
*/
  {
    Clutching();                                   // Tread/Release clutch
    GearTronic();                                  // Shift if needed
  }
  Turning();                                     // Check driving direction
  float deltaZ = DetectFlight();
  //GfOut("%f\n",deltaZ);
  FlightControl();                               // Prepare landing while flying

  CheckTolerance();
  SpeedController();							 // Check speed

/*
  if (IsFullThrottle)                            // Learning only if untroubled
	oMaxAccel.Measurement                        // get samples
	  (CarSpeedLong,CarAccelLong);
*/

  // Filters for brake
  if (oBrake > 0.0)
  {
//    GfOut("oBrake: %g",oBrake);
    oBrake = FilterBrake(oBrake);
//    GfOut(" %g",oBrake);
    oBrake = FilterABS(oBrake);
//    GfOut(" %g\n",oBrake);
  }

  if (oBrake == 0.0)
  {
    // Filters for throttle
    oAccel = FilterLetPass(oAccel);
	if (oUseFilterDrifting || oUnstucking)
//	if (oUseFilterDrifting)
      oAccel = FilterDrifting(oAccel);
	if (oUseFilterTrack)
      oAccel = FilterTrack(oAccel);
	if ((oUseFilterTCL || oUnstucking)
	 && (DistanceRaced > 100))
      oAccel = FilterTCL(oAccel);
//	if (oUseFilterDiff || oUnstucking)
	if (oUseFilterDiff)
      oAccel = FilterDiff(oAccel);
	if (DistanceRaced > 100)
      oAccel = FilterCrv(oAccel);

  }
 
  oLastBrake = oBrake;
  oLastAccel = oAccel;

  // Filters for steering
  if (!oUnstucking)
    oSteer = FilterSteerSpeed(oSteer);
  //oSteer *= (1.0 - fabs(deltaZ));
  oLastSteer = (float) oSteer;

  // Tell TORCS what we want do do
  oCar->ctrl.accelCmd = (float) oAccel;
  oCar->ctrl.brakeCmd = (float) oBrake;
  oCar->ctrl.clutchCmd = (float) oClutch;
  oCar->ctrl.gear = oGear;
  oCar->ctrl.steer = (float) oSteer;

  if ((oShowIndex) && (oIndex == 0))
	  GfOut("%d(%g:%d) TS:%6.1f CS:%6.1f A:%g B:%g C:%g G:%d S:%g\n",
      oIndex,Pos,oSecIndex,oTargetSpeed,oCurrSpeed,oAccel,oBrake,oClutch,oGear,oSteer);
//    GfOut("%d(%g): TS: %6.1f S: %6.1f A:%g B:%g C:%g G:%d S:%g(%s)\n",
//      oIndex,Pos,oTargetSpeed,oCurrSpeed,oAccel,oBrake,oClutch,oGear,oSteer,oBotName);

//  if (oIndex == 0)
//	  GfOut("S:%9.6f\n",oSteer);
//	  GfOut("%d(%d:%8.2f)(r=%.2fm): TS:%6.1f S:%6.1f A:%5.3f B:%5.3f C:%5.3f G:%d S:%6.3f\n",
//	oIndex,oLanePoint.Index,Pos,1/oLanePoint.Crv,oTargetSpeed,oCurrSpeed,oAccel,oBrake,oClutch,oGear,oSteer);

  
  oCar->ctrl.lightCmd =                          // All front lights on
    RM_LIGHT_HEAD1 | RM_LIGHT_HEAD2;

  if (!Qualifying)                               // Don't use pit while
    oStrategy->CheckPitState(0.6f);              //  qualification
}
//==========================================================================*

//==========================================================================*
// Pitstop callback
//--------------------------------------------------------------------------*
int TDriver::PitCmd()
{
  oStanding = true;                              // Standing, no unstucking!
  oUnstucking = false;                           // Reset pending flag
  //oUseFilterDiff = false;                        // Reset pending flag

  // Tell TORCS ...
  oCar->pitcmd.fuel = oStrategy->PitRefuel();    // ... how much to refuel
  oCar->pitcmd.repair = oStrategy->PitRepair();  // and to repair
  oCar->pitcmd.stopType = RM_PIT_REPAIR;         // Set repair flag

  if (oCar->pitcmd.repair > 0)                   // If repairing, show
    GfOut("%s repairing: %d damage\n",           // who and how much
	oBotName,oCar->pitcmd.repair);
  if (oCar->pitcmd.fuel > 0.0)                   // If refueling
    GfOut("%s refueling: %.2f\n",                // show who and how much
	oBotName,oCar->pitcmd.fuel);

  oFuelNeeded += oCar->pitcmd.fuel;
  oRepairNeeded += oCar->pitcmd.repair;

  return ROB_PIT_IM;                             // Ready to be serviced
}
//==========================================================================*

//==========================================================================*
// End of the current race.
//--------------------------------------------------------------------------*
void TDriver::EndRace()
{
  //GfOut("TDriver::EndRace() >>>\n");
  // This is never called by TORCS! Don't use it!
  //GfOut("<<< TDriver::EndRace()\n");
}
//==========================================================================*

//==========================================================================*
// Called before the module is unloaded.
//--------------------------------------------------------------------------*
void TDriver::Shutdown()
{
  //if (!oLoadedLearned)
  //  oRacingLine[oRL_FREE].SaveLearnedToFile(oTrackLoadLearned);
  if ((oLoadLearned) && (!oLoadedLearned))
    oRacingLine[oRL_FREE].SaveLearnedToXMLFile(oTrackLoadLearnedXML);
/*
  if (!Qualification)
  {
    GfOut("TrackLoad: %s\n",oTrackLoad);
    oRacingLine[oRL_FREE].SavePointsToFile(oTrackLoad);
    oRacingLine[oRL_LEFT].SavePointsToFile(oTrackLoadLeft);
    oRacingLine[oRL_RIGHT].SavePointsToFile(oTrackLoadRight);
  }
  else
  {
    GfOut("TrackLoadQualify: %s\n",oTrackLoadQualify);
    oRacingLine[oRL_FREE].SavePointsToFile(oTrackLoadQualify);
  }
*/
  if (oSaveStatistics)
	SaveToFile();
}
//==========================================================================*

//==========================================================================*
// Side of pitlane
//--------------------------------------------------------------------------*
int TDriver::PitSide()
{
  return oTrack->pits.side == TR_LFT ? -1 : 1;
}
//==========================================================================*

//==========================================================================*
// Find own car in opponents list
//--------------------------------------------------------------------------*
void TDriver::OwnCarOppIndex()
{
  oOwnOppIdx = -1;                               // Mark as undefined

  if (oNbrCars == 0)
  {
	// First call: Get memory
    oNbrCars = oSituation->_ncars;               // Save Nbr of cars in race
	oOpponents = new TOpponent[oNbrCars];

    for (int I = 0; I < oNbrCars; I++)           // Loop all cars
    {
      oOpponents[I].Initialise                   // Initialize opponents
	    (&oTrackDesc, oSituation, I);            //   situation
	}
  }

  for (int I = 0; I < oNbrCars; I++)             // Loop all cars
  {
    oOpponents[I].Initialise                     // Initialize opponents
	  (&oTrackDesc, oSituation, I);              //   situation
    if (oSituation->cars[I] == oCar)             // Check if is own car
      oOwnOppIdx = I;                            //   save index
  }

}
//==========================================================================*

//==========================================================================*
// Init drive train
//--------------------------------------------------------------------------*
void TDriver::InitDriveTrain()
{
  oDriveTrainType = cDT_RWD;                     // Assume rear wheel drive
  char* TrainType = strdup(GfParmGetStr(oCarHandle,     // but check it
	(char*) SECT_DRIVETRAIN, (char*) PRM_TYPE, (char*) VAL_TRANS_RWD));

  if (strcmp(TrainType, VAL_TRANS_FWD) == 0)     // If front wheel drive
    oDriveTrainType = cDT_FWD;                   //   change mode
  else if (strcmp(TrainType, VAL_TRANS_4WD) == 0)// and if all wheel drive
    oDriveTrainType = cDT_4WD;                   //   too
  free(TrainType);
}
//==========================================================================*

//==========================================================================*
// FindRacinglines
//--------------------------------------------------------------------------*
void TDriver::FindRacinglines()
{
  GfOut("Update car parameters ...\n");
  Param.Update();                                // update car parameters

  GfOut("... set track ...\n");
  if(oCommonData->Track != oTrackDesc.Track())   // New track?
  {
    oCommonData->Track = oTrackDesc.Track();     // Save pointer
    oCommonData->TeamManager.Clear();            // release old informations
  }

  GfOut("... load smooth path ...\n");
  if (oSituation->_raceType == RM_TYPE_PRACTICE)
  {
    GfOut("... make smooth path ...\n");
    oRacingLine[oRL_FREE].MakeSmoothPath         // Calculate a smooth path
	  (&oTrackDesc, Param,                       // as main racingline
	  TClothoidLane::TOptions(oBumpMod));
	if (oSavePlot)
      oRacingLine[oRL_FREE].SaveToFile("RL_FREE.tk3");
  }
  else if (oSituation->_raceType == RM_TYPE_QUALIF)
  {
    if (!oRacingLine[oRL_FREE].LoadSmoothPath    // Load a smooth path
	  (oTrackLoadQualify,
	  &oTrackDesc, Param,                        // as main racingline
	  TClothoidLane::TOptions(oBumpMod)))
	{
      GfOut("... make smooth path ...\n");
      oRacingLine[oRL_FREE].MakeSmoothPath       // Calculate a smooth path
	    (&oTrackDesc, Param,                     // as main racingline
	    TClothoidLane::TOptions(oBumpMod));
  	  if (oSavePlot)
        oRacingLine[oRL_FREE].SaveToFile("RL_FREE.tk3");
//      oRacingLine[oRL_FREE].SavePointsToFile(oTrackLoadQualify);
	}
  }
  else if (!oRacingLine[oRL_FREE].LoadSmoothPath // Load a smooth path
	  (oTrackLoad,
	  &oTrackDesc, Param,                        // as main racingline
	  TClothoidLane::TOptions(oBumpMod)))
  {
    GfOut("... make smooth path ...\n");
    oRacingLine[oRL_FREE].MakeSmoothPath         // Calculate a smooth path
	  (&oTrackDesc, Param,                       // as main racingline
	  TClothoidLane::TOptions(oBumpMod));
 	if (oSavePlot)
      oRacingLine[oRL_FREE].SaveToFile("RL_FREE.tk3");
//    oRacingLine[oRL_FREE].SavePointsToFile(oTrackLoad);
  }
  if (oSaveSections)
    oRacingLine[oRL_FREE].SaveSections("RL_SECTIONS.XML");

  /*
  if (Qualifying)
  {
	oRL_LEFT = oRL_FREE;
	oRL_RIGHT = oRL_FREE;
	Param.oCarParam.oScaleBumpOuter = 0;
    Param.oCarParam2.oScaleBumpRight = 0;
  	Param.oCarParam2.oScaleBumpLeft = 0;  
  }
  else
  */
  {
//    Param.Fix.oBorderInner = 0.25;
//    Param.Fix.oBorderOuter = 0.75;

//    Param.Fix.oBorderInner = 0.5;
//    Param.Fix.oBorderOuter = 1.0;

    Param.oCarParam2.oScaleBumpRight =           // Adjust outer bump scale
	  Param.oCarParam.oScaleBumpOuter;           //   to be able to avoid
  	Param.oCarParam2.oScaleBumpLeft = 0;         // Adjust inner bump scale
//	  Param.oCarParam.oScaleBump;                //   to keep speed
    Param.oCarParam2.oScaleMu =                  // Adjust mu scale
	  oMinScaleMu * Param.oCarParam.oScaleMu;    //   to be able to avoid
	Param.oCarParam2.oScaleBrake =               // Adjust brake scale
	  oMinScaleBrake * Param.oCarParam.oScaleBrake;
    
	if (!oRacingLine[oRL_LEFT].LoadSmoothPath    // Load a smooth path
	  (oTrackLoadLeft,
	  &oTrackDesc, Param,                        // as avoid to left racingline
	    TClothoidLane::TOptions(oBumpMod, FLT_MAX, -oAvoidWidth, true)))
	{
      GfOut("... make avoid path left ...\n");

      oRacingLine[oRL_LEFT].MakeSmoothPath       // Avoid to left racingline
	    (&oTrackDesc, Param,
		TClothoidLane::TOptions(oBumpMod, FLT_MAX, -oAvoidWidth, true));

 	  if (oSavePlot)
        oRacingLine[oRL_LEFT].SaveToFile("RL_LEFT.tk3");
//      oRacingLine[oRL_LEFT].SavePointsToFile(oTrackLoadLeft);
	}

    Param.oCarParam2.oScaleBumpLeft =            // Adjust outer bump scale
	  Param.oCarParam.oScaleBumpOuter;           //   to be able to avoid
    Param.oCarParam2.oScaleBumpRight = 0;        // Reset outer bump scale
//		Param.oCarParam.oScaleBump;       		 //   to keep speed

    if (!oRacingLine[oRL_RIGHT].LoadSmoothPath   // Load a smooth path
	  (oTrackLoadRight,
	  &oTrackDesc, Param,                        // as avoid to left racingline
  	    TClothoidLane::TOptions(oBumpMod, -oAvoidWidth, FLT_MAX, true)))
	{
      GfOut("... make avoid path right ...\n");

	  oRacingLine[oRL_RIGHT].MakeSmoothPath      // Avoid to right racingline
	    (&oTrackDesc, Param,
  	    TClothoidLane::TOptions(oBumpMod, -oAvoidWidth, FLT_MAX, true));
 	  if (oSavePlot)
        oRacingLine[oRL_RIGHT].SaveToFile("RL_RIGHT.tk3");
//      oRacingLine[oRL_RIGHT].SavePointsToFile(oTrackLoadRight);
	}

    double MaxPitDist = 0.0;
	if (oStrategy->oPit->HasPits())
	{
      for (int I = 0; I < NBRRL; I++)            // Adjust racinglines
      {                                          // using car parameters
	    GfOut("... adjust pit path %d ...\n",I);
        oStrategy->oPit->oPitLane[I].MakePath
	      (oPitLoad[I],&oRacingLine[I], Param, I);

	    if (MaxPitDist < oStrategy->oPit->oPitLane[I].PitDist())
          MaxPitDist = oStrategy->oPit->oPitLane[I].PitDist();
	  }

   	  if (oSavePlot)
	  {
        oStrategy->oPit->oPitLane[0].SaveToFile("RL_PIT_FREE.tk3");
        oStrategy->oPit->oPitLane[1].SaveToFile("RL_PIT_LEFT.tk3");
        oStrategy->oPit->oPitLane[2].SaveToFile("RL_PIT_RIGHT.tk3");
	  }
	}

    oStrategy->oDistToSwitch = MaxPitDist + 100; // Abstand zur Boxeneinfahrt
  }
  GfOut("... Done\n");
}
//==========================================================================*

//==========================================================================*
// Get Team info
//--------------------------------------------------------------------------*
void TDriver::TeamInfo()
{
  oTeam = oCommonData->TeamManager.Add(oCar);
  //GfOut("\n\nTeam: %s\n\n\n",oTeam->TeamName);
}
//==========================================================================*

//==========================================================================*
// Initialize Car modells
//--------------------------------------------------------------------------*
void TDriver::InitCarModells()
{
  oCarParams[0] = &Param.oCarParam;              // Get pointers as array
  oCarParams[1] = &Param.oCarParam2;
  oCarParams[2] = &Param.oCarParam2;

  Param.Initialize(oCar,this);                   // Initialize parameters

  Param.SetEmptyMass(                            // Set car mass
	GfParmGetNum(oCarHandle,
	  (char*) SECT_CAR, (char*) PRM_MASS, (char*) NULL, 1000.0));

  InitBrake();                                   // Brake
  InitCa();                                      // Ca
  InitCw();                                      // Cw
  InitDriveTrain();                              // Drive train
  InitTireMu();                                  // Tyre friction
  InitWheelRadius();                             // Wheel radius
  InitAdaptiveShiftLevels();                     // Gear shifting

  Param.Tmp.oFuel = 0;                           // Still unfueld
  Param.Fix.oWidth = CarWidth;                   // width of car
  Param.oCarParam2 = Param.oCarParam;            // Copy to avoid set
  Param.oCarParam2.oScaleMu =                    // Adjust mu scale to
	MIN(0.95f, 0.9f * Param.oCarParam.oScaleMu); //   be able to avoid
  Param.oCarParam3 = Param.oCarParam;            // Copy to pit set
}
//==========================================================================*

//==========================================================================*
// Update pointers and multiple used values
//--------------------------------------------------------------------------*
void TDriver::Update(tCarElt* Car, tSituation* S)
{
  oCar = Car;                                    // Update pointers
  oSituation = S;

  // Shortcuts
  oCurrSpeed = myhypot(CarSpeedLong, CarSpeedLat); // Save currend speed
  if (fabs(oCurrSpeed) < 1)                      // At slow speeds use
    oAngleSpeed = CarYaw;                        // direction of cars x-axis
  else                                           // else use
    oAngleSpeed = atan2(CarSpeedY, CarSpeedX);   // direction of movement

  oTrackAngle =                                  // Direction of track at the
	 RtTrackSideTgAngleL(&CarTrackPos);          // position of the car
  oDistFromStart = oTrackDesc.CalcPos(oCar, 0.0);// Cars distance from Start
  TVec2d Target =                                // Target to steer to
	CalcPathTarget2(oDistFromStart + 5.0, 0.0);  // while unstucking
  oSteerAngle = (float) atan2                    // Direction to steer
	(Target.y - CarPosY, Target.x - CarPosX);    //   from here to target
  oSteerAngle -= (float) CarYaw;                 // Relative to cars x-axis
//  oSteerAngle -= (float) oAngleSpeed;            // Relative to cars movement
  FLOAT_NORM_PI_PI(oSteerAngle);                 // Normalize to -PI,+PI

  oDriftAngle =                                  // Actual drift angle
	atan2(CarSpeedY, CarSpeedX) - CarYaw;
  DOUBLE_NORM_PI_PI(oDriftAngle);                // normalized to +Pi .. -Pi

  // Get direction of motion
  double MySpd = MAX(0.01,myhypot(CarSpeedX, CarSpeedY));
  double MyDomX = CarSpeedX / MySpd;
  double MyDomY = CarSpeedY / MySpd;

  // Update all opponents data
  for (int I = 0; I < oNbrCars; I++)
  {
	oOpponents[I].Update(oCar,
	  &oCommonData->TeamManager, MyDomX, MyDomY);
  }

  if (CarLaps > oLastLap)
  {
	UpdateTargetSpeed();
  }

  oStrategy->Update(oCar);                       // Update strategic params

  oSideReduction = 1.0;
  if (WheelSeg(REAR_RGT) != WheelSeg(REAR_LFT))
  {
    float MinFriction = MIN(WheelSegFriction(REAR_RGT),WheelSegFriction(REAR_LFT));
	oSideReduction = (float) MIN(1.0,MinFriction / CarFriction);
	if (oSideReduction > 0.9f) 
  	  oSideReduction = 1.0;
	//GfOut("SideReduction: %g\n",oSideReduction);
  }
  oOppsNearBehind = 0;
  oOppsNearInFront = 0;

  //float Angle = oTrackAngle - CarYaw;
  //FLOAT_NORM_PI_PI(Angle);
  //GfOut("%f %f %f %f\n",
  //  oSteerAngle,Angle,oTrackAngle,CarYaw);

}
//==========================================================================*

//==========================================================================*
// Detect flight
//--------------------------------------------------------------------------*
float TDriver::DetectFlight()
{
  double H[4];
  double HMax = 0.0;

  double RideHeight = 0.0;
  for (int I = 0; I < 4; I++)
  {
    tTrkLocPos Wp;
    float Wx = oCar->pub.DynGCg.pos.x;
    float Wy = oCar->pub.DynGCg.pos.y;
    RtTrackGlobal2Local(CarSeg, Wx, Wy, &Wp, TR_LPOS_SEGMENT);
    H[I] = CarPosZ - RtTrackHeightL(&Wp) - WheelRad(I);
    RideHeight += H[I];
	if (HMax < H[I])
      HMax = H[I];
  }

  if (HMax > oFlyHeight)
  {
    oFlying = MIN(FLY_COUNT, oFlying + (FLY_COUNT / 2));
  }
  else if (oFlying > 0)
  {
    oFlying--;
  }
  float Delta = (float)((oLastRideHeight - RideHeight)/4.0);
  oLastRideHeight = (float) RideHeight;
  return Delta;
}
//==========================================================================*

//==========================================================================*
// Prepare landing
//--------------------------------------------------------------------------*
void TDriver::FlightControl()
{
  if (oFlying)
  {
    // Steer in direction of car movement
    double Angle = oAngleSpeed - CarYaw;
    DOUBLE_NORM_PI_PI(Angle);
    int F = FLY_COUNT - oFlying;
    double T = MAX(0, MIN(1.0 * F / FLY_COUNT, 1));
    oSteer = oSteer * T + (1 - T) * Angle / SteerLock;
  }
}
//==========================================================================*

//==========================================================================*
// Propagation
//--------------------------------------------------------------------------*
void TDriver::Propagation()
{
  if ((oLastCarLaps < CarLaps) || (Param.Tmp.Needed()))
  {
	oLastCarLaps = CarLaps;
	Param.Update();
/**/
	for (int I = 0; I < NBRRL; I++)
	{
      oRacingLine[I].CalcMaxSpeeds(1);
	  oRacingLine[I].PropagateBreaking(1);
      oRacingLine[I].PropagateAcceleration(1);
	}
/**/
  }
}
//==========================================================================*

//==========================================================================*
// Steering
//--------------------------------------------------------------------------*
double TDriver::Steering()
{
  if (oUnstucking)
  {
    oAngle = UnstuckSteerAngle();
  }
  else
  {
    TLanePoint AheadPointInfo;
    oAngle = SteerAngle(AheadPointInfo);
  }
  oDeltaOffset = oLanePoint.Offset + CarToMiddle; // Delta to planned offset
  return oAngle / SteerLock;
}
//==========================================================================*

//==========================================================================*
// Calculate mean wheel radius
//--------------------------------------------------------------------------*
void TDriver::InitWheelRadius()
{
  int Count = 0;
  oWheelRadius = 0.0;

  if(HasDriveTrainFront)
  {
    oWheelRadius += WheelRad(FRNT_LFT) + WheelRad(FRNT_RGT);
    Count += 2;
  }

  if(HasDriveTrainRear)
  {
    oWheelRadius += WheelRad(REAR_LFT) + WheelRad(REAR_RGT);
    Count += 2;
  }
  oWheelRadius /= Count;
}
//==========================================================================*

//==========================================================================*
// Initialize tire mu
//--------------------------------------------------------------------------*
void TDriver::InitTireMu()
{
  int I;

  Param.Fix.oTyreMuFront = FLT_MAX;
  for (I = 0; I < 2; I++)
	Param.Fix.oTyreMuFront = MIN(Param.Fix.oTyreMuFront,
	  GfParmGetNum(oCarHandle, WheelSect[I],
	  (char*) PRM_MU, (char*) NULL, 1.0f));

  Param.Fix.oTyreMuRear = FLT_MAX;
  for (I = 2; I < 4; I++)
	Param.Fix.oTyreMuRear = MIN(Param.Fix.oTyreMuRear,
	  GfParmGetNum(oCarHandle, WheelSect[I],
	  (char*) PRM_MU, (char*) NULL, 1.0f));

  Param.Fix.oTyreMu =
	MIN(Param.Fix.oTyreMuFront,Param.Fix.oTyreMuRear);
}
//==========================================================================*

//==========================================================================*
// Initialize brake
//--------------------------------------------------------------------------*
void TDriver::InitBrake()
{
	GfOut("\n\n\n");
    float DiameterFront = 
		GfParmGetNum(oCarHandle, (char*) SECT_FRNTRGTBRAKE, 
			PRM_BRKDIAM, (char*)NULL, 0.2f);
    float DiameterRear = 
		GfParmGetNum(oCarHandle, (char*) SECT_REARRGTBRAKE, 
			PRM_BRKDIAM, (char*)NULL, 0.2f);
	GfOut("# Brake diameter    : %0.3f m / %0.3f m\n",DiameterFront,DiameterRear);

	float AreaFront = 
		GfParmGetNum(oCarHandle, (char*) SECT_FRNTRGTBRAKE, 
			PRM_BRKAREA, (char*)NULL, 0.002f);
	float AreaRear = 
		GfParmGetNum(oCarHandle, (char*) SECT_REARRGTBRAKE, 
			PRM_BRKAREA, (char*)NULL, 0.002f);
	GfOut("# Brake area        : %0.5f m2 / %0.5f m2\n",AreaFront,AreaRear);

	float MuFront
		= GfParmGetNum(oCarHandle, (char*) SECT_FRNTRGTBRAKE, 
			PRM_MU, (char*)NULL, 0.30f);
	float MuRear
		= GfParmGetNum(oCarHandle, (char*) SECT_REARRGTBRAKE, 
			PRM_MU, (char*)NULL, 0.30f);
	GfOut("# Brake mu          : %0.5f / %0.5f\n",MuFront, MuRear);

	float Rep =
		GfParmGetNum(oCarHandle, (char*) SECT_BRKSYST, 
			PRM_BRKREP, (char*)NULL, 0.5);
	GfOut("# Brake repartition : %0.2f\n",Rep);

	float Press =
		GfParmGetNum(oCarHandle, (char*) SECT_BRKSYST, 
			PRM_BRKPRESS, (char*)NULL, 1000000);
	GfOut("# Brake pressure    : %0.0f\n",Press);

    oBrakeForceMax =
		GfParmGetNum(oCarHandle, TDriver::SECT_PRIV, 
			PRV_MAX_BRAKING, (char*)NULL, oBrakeForceMax);
	GfOut("# Brake force max   : %0.7f\n",oBrakeForceMax);
    
    float BrakeCoeffFront = (float) (DiameterFront * 0.5 * AreaFront * MuFront);
    float BrakeCoeffRear = (float) (DiameterRear * 0.5 * AreaRear * MuRear);
	GfOut("# Brake coefficient : %0.7f / %0.7f\n",BrakeCoeffFront,BrakeCoeffRear);

	oBrakeMaxTqFront = /* oBrakeForceMax * */ BrakeCoeffFront * Press * Rep;
	GfOut("# Brake torque front: %0.2f\n",oBrakeMaxTqFront);

	oBrakeMaxTqRear = /* oBrakeForceMax * */ BrakeCoeffRear * Press * (1 - Rep);
	GfOut("# Brake torque rear : %0.2f\n",oBrakeMaxTqRear);

	oBrakeForce = oBrakeMaxTqFront * (WheelRad(FRNT_LFT) + WheelRad(FRNT_RGT)) 
		+ oBrakeMaxTqRear * (WheelRad(REAR_LFT) + WheelRad(REAR_RGT)); 
	GfOut("# Brake force       : %0.2f\n",oBrakeForce);

	GfOut("\n\n\n");
}
//==========================================================================*

//==========================================================================*
// Initialize Ca
//--------------------------------------------------------------------------*
void TDriver::InitCa()
{
  float FrontWingArea =
	GfParmGetNum(oCarHandle, (char*) SECT_FRNTWING,
	  (char*) PRM_WINGAREA, (char*) NULL, 0.0);
  float FrontWingAngle =
	GfParmGetNum(oCarHandle, (char*) SECT_FRNTWING,
	  (char*) PRM_WINGANGLE, (char*) NULL, 0.0);
  float RearWingArea =
	GfParmGetNum(oCarHandle, (char*) SECT_REARWING,
	  (char*) PRM_WINGAREA, (char*) NULL, 0.0f);
  float RearWingAngle =
	GfParmGetNum(oCarHandle, (char*) SECT_REARWING,
	  (char*) PRM_WINGANGLE, (char*) NULL, 0.0f);

  FrontWingArea = FrontWingArea * sin(FrontWingAngle);
  RearWingArea = RearWingArea * sin(RearWingAngle);
  float WingCd = (float) (1.23 * (FrontWingArea + RearWingArea));
  Param.Fix.oCdWing = WingCd;

  float CLFront =
	GfParmGetNum(oCarHandle, (char*) SECT_AERODYNAMICS,
	  (char*) PRM_FCL, (char*) NULL, 0.0f);
  float CLRear =
	+ GfParmGetNum(oCarHandle, (char*) SECT_AERODYNAMICS,
	  (char*) PRM_RCL, (char*) NULL, 0.0f);
  float CL = 1.8f * MIN(CLFront,CLRear);
 
  float H = 0.0; 
  int I;
  for (I = 0; I < 4; I++)
	H += GfParmGetNum(oCarHandle, WheelSect[I],
	  (char*) PRM_RIDEHEIGHT, (char*) NULL, 0.20f);

  H *= 1.5;
  H = H*H;
  H = H*H;
  H = (float) (2.0 * exp(-3.0 * H));
  Param.Fix.oCa = H * CL + 4.0 * WingCd;
  Param.Fix.oCaFrontWing = 4 * 1.23 * FrontWingArea;
  Param.Fix.oCaRearWing = 4 * 1.23 * RearWingArea;
  Param.Fix.oCaGroundEffect = H * CL;

}
//==========================================================================*

//==========================================================================*
// Initialize Cw
//--------------------------------------------------------------------------*
void TDriver::InitCw()
{
  float Cx =
	GfParmGetNum(oCarHandle,
	(char*) SECT_AERODYNAMICS, (char*) PRM_CX, (char*) NULL, 0.0f);
  float FrontArea =
	GfParmGetNum(oCarHandle,
	(char*) SECT_AERODYNAMICS, (char*) PRM_FRNTAREA, (char*) NULL, 0.0f);

  Param.Fix.oCdBody = 0.645 * Cx * FrontArea;
}
//==========================================================================*

//==========================================================================*
// Get gear ratio
//--------------------------------------------------------------------------*
double TDriver::GearRatio()
{
  return CarGearRatio[UsedGear + CarGearOffset];
}
//==========================================================================*

//==========================================================================*
// Get gear ratio of previous gear
//--------------------------------------------------------------------------*
double TDriver::PrevGearRatio()
{
  return CarGearRatio[UsedGear + CarGearOffset-1];
}
//==========================================================================*

//==========================================================================*
// Get gear ratio of next gear
//--------------------------------------------------------------------------*
double TDriver::NextGearRatio()
{
  return CarGearRatio[UsedGear + CarGearOffset+1];
}
//==========================================================================*
/* */
//==========================================================================*
// Start automatic
//--------------------------------------------------------------------------*
double TDriver::StartAutomatic()
{
  if (CarGearCmd == 1)
  {
	if (CarRpm < oStartRPM)
	  oClutch += 0.9 * oStartRPM/CarRpm * oClutchDelta;
//	else if (CarRpm > 1.3 * oStartRPM)
//	  oClutch -= oClutchDelta * 30 * oClutchRelease;
//	else if (CarRpm > 1.2 * oStartRPM)
//	  oClutch -= oClutchDelta * 10 * oClutchRelease;
	else if (CarRpm > 1.1 * oStartRPM)
	  oClutch -= oClutchDelta * oClutchRelease;
  }
  return MAX(oClutch,oClutchMax);
}
//==========================================================================*
/* */
/*
//==========================================================================*
// Start automatic
//--------------------------------------------------------------------------*
double TDriver::StartAutomatic()
{
  if (oSituation->_raceState & RM_RACE_PRESTART) 
  {
	oClutch = oClutchMax;
	return oClutch;
  }

  if ((CarGearCmd == 1) && (TDriver::CurrSimTime < 20))
  {
	if (CarRpm < oStartRPM) 
	  oClutch += oClutchDelta;
	else if (CarRpm > 1.1 * oStartRPM) 
	  oClutch -= oClutchDelta * oClutchRelease;
  }
  return oClutch;
}
//==========================================================================*
*/
/* */
//==========================================================================*
// Simplified clutch controller
//--------------------------------------------------------------------------*
void TDriver::Clutching()
{
  double ClutchMax = oClutchMax;
  if (oSituation->_raceState & RM_RACE_PRESTART)
  {
	oClutch = oClutchMax * 1.39;
	return;
  }
  //if (DistanceRaced < 100.0)
  //  GfOut("C:%g rpm:%g T:%g s S:%g m\n",oClutch,RADS2RPM(CarRpm),CurrSimTime,DistanceRaced);

  if(oClutch > 0)
  {
    if ((oGear < 2) && (DistanceRaced < 1000))
	{
	  if (oUnstucking)
        ClutchMax = MIN(0.8,StartAutomatic());
	  else
        ClutchMax = StartAutomatic();
//        ClutchMax = MIN(0.7,StartAutomatic());
	}
	oClutch = MIN(ClutchMax,oClutch);
	if(oClutch == ClutchMax)
	{
	  if(GearRatio() * CarSpeedLong
		  / (oWheelRadius * CarRpm) > oClutchRange)
	  {
        oClutch = ClutchMax - 0.01;
	  }
	}
	else
	{
	  oClutch -= oClutchDelta;
	  oClutch = MAX(0.0,oClutch);
	}
  }
}
//==========================================================================*
/* */
/*
//==========================================================================*
// Simplified clutch controller
//--------------------------------------------------------------------------*
void TDriver::Clutching()
{
  if (TDriver::CurrSimTime < oSituation->deltaTime)
  {
    oClutch = oClutchMax;
	return;
  }

  if(oClutch > 0)
  {
    if (oGear < 2)
      StartAutomatic();

	oClutch = MIN(oClutchMax,oClutch);
	if(oClutch == oClutchMax)
	{
	  if(GearRatio() * CarSpeedLong
		  / (oWheelRadius * CarRpm) > oClutchRange)
	  {
        oClutch = oClutchMax - 0.01;
	  }
	}
	else
	{
	  oClutch -= oClutchDelta;
	  oClutch = MAX(0.0,oClutch);
	}
  }
}
//==========================================================================*
*/
//==========================================================================*
// Turn if driving backwards unexpectedly
//--------------------------------------------------------------------------*
void TDriver::Turning()
{
  if (!oUnstucking && (DistanceRaced > 25))
  {
    double Angle = oLanePoint.Angle - CarYaw;    // Direction moving to
    DOUBLE_NORM_PI_PI(Angle);                    // normalize it

    if((oGear > 0)                               // If a gear is selected
//      && (fabs(Angle) > 75 * PI / 180))          // but irregular direction
      && (fabs(Angle) > 45 * PI / 180))          // but irregular direction
    {
	  if (oTurnCounter == 0)
	  {
        if(Angle * CarToMiddle < 0)                // Lets turn
  	    {
          oTurnCounter = 100;
          oGear = -1;
          oAccel = 0.5;
          oBrake = 0;
//		  oLastSteer = (float) (-SGN(Angle) * 1.0);
          oSteer = (float) (-SGN(Angle) * 1.0);
		  //GfOut("*Angle: %gCarToMiddle: %g (%g): %g\n",Angle,CarToMiddle,Angle * CarToMiddle,oSteer);
		}
	  }
	  else
	  {
        oSteer = oLastSteer;
        oTurnCounter -= 1;
		oTurnCounter = MAX(oTurnCounter,0);
	    //GfOut(" Angle: %gCarToMiddle: %g (%g): %g\n",Angle,CarToMiddle,Angle * CarToMiddle,oSteer);
	  }
    }
	else
      oTurnCounter = 0;

    if((oGear > 0) && (CarSpeedLong < -0.01))    // Rolling back?
    {                                            // Shift down and start
      oGear = 1;
      oBrake = CarSpeedLong < -0.5 ? 0.25 : 0;
      oAccel = 0.25;
    }

    if ((oGear == 1)                             // If starting
	  && (CarSpeedLong < 10)                     //   and slow
	  && (fabs(CarSpeedLong) >= 0.01)            //   but moving
	  && (oAccel == 1.0 && oBrake == 0))         //   and acc. (not braking)
    {                                            // use clutch
      double rpm = CarRpm;
      oClutch = (850 - rpm) / 400;
      if(CarSpeedLong < 0.05)
        oClutch = 0.5;

      oClutch = MAX(0, MIN(oClutch, 0.9));       // Normalize
      //if (DistanceRaced < 100.0)
      //  GfOut("c:%g rpm:%g T:%g s S:%g m\n",oClutch,RADS2RPM(CarRpm),CurrSimTime,DistanceRaced);
    }
  }
}
//==========================================================================*

//==========================================================================*
// Calculate shift levels
//--------------------------------------------------------------------------*
void TDriver::InitAdaptiveShiftLevels()
{
  //GfOut("TDriver::InitShiftLevels() >>>\n");

  struct tEdesc
  {
    tdble rpm;
    tdble tq;
  } *Edesc;

  struct TDataPoints
  {
    tdble rads;
    tdble a;
    tdble b;
  } *DataPoints;

  float RevsMax;
  float Tickover;
  float RevsLimiter;
  double RpmFactor = 30 / PI;                    // Unit conversion

  char	idx[64];
  sprintf(idx, "%s/%s", SECT_ENGINE, ARR_DATAPTS);
  int IMax = GfParmGetEltNb(oCarHandle, idx);
  //GfOut("\nIMax: %d\n",IMax);

  RevsMax = GfParmGetNum(oCarHandle, (char*) SECT_ENGINE,
	(char*) PRM_REVSMAX, (char*)NULL, 1000);
  //GfOut("RevsMax: %f = %0.0f [rpm]\n\n",RevsMax,RevsMax*RpmFactor);

  Tickover = GfParmGetNum(oCarHandle, (char*) SECT_ENGINE,
	  (char*) PRM_TICKOVER, (char*)NULL, 150);
  //GfOut("Tickover: %f = %0.0f [rpm]\n\n",Tickover,Tickover*RpmFactor);

  RevsLimiter = GfParmGetNum(oCarHandle, (char*) SECT_ENGINE,
	  (char*) PRM_REVSLIM, (char*) NULL, 800);
  //GfOut("RevsLimiter: %f = %0.0f [rpm]\n",RevsLimiter,RevsLimiter*RpmFactor);

  Edesc = (struct tEdesc*) malloc((IMax + 1) * sizeof(struct tEdesc));

  int I;

  oShiftMargin = 0.9;                            //
  for (I = 0; I < MAX_GEARS; I++)
  {
    oShift[I] = 2000.0;
    oGearEff[I] = 0.95;
  }

  for (I = 0; I < IMax; I++)
  {
	sprintf(idx, "%s/%s/%d", SECT_ENGINE, ARR_DATAPTS, I+1);
    Edesc[I].rpm = GfParmGetNum(oCarHandle, idx,
	  (char*) PRM_RPM, (char*) NULL, RevsMax);
    Edesc[I].tq = GfParmGetNum(oCarHandle, idx,
	  (char*) PRM_TQ, (char*) NULL, 0.0f);

	//GfOut("rpm: %f = %0.0f [rpm] tq: %0.1f\n",Edesc[I].rpm,Edesc[I].rpm*RpmFactor,Edesc[I].tq);
  }

  Edesc[IMax].rpm = Edesc[IMax - 1].rpm;
  Edesc[IMax].tq  = Edesc[IMax - 1].tq;
  //GfOut("rpm: %f = %0.0f [rpm] tq: %0.1f\n",Edesc[IMax].rpm,Edesc[IMax].rpm*RpmFactor,Edesc[IMax].tq);
  //GfOut("\n");

  double maxTq = 0;
  double rpmMaxTq = 0;
  double maxPw = 0;
  double rpmMaxPw = 0;
  double TqAtMaxPw = 0;
  DataPoints = (TDataPoints *) malloc(IMax * sizeof(TDataPoints));
  TDataPoints *Data;
  for (I = 0; I < IMax; I++)
  {
	Data = &DataPoints[I];

	Data->rads = Edesc[I+1].rpm;
	if ((Data->rads >= Tickover)
			&& (Edesc[I+1].tq > maxTq)
			&& (Data->rads < RevsLimiter))
	{
	  maxTq = Edesc[I+1].tq;
	  rpmMaxTq = Data->rads;
	}
	if ((Data->rads >= Tickover)
			&& (Data->rads * Edesc[I+1].tq > maxPw)
			&& (Data->rads < RevsLimiter))
	{
	  TqAtMaxPw = Edesc[I+1].tq;
	  maxPw = Data->rads * Edesc[I+1].tq;
	  rpmMaxPw = Data->rads;
	}

	Data->a = (Edesc[I+1].tq - Edesc[I].tq)
	  / (Edesc[I+1].rpm - Edesc[I].rpm);
	Data->b = Edesc[I].tq - Data->a * Edesc[I].rpm;
  }
  oStartRPM = rpmMaxTq;
  //GfOut("maxTq: %f rpmMaxTq: %f maxPw: %f rpmMaxPw: %f\n",
  //  maxTq,rpmMaxTq*RpmFactor,maxPw,rpmMaxPw*RpmFactor);
  //GfOut("TqAtMaxPw: %f\n", TqAtMaxPw);

/*
  for (I = 0; I < IMax; I++)
  {
	Data = &DataPoints[I];
	GfOut("rads: %f a: %f b: %f\n",Data->rads,Data->a,Data->b);
  }
  GfOut("\n");
*/

  //GfOut("Nbr of Gears: %d\n",CarGearNbr - 1);
  for (I = 0; I < CarGearNbr - 1; I++)
  {
	sprintf(idx, "%s/%s/%d", SECT_GEARBOX, ARR_GEARS, I+1);
    oGearEff[I] = GfParmGetNum(oCarHandle, idx,
	  (char*) PRM_EFFICIENCY, (char*) NULL, 0.94f);

	//GfOut("Gear: %d Efficency: %f\n",I+1,oGearEff[I]);
  }

  int J;
  for (J = 0; J < CarGearNbr; J++)
    oShift[J] = RevsLimiter * 0.974;
//    oShift[J] = RevsLimiter * 0.99;

  //GfOut("\n\n");
  for (J = 1; J < oLastGear; J++)
  {
      double Rpm = Tickover;
      double RpmNext = Tickover;
      //double BrakeCoeff = 0.33f;
      double Tq = 0.0;
      double TqNext = 0.0;
      double GearRatioAct;
      double GearRatioNext;

      while (Rpm <= RevsLimiter)
	  {
		for (I = 0; I < IMax; I++)
		{
			if (Rpm < DataPoints[I].rads)
			{
				Tq = (Rpm * DataPoints[I].a
				  + DataPoints[I].b) * oGearEff[J-1];
				break;
			}
		}

        GearRatioAct = CarGearRatio[J + CarGearOffset];
        GearRatioNext = CarGearRatio[J + 1 + CarGearOffset];
        RpmNext = Rpm * GearRatioNext / GearRatioAct;
		for (I = 0; I < IMax; I++)
		{
			if (RpmNext < DataPoints[I].rads)
			{
				TqNext = (RpmNext * DataPoints[I].a
				  + DataPoints[I].b) * GearRatioNext
				  / GearRatioAct * oGearEff[J];
		        //GfOut("J: %d UD: %0.3f Rpm: %0.0f Rpm: %0.0f Tq; %0.1f RpmNext: %0.0f TqNext; %0.1f\n",
		        //  J, Rpm / RevsLimiter, oShift[J], Rpm*RpmFactor, Tq, RpmNext*RpmFactor, TqNext);
				break;
			}
		}

        if ((TqNext > Tq ) && (Rpm*RpmFactor > 2000))
		{
		  oShift[J] = Rpm * 0.98;
		  GfOut("J: %d UD: %0.3f Shift: %0.0f Rpm: %0.0f Tq; %0.1f RpmNext: %0.0f TqNext; %0.1f\n",
		    J, Rpm / RevsLimiter, oShift[J]*RpmFactor, Rpm*RpmFactor, Tq, RpmNext*RpmFactor, TqNext);
		  break;
		}
 	    Rpm += 1;
	  }
	  GfOut("J: %d UD: %0.3f Rpm: %0.0f [rpm] Shift: %0.5f\n",
	    J, oShift[J] / RevsLimiter, oShift[J]*RpmFactor, oShift[J]);
  }
  GfOut("\n\n");

  free(DataPoints);
  free(Edesc);

  GfOut("<<< TDriver::InitShiftLevels()\n");
}
//==========================================================================*

//==========================================================================*
// S²GCuASL ;D
// = Simplified sequential gear controller using adaptive shift levels
//--------------------------------------------------------------------------*
void TDriver::GearTronic()
{
  if (IsTickover)
  {
    oGear = 1;
  }
  else
  {
    if((UsedGear < oLastGear)
	  && (GearRatio() * CarSpeedLong / oWheelRadius > NextRpm))
	{
      oUnstucking = false;
//      if(UsedGear > 3)
//        oUseFilterDiff = false;
      TreadClutch;
	  oGear = NextGear;
	}
    else if(UsedGear > 1)
	{
      double PrevRpm =
  	    oShift[UsedGear-1] * oShiftMargin
	    * GearRatio() / PrevGearRatio();

      if(GearRatio() * CarSpeedLong / oWheelRadius < PrevRpm)
	  {
	    TreadClutch;
	    oGear = PrevGear;
	  }
	}
  }
}
//==========================================================================*

//==========================================================================*
// Get info to point
//--------------------------------------------------------------------------*
void TDriver::GetLanePoint(int Path, double Pos, TLanePoint& LanePoint)
{
  if (oStrategy->oPit != NULL
	&& oStrategy->oPit->HasPits()
	&& !oStrategy->oWasInPit
	&& oStrategy->GoToPit() && oStrategy->oPit->oPitLane[Path].ContainsPos(Pos))
  {
    //GfOut("+");
    oStrategy->oPit->oPitLane[Path].GetLanePoint(Pos, LanePoint);
	oLookScale = 0.05;
	oOmegaScale = 0.2;
	oLookBase = Param.Fix.oLength / 4;
	oOmegaBase = Param.Fix.oLength / 2;
	oGoToPit = true;
  }
  else if (oStrategy->oPit != NULL
	&& oStrategy->oPit->HasPits()
	&& oStrategy->oWasInPit
	&& oStrategy->oPit->oPitLane[Path].ContainsPos(Pos))
  {
    //GfOut("-");
    oStrategy->oPit->oPitLane[Path].GetLanePoint(Pos, LanePoint);
	oLookScale = 0.02;
	oOmegaScale = 0.2;
	oLookBase = Param.Fix.oLength / 10;
	oOmegaBase = Param.Fix.oLength / 2;
	oGoToPit = true;
  }
  else
  {
    //GfOut("*");
    oRacingLine[Path].GetLanePoint(Pos, LanePoint);
	oLookScale = oLookAheadFactor;
	oOmegaScale = oOmegaAheadFactor;
	oLookBase = oLookAhead;
	oOmegaBase = oOmegaAhead;
	oGoToPit = false;
  }
}
//==========================================================================*

//==========================================================================*
// Get info to position
//--------------------------------------------------------------------------*
void TDriver::GetPosInfo
  (double Pos, TLanePoint& PointInfo, double U, double V )
{
  GetLanePoint(oRL_FREE, Pos, PointInfo);

  if(U != 0.0)
  {
    TLanePoint PointInfoL, PointInfoR;
    GetLanePoint(oRL_LEFT,  Pos, PointInfoL );
    GetLanePoint(oRL_RIGHT, Pos, PointInfoR );

    double T = (1.0 - V) * 0.5;

	InterpolatePointInfo(PointInfoL, PointInfo, U);
    InterpolatePointInfo(PointInfoR, PointInfo, U);

    PointInfo = PointInfoL;

    InterpolatePointInfo(PointInfo, PointInfoR, T);
  }
}
//==========================================================================*

//==========================================================================*
// Get info to position
//--------------------------------------------------------------------------*
void TDriver::GetPosInfo(double Pos, TLanePoint& PointInfo)
{
  GetPosInfo(Pos, PointInfo, oAvoidRange, oAvoidOffset);
}
//==========================================================================*

//==========================================================================*
// Calculate path target
//--------------------------------------------------------------------------*
double TDriver::CalcPathTarget(double Pos, double Offset)
{
  TLanePoint PointInfo, PointInfoL, PointInfoR;

  GetLanePoint(oRL_FREE,Pos,PointInfo);
  GetLanePoint(oRL_LEFT,Pos,PointInfoL);
  GetLanePoint(oRL_RIGHT,Pos,PointInfoR);

  InterpolatePointInfo(PointInfoL,PointInfo,oAvoidRange);
  InterpolatePointInfo(PointInfoR,PointInfo,oAvoidRange);

  double T = (Offset - PointInfoL.Offset) / (PointInfoR.Offset - PointInfoL.Offset);

  return MAX(-1, MIN(T, 1)) * 2 - 1;
}
//==========================================================================*

//==========================================================================*
// Calculate path target
//--------------------------------------------------------------------------*
TVec2d TDriver::CalcPathTarget2(double Pos, double Offset)
{
  TLanePoint PointInfo, PointInfoL, PointInfoR;

  GetLanePoint(oRL_FREE,Pos,PointInfo);
  GetLanePoint(oRL_LEFT,Pos,PointInfoL);
  GetLanePoint(oRL_RIGHT,Pos,PointInfoR);

  InterpolatePointInfo(PointInfoL,PointInfo,oAvoidRange);
  InterpolatePointInfo(PointInfoR,PointInfo,oAvoidRange);

  double T = (Offset - PointInfoL.Offset) / (PointInfoR.Offset - PointInfoL.Offset);

  return TVec2d(MAX(-1, MIN(T, 1)) * 2 - 1, 1);
}
//==========================================================================*

//==========================================================================*
// Calculate path to left and to right
//--------------------------------------------------------------------------*
void TDriver::GetPathToLeftAndRight
  (const PCarElt pCar, double& ToL, double& ToR)
{
  double Pos = pCar->_distFromStartLine;
  double Offset = -pCar->_trkPos.toMiddle;

  TLanePoint PointInfo;
  GetLanePoint(oRL_LEFT,Pos,PointInfo);
  ToL = -(PointInfo.Offset - Offset);
  GetLanePoint(oRL_RIGHT,Pos,PointInfo);
  ToR = PointInfo.Offset - Offset;
}
//==========================================================================*

//==========================================================================*
// Steering angle
//--------------------------------------------------------------------------*
double TDriver::SteerAngle(TLanePoint& AheadPointInfo)
{
  // Look this far ahead.
  double AheadDist = oLookBase + oCurrSpeed * oLookScale;
  if (oGoToPit)
	AheadDist = 2.0;
  double AheadPos = oTrackDesc.CalcPos(oCar, AheadDist);

  // Get info about pts on track.
  GetPosInfo(AheadPos,AheadPointInfo);

  TLanePoint PointInfoOmega;
  double AheadOmega = oOmegaBase + oCurrSpeed * oOmegaScale;
  double AheadOmegaPos = oTrackDesc.CalcPos(oCar, AheadOmega);
  GetPosInfo(AheadOmegaPos,PointInfoOmega);

  // Work out basic steering angle.
  double Angle = AheadPointInfo.Angle - CarYaw;
  DOUBLE_NORM_PI_PI(Angle);

//  if (oCurrSpeed < oSlowSpeed)
//    return Angle;

  double Delta = oLanePoint.Offset + CarToMiddle;

  // Control rotational velocity.
  double AvgK = (oLanePoint.Crv + PointInfoOmega.Crv) / 2;
  double Omega = CarSpeedLong * AvgK;
  double O2 = (AheadPointInfo.Crv - oLanePoint.Crv) * oCurrSpeed / AheadDist;

  Angle += (0.08 * (Omega - CarYawRate + O2) + AvgK) * oScaleSteer;

  // control offset from path.
  if (oStartSteerFactor < 0.15)
    oStartSteerFactor += 0.0002;
  double Factor = MIN(0.15,oStartSteerFactor);
  Angle -= Factor * atan(oPIDCLine.Sample(Delta));
/*
  if (oIndex == 0)
  {
	  GfOut("\nAD: %g AP: %g\n",AheadDist, AheadPos);
	  GfOut("OD: %g OP: %g\n",AheadOmega, AheadOmegaPos);
	  GfOut("AA: %g CY: %g A: %g\n",AheadPointInfo.Angle, CarYaw, AheadPointInfo.Angle - CarYaw);
	  GfOut("D: %g AvgK: %g Omega: %g O2: %g\n",Delta,AvgK,Omega,O2);
	  GfOut("DA: %g\n",(0.06 * (Omega - CarYawRate + O2) + AvgK) * oScaleSteer);
	  GfOut("DA: %g\n",0.15 * atan(oPIDCLine.Sample(Delta)));
	  GfOut("A: %g\n",Angle);
  }
*/
  return Angle;
}
//==========================================================================*

//==========================================================================*
// Unstuck Steering angle
//--------------------------------------------------------------------------*
double TDriver::UnstuckSteerAngle()
{
//  TVec2d oTarget = GetTargetPoint(10.0);         // Target to steer to      
  TVec2d oTarget;                                // Target to steer to      
  if (oAvoidSide != 0.0)
  {
    oTarget = GetAvoidTargetPoint(oAvoidSide,5.0);// Target to steer to      
	//GfOut("AS\n");
  }
  else
    oTarget = GetTargetPoint(10.0);              // Target to steer to      

  // Direction to steer to target
  oSteerAngle = (float) atan2(oTarget.y - CarPosY, oTarget.x - CarPosX);
  oSteerAngle -= (float) CarYaw;				 // Car's rotation (z-axis) 
  FLOAT_NORM_PI_PI(oSteerAngle);				 // Normalize to -PI,+PI
  if (CarSpeedLong < -0.01)
    return -oSteerAngle;
  else
    return oSteerAngle;
}
//==========================================================================*

//==========================================================================*
// Control brake press
//--------------------------------------------------------------------------*
void TDriver::BrakingForceRegulator()
{
/* LearnFriction * /
  double Err = 0.0;
  if(oLastBrake && oLastTargetSpeed)
  {
    Err = oCurrSpeed - oLastTargetSpeed;
	if (Err > 7.0)
	{
      double Pos = oTrackDesc.CalcPos(oCar);     // Get current pos on track
	  int PosIdx = oTrackDesc.IndexFromPos(Pos);
	  //GfOut("L Idx: %d Pos: %g Err: %g\n",PosIdx,Pos,Err);
  	  oTrackDesc.LearnFriction(PosIdx, 0.02, 0.85);
	}
    oBrakeCoeff[oLastBrakeCoefIndex] += (float)(Err * 0.001);
    oBrakeCoeff[oLastBrakeCoefIndex] = (float) MAX(0.5f,MIN(1.0,oBrakeCoeff[oLastBrakeCoefIndex]));
	oLastBrake = 0;
	oLastTargetSpeed = 0;
  }
/ * */
  double Diff = oCurrSpeed - oTargetSpeed + 1;

  if (Diff > 1.0)
  {
	int	B = int(floor(oCurrSpeed/2));
	oLastBrakeCoefIndex = B;
	oLastBrake = oBrake;

	if (Diff > oBrakeDiffInitial)
	{
	  oAccel = 0;
      oBrake = MIN(oBrakeForceMax,oBrakeCoeff[B]/oInitialBrakeCoeff * Diff * Diff / oBrakeScale);
	  // if (oIndex == 0) GfOut("Diff1: %g oBrake: %g\n",Diff,oBrake);
	}
	else
	{
	  if (oTargetSpeed > 1)
	  {
	    oAccel = MIN(oAccel, 0.25);
		oBrake = 0.0;
        // if (oIndex == 0) GfOut("Diff2: %g oBrake: %g\n",Diff,oBrake);
	  }
	  else
	  {
		oAccel = 0;
		oBrake = 0.1;
	    // if (oIndex == 0) GfOut("Diff3: %g oBrake: %g\n",Diff,oBrake);
	  }
	}
  }
/**/
  if ((oLastBrake > 0)
	&& (oBrake > 0)
	&& (Diff < 6))
  {
	//GfOut("*");
	oBrake = 0;
	oAccel = 0.06;
    // if (oIndex == 0) GfOut("Diff4: %g oBrake: %g\n",Diff,oBrake);
  }
/**/
  oLastBrake = oBrake;
  oLastTargetSpeed = 0;
  if (oBrake > 0)
  {
    if (oTargetSpeed > 0)
  	  oLastTargetSpeed = oTargetSpeed;
  }

  oBrake *= (1 + MAX(0.0,(oCurrSpeed - 40.0)/40.0));
  double BFactor = 1.0;
  if (oPreviewSlow)
    BFactor = 1.03;
  else if (oBrakeForTeamMate)
    BFactor = 1.03;
  else if (oMinDistLong < 10.0)
	BFactor = 1.03;
  oBrake *= BFactor;
}
//==========================================================================*

//==========================================================================*
// Control brake press while avoiding
//--------------------------------------------------------------------------*
void TDriver::BrakingForceRegulatorAvoid()
{
  double Err = 0.0;
  if(oLastBrake && oLastTargetSpeed)
  {
/*
    Err = oCurrSpeed - oLastTargetSpeed;
	if (Err > 2.0)
	{
      double Pos = oTrackDesc.CalcPos(oCar);     // Get current pos on track
	  //int PosIdx = oTrackDesc.IndexFromPos(Pos);
	  //GfOut("A Idx: %d Pos: %g Err: %g\n",PosIdx,Pos,Err);
	}
*/
/*
	oBrakeCoeff[oLastBrakeCoefIndex] += (float)(Err * 0.001);
    oBrakeCoeff[oLastBrakeCoefIndex] = (float) MAX(0.5f,MIN(1.0,oBrakeCoeff[oLastBrakeCoefIndex]));
*/
	oLastBrake = 0;
	oLastTargetSpeed = 0;
  }

  double Diff = oCurrSpeed - oTargetSpeed + 1;

  if (Diff > 1.0)
  {
	int	B = int(floor(oCurrSpeed/2));
	oLastBrakeCoefIndex = B;
	oLastBrake = oBrake;

    if (Diff > 1.0)
	{
	  oAccel = 0;
      oBrake = MIN(oBrakeForceMax,oBrakeCoeff[B]/oInitialBrakeCoeff * Diff * Diff * Diff / oBrakeScale);
	  // if (oIndex == 0) GfOut("Diff5: %g oBrake: %g\n",Diff,oBrake);
	}
	else
	{
	  if (oTargetSpeed > 1)
	  {
		oAccel = MIN(oAccel, 0.25);
        oBrake = 0.0;
        // if (oIndex == 0) GfOut("Diff6: %g oBrake: %g\n",Diff,oBrake);
	  }
	  else
	  {
		oAccel = 0;
		oBrake = 0.1;
        // if (oIndex == 0) GfOut("Diff7: %g oBrake: %g\n",Diff,oBrake);
	  }
	}
  }

  oLastTargetSpeed = 0;
  if ((oBrake > 0) && (oBrake < oBrakeForceMax))
  {
    if (oTargetSpeed > 0)
  	  oLastTargetSpeed = oTargetSpeed;
  }


  oBrake *= (1 + MAX(0.0,(oCurrSpeed - 40.0)/40.0));

  double BFactor = 1.0;
  if (oPreviewSlow)
    BFactor = 1.03;
  else if (oBrakeForTeamMate)
    BFactor = 1.03;
  else if (oMinDistLong < 10.0)
	BFactor = 1.03;
  oBrake *= BFactor;
}
//==========================================================================*

//==========================================================================*
// Control brake press in traffic
//--------------------------------------------------------------------------*
void TDriver::BrakingForceRegulatorTraffic()
{
  double Err = 0.0;
  if(oLastBrake && oLastTargetSpeed)
  {
/*
    Err = oCurrSpeed - oLastTargetSpeed;
	if (Err > 2.0)
	{
      double Pos = oTrackDesc.CalcPos(oCar);     // Get current pos on track
	  //int PosIdx = oTrackDesc.IndexFromPos(Pos);
	  //GfOut("T Idx: %d Pos: %g Err: %g\n",PosIdx,Pos,Err);
	}
*/
/*
    oBrakeCoeff[oLastBrakeCoefIndex] += (float)(Err * 0.001);
    oBrakeCoeff[oLastBrakeCoefIndex] = (float) MAX(0.5f,MIN(1.0,oBrakeCoeff[oLastBrakeCoefIndex]));
*/
	oLastBrake = 0;
	oLastTargetSpeed = 0;
  }

  double Diff = oCurrSpeed - oTargetSpeed + 1;

  if (Diff > 1.0)
  {
	int	B = int(floor(oCurrSpeed/2));
	oAccel = 0;
	oBrake = MAX(0, MIN(oBrakeCoeff[B]/oInitialBrakeCoeff * Diff * Diff * Diff, oBrakeForceMax));
//    Err = 1.0 + MIN(Err,0.5);
//    oBrake = Err * oBrake;
	oLastBrakeCoefIndex = B;
	oLastBrake = oBrake;
	oLastTargetSpeed = 0;
	// if (oIndex == 0) GfOut("Diff8: %g oBrake: %g CS: %g TS: %g Err: %g\n",Diff,oBrake,oCurrSpeed,oTargetSpeed,Err);

//	if ((oBrake > 0) && (oBrake < oBrakeForceMax))
	if (oBrake > 0)
	{
	  if (oTargetSpeed > 0)
		oLastTargetSpeed = oTargetSpeed;
	}
  }
  else
  {
	// if (oIndex == 0) GfOut("Diff9: %g oBrake: %g CS: %g TS: %g Err: %g\n",Diff,oBrake,oCurrSpeed,oTargetSpeed,Err);
  }

  oBrake *= (1 + MAX(0.0,(oCurrSpeed - 40.0)/40.0));

  double BFactor = 1.0;
  if (oPreviewSlow)
    BFactor = 1.03;
  else if (oBrakeForTeamMate)
    BFactor = 1.03;
  else if (oMinDistLong < 10.0)
	BFactor = 1.03;
  oBrake *= BFactor;
}
//==========================================================================*

//==========================================================================*
// Evaluate collision flags
//--------------------------------------------------------------------------*
void TDriver::EvaluateCollisionFlags(
  int OppIndx,
  TCollision::TCollInfo& Coll,
  double Crv,
  double& MinCatchTime,
  double& MinCatchAccTime,
  double& MinVCatTime,
  bool& IsLapper)
{
  TOpponent::TInfo& OppInfo =                    // Information about opponent
    oOpponents[OppIndx].Info();                  // collected at classification

  PCarElt OppCar = oOpponents[OppIndx].Car();    // TORCS data of opponents car

  Coll.Flags |= OppInfo.Flags;                   // subsume the collision flags
  for (int I = 0; I < MAXBLOCKED; I++)
    Coll.Blocked[I] |= OppInfo.Blocked[I];       // subsume the blocked flags

  if (OppInfo.GotFlags(F_FRONT))                 // Is opponent in front of us
  {
	if (OppInfo.GotFlags(F_CLOSE))
      Coll.OppsNearInFront += 1;
    if (oMinDistLong > OppInfo.State.CarDistLong)
	  oMinDistLong = (float) OppInfo.State.CarDistLong;

	if (OppInfo.GotFlags(F_COLLIDE)
	  && (OppInfo.CatchDecel > 12.5 * CarFriction))
	  Coll.TargetSpeed = MIN(Coll.TargetSpeed, OppInfo.CatchSpeed);

	if (OppInfo.Flags & (F_COLLIDE | F_CATCHING))
	  MinCatchTime = MIN(MinCatchTime, OppInfo.CatchTime);

	if (OppInfo.Flags & F_CATCHING_ACC)
	  MinCatchAccTime = MIN(MinCatchAccTime, OppInfo.CatchAccTime);

	if (OppInfo.State.CarDiffVelLong < 0)
	{
	  double VCatTime =
		-(OppInfo.State.CarDistLong - OppInfo.State.MinDistLong) / OppInfo.State.CarDiffVelLong;

	  if (VCatTime > 0)
	    MinVCatTime = MIN(MinVCatTime, VCatTime);
	}

	bool IgnoreTeamMate =
	  OppInfo.GotFlags(F_TEAMMATE)
	  && ((CarLaps < OppCar->_laps
	  || CarDamage + 1000 >= OppInfo.TeamMateDamage));

	OppInfo.AvoidLatchTime = MAX(0, OppInfo.AvoidLatchTime - oSituation->deltaTime);

	double MaxSpdCrv = Param.Fix.CalcMaxSpeedCrv();
//	double ColTime = fabs(Crv) > MaxSpdCrv ? 0.5 : 0.7;
//	double CatTime = fabs(Crv) > MaxSpdCrv ? 0.5 : 2.5;
//	double CacTime = fabs(Crv) > MaxSpdCrv ? 0.5 : 2.5;

//	double ColTime = fabs(Crv) > MaxSpdCrv ? 1.5 : 1.7;
//	double CatTime = fabs(Crv) > MaxSpdCrv ? 1.5 : 3.5;
//	double CacTime = fabs(Crv) > MaxSpdCrv ? 1.5 : 3.5;

	double ColTime = fabs(Crv) > MaxSpdCrv ? 1.0 : 1.2;
	double CatTime = fabs(Crv) > MaxSpdCrv ? 1.0 : 3.0;
	double CacTime = fabs(Crv) > MaxSpdCrv ? 1.0 : 3.0;

	bool Catching =
	    ((OppInfo.CatchTime < ColTime) && OppInfo.GotFlags(F_COLLIDE))
	    || ((OppInfo.CatchTime < CatTime) && OppInfo.GotFlags(F_CATCHING))
	    || ((OppInfo.CatchAccTime < CacTime) && OppInfo.GotFlags(F_CATCHING_ACC))
		|| ((OppInfo.CatchSpeed < 0.95 * oTargetSpeed) && (OppInfo.State.RelPos < 30));

	if (!IgnoreTeamMate &&
	  (OppInfo.AvoidLatchTime > 0 || Catching || OppInfo.GotFlags(F_DANGEROUS)))
	{
	  double ToL, ToR;

	  GetPathToLeftAndRight(OppCar, ToL, ToR);
	  ToL += OppInfo.State.TrackVelLat * OppInfo.CatchTime;
	  ToR -= OppInfo.State.TrackVelLat * OppInfo.CatchTime;
//	  bool SpaceL = ToL > OppInfo.State.MinDistLat;// + 0.25;
//	  bool SpaceR = ToR > OppInfo.State.MinDistLat;// + 0.25;
	  bool SpaceL = ToL > OppInfo.State.MinDistLat + 0.75;// + 0.25;
	  bool SpaceR = ToR > OppInfo.State.MinDistLat + 0.75;// + 0.25;
	  bool AvoidL = OppInfo.State.CarDistLat < 0 && SpaceR;
	  bool AvoidR = OppInfo.State.CarDistLat > 0 && SpaceL;

	  if (Catching)
//	    OppInfo.AvoidLatchTime = fabs(Crv) < MaxSpdCrv ? 0.5 : 0.1;
	    OppInfo.AvoidLatchTime = fabs(Crv) < MaxSpdCrv ? 2.0 : 0.5;

	  if (fabs(Crv) < MaxSpdCrv)
	  {
	    if (!AvoidL && !AvoidR)
	    {
	      AvoidL = !SpaceL && SpaceR;
	      AvoidR = !SpaceR && SpaceL;
	    }
      }

	  if (AvoidL)
	  {
	    Coll.OppsAhead |= F_LEFT;
		Coll.MinLDist = MIN(OppInfo.State.CarAvgVelLong, Coll.MinLDist);
	  }

	  if (AvoidR)
	  {
		Coll.OppsAhead |= F_RIGHT;
		Coll.MinRDist = MIN(OppInfo.State.CarAvgVelLong, Coll.MinRDist);
	  }
	}
  }

  if (OppInfo.GotFlags(F_BEHIND))                // Is Opponent near behind us
  {
	if (OppInfo.GotFlags(F_CLOSE))
      Coll.OppsNearBehind += 1;
  }

  if (OppInfo.GotFlags(F_AT_SIDE))               // Is Opponent at side of us
  {
    if (DistanceRaced > 100.0
      && OppInfo.GotFlags(F_FRONT))
      oNoAvoidLength = 0;
	Coll.OppsAtSide |= OppInfo.State.CarDistLat < 0 ? F_LEFT : F_RIGHT;
	if (OppInfo.State.CarDistLat < 0)
	  Coll.MinLSideDist = MIN(Coll.MinLSideDist,
	    -OppInfo.State.CarDistLat - OppInfo.State.MinDistLat);
	else
	  Coll.MinRSideDist = MIN(Coll.MinRSideDist,
	    OppInfo.State.CarDistLat - OppInfo.State.MinDistLat);

    oBrakeTeamMateAside = oBrakeTeamMateAside
  	  || (OppInfo.GotFlags(F_TEAMMATE) && (OppInfo.State.RelPos > 0))
  	  || ((CarDamage > OppInfo.TeamMateDamage) && (CarLaps <= OppCar->_laps));
  }

  oBrakeForTeamMate = oBrakeForTeamMate
	|| (OppInfo.GotFlags(F_TEAMMATE | F_FRONT) && (OppInfo.State.RelPos < 50));

  oTreatTeamMateAsLapper =
	OppInfo.GotFlags(F_TEAMMATE | F_REAR)
	&& OppInfo.State.RelPos > -35
	&& CarLaps <= OppCar->_laps
	&& CarDamage > OppInfo.TeamMateDamage + 500;
//	&& CarDamage > OppInfo.TeamMateDamage + 1000;

  if (oStayTogether > 50
	&& OppInfo.GotFlags(F_TEAMMATE | F_REAR)
//	&& OppInfo.State.RelPos < -25
	&& OppInfo.State.RelPos < -35
	&& OppInfo.State.RelPos > -oStayTogether
//	&& CarLaps >= OppCar->_laps
	&& CarDamage + 500 > OppInfo.TeamMateDamage)
//	&& CarDamage + 2500 > OppInfo.TeamMateDamage)
  {
	IsLapper = true;
  }

  if (OppInfo.GotFlags(F_LAPPER) || oTreatTeamMateAsLapper)
  {
	Coll.LappersBehind |= OppInfo.State.CarDistLat < 0 ? F_LEFT : F_RIGHT;
	IsLapper = true;
  }
}
//==========================================================================*

//==========================================================================*
// Get next curvature to opponent
//--------------------------------------------------------------------------*
void TDriver::NextCurvature(TCollision::TCollInfo& Coll, PtCarElt Car)
{
  int OppPosIndex = oTrackDesc.IndexFromPos(oTrackDesc.CalcPos(Car));
  Coll.NextSide =
	(oRacingLine[oRL_FREE].PathPoints(OppPosIndex).Crv < 0) ? -1 : 0;
}
//==========================================================================*

//==========================================================================*
// Target Reached
//--------------------------------------------------------------------------*
bool TDriver::TargetReached(double Target, double AvoidTarget)
{
  if(((oAvoidRange != 0.0)
	&& (Target == 0))
	|| ((AvoidTarget != oAvoidRange)
	&& (Target != 0)))
    return false;
  else
    return true;
}
//==========================================================================*

//==========================================================================*
// Runaround most urgent obstacle
//--------------------------------------------------------------------------*
void TDriver::Runaround(double Scale, double Target, bool DoAvoid)
{
  // Scale limits of change of lateral movement (accellerations/velocities)
/*
  double RangeAccMax = 0.00075 * Scale;          // Range accelleration and
//  double RangeVelMax = 0.005 * Scale;            // velocity per sim.step
  double RangeVelMax = 0.010 * Scale;            // velocity per sim.step
//  double OffsetAccMax = 0.0003 * Scale;          // Offset accelleration and
  double OffsetAccMax = 0.0002 * Scale;          // Offset accelleration and
//  double OffsetAccMax = 0.0001 * Scale;          // Offset accelleration and
  double OffsetVelMax = 0.2 * Scale;             // velocity per sim.step
//double OffsetVelMax = 0.05 Scale;             // velocity per sim.step
*/
/*
  double RangeAccMax = 0.0005 * Scale;           // Range accelleration and
  double RangeVelMax = 0.005 * Scale;            // velocity per sim.step
  double OffsetAccMax = 0.00015 * Scale;         // Offset accelleration and
  double OffsetVelMax = 0.1 * Scale;             // velocity per sim.step
*/
/*
  double RangeAccMax = 0.00075 * Scale;          // Range acceleration and
  double RangeVelMax = 0.005 * Scale;            // velocity per sim.step
  double OffsetAccMax = 0.0003 * Scale;          // Offset acceleration and
  double OffsetVelMax = 0.2 * Scale;             // velocity per sim.step
*/
/*
  double RangeAccMax = 0.00025 * Scale;          // Range acceleration and
  double RangeVelMax = 0.005 * Scale;            // velocity per sim.step
  double OffsetAccMax = 0.000025 * Scale;        // Offset acceleration and
  double OffsetVelMax = 0.1 * Scale;             // velocity per sim.step
*/
  double RangeAccMax = 0.00075 * Scale;          // Range accelleration and
  double RangeVelMax = 0.010 * Scale;            // velocity per sim.step
  double OffsetAccMax = 0.0002 * Scale;          // Offset accelleration and
  double OffsetVelMax = 0.2 * Scale;             // velocity per sim.step

  double AvoidTarget = 0;                        // Assume come back to RL
  if (DoAvoid)                                   // But if needed
    AvoidTarget = 2.0;                           //   avoid to side
/*
  else
  {
    OffsetAccMax /= 4;                           // Offset accelleration and
    OffsetVelMax /= 4;                           // velocity per sim.step
  }
*/
  // Adjusting allowed range of offset ...
  if (!TargetReached(Target,AvoidTarget))        // Target Range reached?
  {
	AvoidTarget = (Target == 0) ? 0 : 1;         // Direction to move to
    double LatAccel = oAvoidRange > AvoidTarget  // Xceleration of changing
      ? RangeAccMax : -RangeAccMax;              // the range
    double Dist =                                // Remaining distance to
	  oAvoidRange - AvoidTarget;                 //   target range

	if (fabs(Dist) < 0.0005)                     // If close to target
	{
      oAvoidRangeDelta = 0.0;                    //   don't change any longer
	}
	else
	{
	  if (fabs(Dist) <=                          // Check wether to decellerate
		XX2Y(oAvoidRangeDelta,RangeAccMax))      // enlargement/contracting
	  {
		LatAccel = -XX2Y(oAvoidRangeDelta,Dist); // Decelerate
	  }
	  oAvoidRangeDelta += LatAccel;              // Accellerate or Decellerate
      oAvoidRangeDelta =                         // Restrict to limits
	    MINMAX(RangeVelMax,oAvoidRangeDelta);
	}
  }
  else                                           // If target is reached
	oAvoidRangeDelta = 0;                        //   stop changing

  double OldAvoidRange = oAvoidRange;            // Save old range
  oAvoidRange -= oAvoidRangeDelta;               // Set new range

  if ((oAvoidRange > 0.9995)                     // If close to Max
	&& (oAvoidRangeDelta <= 0))                  // fix range and change
  {
    oAvoidRange = 1.0;                           // Max reached
	oAvoidRangeDelta = 0.0;                      // Stop changing
  }
  else if((oAvoidRange <= 0.0005)                // If close to Min
	&& (oAvoidRangeDelta >= 0))                  // fix range and change
  {
	oAvoidRange = 0.0;                           // Min reached
	oAvoidRangeDelta = 0.0;                      // Stop changing
  }
  else if (FixRange)                             // If close to target
  {
	//GfOut("oAvoidRange: %g <= AvoidTarget %g\n",oAvoidRange,AvoidTarget);
	oAvoidRange = AvoidTarget;                   // Target reached
	oAvoidRangeDelta = 0.0;                      // Stop changing
  }

  if ((Target == 0) && (oAvoidRange == 0))
  {
    oAvoidOffset = 0;                            // reset and
    oAvoidOffsetDelta = 0;                       //   stop changing
  }
	  // Adjusting offset ...
  if (Target != oAvoidOffset)                    // Target Offset reached?
  {
 	double LatAccel =                            // Xcelleration of lateral movement
	  OffsetAccMax / MAX(0.2, oAvoidRange);
	LatAccel = Target > oAvoidOffset             // Accelleate or Decellerate?
	  ? LatAccel : -LatAccel;
	double Dist = Target - oAvoidOffset;         // Distance to target offset
	if ((Dist * oAvoidOffsetDelta > 0)           // Check wether to decellerate
      && (fabs(Dist) <=                          //   lateral movement
	    XX2Y(oAvoidOffsetDelta,OffsetAccMax)))
	{
	  LatAccel = -XX2Y(oAvoidOffsetDelta,Dist);  // Decellerate
	}
	LatAccel = MINMAX(OffsetAccMax,LatAccel);    // Restrict to limits
	oAvoidOffsetDelta += LatAccel;               // Accellerate or decellerate
    oAvoidOffsetDelta =                          // Restrict to limits
	    MINMAX(OffsetVelMax,oAvoidOffsetDelta);
  }
  else                                           // If target offset reached
    oAvoidOffsetDelta = 0;                       //   stop changing

  double OldAvoidOffset = oAvoidOffset;          // Save old offset
    oAvoidOffset += oAvoidOffsetDelta;           // Set new offset

  if ((oAvoidOffset < -0.99)                     // If close to Min
	&& (Target <= 0.0))                          // of target dir
  {
	oAvoidOffset = -1;                           // Min reached
	oAvoidOffsetDelta = 0.0;                     // Stop changing
  }
  else if ((oAvoidOffset > 0.99)                 // If close to Max
	&& (Target >= 0.0))                          // of target dir
  {
	oAvoidOffset = 1;                            // Max reached
	oAvoidOffsetDelta = 0.0;                     // Stop changing
  }
  else if (FixOffset)                            // If close to target
  {
	//GfOut("oAvoidOffset: %g <= Target %g\n",oAvoidOffset,Target);
	oAvoidOffset = Target;                       // Target reached
 	oAvoidOffsetDelta = 0.0;                     // Stop changing
  }

  //if (oIndex == 0)
  //  GfOut("T:%5.2g AR:%8.3g ARD:%8.3g AO:%8.3g AOD:%8.3g Scale:%8.3g %s\n",Target,oAvoidRange,oAvoidRangeDelta,oAvoidOffset,oAvoidOffsetDelta,Scale,oBotName);
}
//==========================================================================*

//==========================================================================*
// General avoidance controll
//--------------------------------------------------------------------------*
void TDriver::AvoidOtherCars(double K, bool& IsClose, bool& IsLapper)
{
  const TOpponent::TState& MyState =             // Get my own state
	oOpponents[oOwnOppIdx].Info().State;

  int I;
  for (I = 0; I < oNbrCars; I++)                 // All opponents
	for (int J = 0; J < MAXBLOCKED; J++)		 //   all lanes
	  oOpponents[I].Info().Blocked[J] = false;

  for (I = 0; I < oNbrCars; I++)                 // Get info about imminent
  {                                              //   collisions from
      oOpponents[I].Classify(                    //   all opponents depending
	  oCar,                                      //   on TORCS data of own car
	  MyState,                                   //   my own state,
	  oStrategy->OutOfPitlane(),                 //   In pitlane?
	  oMaxAccel.Estimate(CarSpeedLong));         //   Estimate of accelleration
  }

  // Place to subsume the collision flags from all opponents
  TCollision::TCollInfo Coll;  

  double MinCatchTime = FLT_MAX;                 // Initialize limits
  double MinCatchAccTime = FLT_MAX;
  double MinVCatTime = FLT_MAX;

  IsLapper = false;                              // Initialize flags

  TLanePoint PointInfo;                          // Infos to point
  GetLanePoint                                   //   own position
	(oRL_FREE,DistanceFromStartLine,PointInfo);
/*
  if ((oShowIndex) && (oIndex == 0))
  {
	if ((oExcludeFrom < oSecIndex) && (oExcludeTill > oSecIndex))
	  GfOut("X D: %.0f P: %d O: %g\n",DistanceFromStartLine,PointInfo.Index,PointInfo.Offset);
	else
	  GfOut("O D: %.0f P: %d O: %g\n",DistanceFromStartLine,PointInfo.Index,PointInfo.Offset);
  }
*/
  oBrakeForTeamMate = false;
  oBrakeTeamMateAside = false;
  oMinDistLong = FLT_MAX;
  //if (CurrSimTime > 0.2)
  if (CurrSimTime > 0.2)
  {
    for (I = 0; I < oNbrCars; I++)                 // Loop all opponents
    {
      EvaluateCollisionFlags(                      // Evaluate collision flags
        I, Coll, K,
	    MinCatchTime,
	    MinCatchAccTime,
	    MinVCatTime,
	    IsLapper);
    }
  }

  oOppsNearBehind = Coll.OppsNearBehind;
  oOppsNearInFront = Coll.OppsNearInFront;
  //if (oOppsNearInFront + oOppsNearBehind > 0)
  //  GfOut("%d+%d-%d\n",oIndex,oOppsNearInFront,oOppsNearBehind);

  oAvoidSide = 0;
  oPreviewSlow = false;
/**/
  if (((Coll.Flags & F_PREVIEWSLOW) != 0)
    && (DistanceRaced > 1000))
  {
    oPreviewSlow = true;
    Coll.AvoidSide = 0;
	int I;
	int K = 0;
	if (Coll.Blocked[0] || Coll.Blocked[1] || Coll.Blocked[2])
	{
      for (I = MAXBLOCKED - 1; I > 1; I--)
      {
		K++;
	    if (Coll.Blocked[I] || Coll.Blocked[I-1] || Coll.Blocked[I-2])
		  continue;
	    else
	    {
	      Coll.AvoidSide = ((I-1) * 2.0/(MAXBLOCKED - 1.0) - 1);
	      break;
	    }
  	  }
	}
	else
	{
      for (I = 0; I < MAXBLOCKED - 2; I++)
      {
		K++;
	    if (Coll.Blocked[I] || Coll.Blocked[I+1] || Coll.Blocked[I+2])
		  continue;
	    else
	    {
	      Coll.AvoidSide = ((I+1) * 2.0/(MAXBLOCKED - 1.0) - 1);
	      break;
	    }
  	  }
	}

	oAvoidSide = Coll.AvoidSide;

	//GfOut("K: %d (%g) spsc: %g %g %g\n",K,Coll.AvoidSide,oSpeedScale,oTargetSpeed,Coll.TargetSpeed);
    //GfOut(">");
	int FreeLanes = 0;
	int Lanes = 0;
    for (I = 0; I < MAXBLOCKED; I++)
	{
      if (Coll.Blocked[I])
	  {
	    //GfOut("X");
		FreeLanes = MAX(FreeLanes,Lanes);
		Lanes = 0;
	  }
	  else
	  {
	    //GfOut(" ");
		Lanes++;
	  }
	}
	FreeLanes = MAX(FreeLanes,Lanes);
    //GfOut("< %d\n",FreeLanes);

//	if (K == MAXBLOCKED - 4)
	if (FreeLanes < MAXBLOCKED / 2)
	{
//      oSpeedScale += 0.00025;
      oSpeedScale += 0.002;
      Coll.TargetSpeed = MIN(Coll.TargetSpeed, MAX(0.5,(1 - oSpeedScale)) * oTargetSpeed);
	  //GfOut("-?(%g) spsc: %g %g %g\n",Coll.AvoidSide,oSpeedScale,oTargetSpeed,Coll.TargetSpeed);
	}
	else if (FreeLanes < 3)
	{
//      Coll.TargetSpeed = 2.5;
      Coll.TargetSpeed = MIN(Coll.TargetSpeed, MAX(0.25,(1 - oSpeedScale)) * oTargetSpeed);
	  //GfOut("+?(%g) spsc: %g %g %g\n",Coll.AvoidSide,oSpeedScale,oTargetSpeed,Coll.TargetSpeed);
	}
	//  GfOut("%d(%g) spsc: %g %g %g\n",I,Coll.AvoidSide,oSpeedScale,oTargetSpeed,Coll.TargetSpeed);
  }
/**/
  if ((Coll.Flags & F_PREVIEWSLOW) == 0)
    oSpeedScale = 0.0;

  NextCurvature(Coll,oCar);                      // Find side of next curvature

  bool DoAvoid = oDoAvoid;
  if (oAvoidSide != 0)
    oDoAvoid = true;                             // Assume avoiding
  else
    oDoAvoid = false;                            // Assume not avoiding

  TCollision RunAround;                          // To runaround we have to decide
  double Target = 0.0;                           // which way to take
  float Ratio = 0.0;

  if (DistanceRaced > oNoAvoidLength)
  {

	if (oSideCounter > 0)
	{
      oSideCounter--;
      if (oSideCounter < 0)
        oSideCounter = 0;
	  else
        Target = oLastTarget;
	}
	else
	{
      Target = RunAround.AvoidTo                 // Check which way we should take
        (Coll,oCar,*this,oDoAvoid);              //   depending on opponents
//      oSideCounter = 10;
      oSideCounter = 0;
	}
/*
    if ((Coll.Flags & F_DANGEROUS) != 0)
	{
		oTargetSpeed *= 0.80;
	}

	if (oDoAvoid)
	{
		oTargetSpeed *= 0.95;
		//GfOut("Pos: %g m  oTargetSpeed: %g km/h (%g)\n",DistanceFromStartLine,oTargetSpeed*3.6,oCurrSpeed*3.6);
	}
*/
	oLastTarget = Target;
  }
  else
  {
//    Target = oStartSide * -1 * oIndex;           // side to 
    Target = oStartSide;                         // side to 
	oDoAvoid = true;                             // go to
  }

  if (oStrategy->StartPitEntry(Ratio))           // If entrering pit
  {
	if (!oDoAvoid)                               // If no avoiding needed
	{
      Target = Ratio * PitSide();                // Bring us to the correct
	  oDoAvoid = true;                           // side to make pit stop
	}
  }
  else if (oStrategy->StopPitEntry(Param.Pit.oExitLength)) // If coming back to track
  {
	if (!oDoAvoid)                                // If no avoiding needed
	{
      Target = PitSide();                        // Bring us to the correct
	  oDoAvoid = true;                           // side to make pit stop
	}
  }

  oTargetSpeed =                                 // Adjust target speed
	MIN(oTargetSpeed, Coll.TargetSpeed);

  IsClose = (Coll.Flags & F_CLOSE) != 0;         // Set flag, if opponent is close by

  double HalfWidth = oTrackDesc.Width() / 2;     // Half width of track
  double Scale =                                 // Scale reaction
	oAvoidScale / (HalfWidth - oAvoidWidth);

  if (oFlying)                                   // Sorry, but we can't do anything
    return;                                      //   right now, we are flying!

  Runaround(Scale,Target,oDoAvoid);              // runaround most urgent obstacle
}
//==========================================================================*

//==========================================================================*
// Relax mode
//--------------------------------------------------------------------------*
double TDriver::FilterRelax(double TargetSpeed)
{
  if (!Qualifying)
  {
	if (oCar->race.timeBeforeNext > 20)
	{
      if (oCar->race.pos < 3)
	  {
        TargetSpeed *= 0.98;
        //GfOut("\n%d(%d): %.3fs %.1f km/h\n",oIndex,oCar->race.pos,oCar->race.timeBeforeNext,oTargetSpeed*3.6);
        //GfOut("R%d",oIndex);
	  }
	}
  }
  return TargetSpeed;
}
//==========================================================================*

//==========================================================================*
// Limit steer speed
//--------------------------------------------------------------------------*
double TDriver::FilterSteerSpeed(double Steer)
{
  if (oCurrSpeed < 20)
	return Steer;

  const float MaxSteerSpeed = 0.1f;
//  float MaxSteerSpeed = (float) (300.0 / (oCurrSpeed*oCurrSpeed));
  double Ratio = fabs(oLastSteer - Steer)/MaxSteerSpeed;

  if (Ratio > 1.0)
  {
    if (Steer > oLastSteer)
      Steer = oLastSteer + MaxSteerSpeed;
    else
      Steer = oLastSteer - MaxSteerSpeed;
  }

  double Range = MIN(1.0,0.3 + 1250.0/(oCurrSpeed*oCurrSpeed));

  if (Steer > 0)
    Steer = MIN(Range,Steer);
  else
    Steer = MAX(-Range,Steer);

  return Steer;
}
//==========================================================================*

//==========================================================================*
// ABS
//--------------------------------------------------------------------------*
double TDriver::FilterABS(double Brake)
{
  if(CarSpeedLong < 10)
	return Brake;

  double Slip = 0.0;

  for (int I = 0; I < 4; I++)
	Slip += WheelSpinVel(I) * WheelRad(I);

  Slip = 4.0 * CarSpeedLong / Slip;

  if (Slip > oAbsDelta)
  {
	Brake *= oAbsScale;
  }

  return Brake;
}
//==========================================================================*

//==========================================================================*
// Filter Brake while drifting
//--------------------------------------------------------------------------*
double TDriver::FilterBrake(double Brake)
{
  double MaxBrake = 0.0;

  // Watch brake lights
  for (int I = 0; I < oNbrCars; I++)
  {
    if (I == oOwnOppIdx)
	  continue;

    PCarElt OppCar = oOpponents[I].Car();    
//    if (!oCommonData->TeamManager.IsTeamMate(OppCar,oCar))
//	  continue;

    TOpponent::TInfo& OppInfo =          
      oOpponents[I].Info();        

	if (fabs(OppInfo.State.CarDistLat) > OppInfo.State.MinDistLat)
	  continue;

    if ((OppInfo.State.RelPos > 15.0) || (OppInfo.State.RelPos < 0.0))
	  continue;

    if (!oCommonData->TeamManager.IsTeamMate(OppCar,oCar))
	  MaxBrake = MAX(MaxBrake, OppCar->_brakeCmd);
	else
	  MaxBrake = MAX(MaxBrake, OppCar->_brakeCmd*0.8);
  }

  Brake = MAX(Brake, MaxBrake*0.8);

  // If braking, decrease braking force while drifting
  if((CarSpeedLong > oSlowSpeed) && (Brake > 0.0))
  {
    double DriftAngle = MAX(MIN(oDriftAngle * 2, PI),-PI);
	double CosAngle = cos(DriftAngle);
	if ((CosAngle < 0.98f) || (fabs(oSteer) > 0.002))
	{
      Brake *= MAX(0.1, 0.7*CosAngle);
	  //GfOut("+");
	}
	else
	{
	  //GfOut("*");
	}
  }

  if (oBrakeTeamMateAside && oSecond) 
    Brake += 0.05;

  return Brake;
}
//==========================================================================*

//==========================================================================*
// Filter Traction Control
//--------------------------------------------------------------------------*
double TDriver::FilterTCL(double Accel)
{
//  double Delta = 1.05;
  double Delta = 1.10;

  if (DistanceRaced < 50)                        // Not at start
	return Accel;

  if(fabs(CarSpeedLong) < 0.001)                 // Only if driving faster
	return Accel;

  double Spin = 0;                               // Initialize spin
  double Wr = 0;                                 // wheel radius
  int Count = 0;                                 // count impellers

  if(HasDriveTrainFront)                         // If front wheels
  {                                              //   are impellers
	Spin += WheelSpinVel(FRNT_LFT);              // Summarize spin
	Spin += WheelSpinVel(FRNT_RGT);              // of both wheels
	Wr += WheelRad(FRNT_LFT)+WheelRad(FRNT_RGT); // measure radius
	Count += 2;                                  // and count both
  }

  if(HasDriveTrainRear)                          // If rear wheels
  {                                              //   are impellers
	Spin += WheelSpinVel(REAR_LFT);              // Summarize spin
	Spin += WheelSpinVel(REAR_RGT);              // of both wheels
	Wr += WheelRad(REAR_LFT)+WheelRad(REAR_RGT); // measure radius
	Count += 2;                                  // and count both
  }
  Spin /= Count;                                 // Calculate spin
  Wr /= Count;                                   // and radius

  double Slip = Spin * Wr - CarSpeedLong;        // Calculate slip
  if ((oUnstucking) && (Slip > oTclSlip * 0.6))  // Decrease accel if needed)
  {
    if (Accel > oLastAccel * Delta)
      Accel = MIN(Accel,MAX(0.05,oLastAccel*Delta));
	if (CarRpm > CarRpmLimit / 2)
      Accel = MIN(Accel,0.5);
  }
  else if (oUnstucking)                          // Decrease accel if needed)
  {
    if (Accel > oLastAccel * Delta)
      Accel = MIN(Accel,MAX(0.05,oLastAccel*Delta));
	if (CarRpm > CarRpmLimit / 2)
      Accel = MIN(Accel,0.7);
  }
  else if (Slip > oTclSlip)                      // Decrease accel if needed
	Accel -= MIN(Accel, (Slip - oTclSlip)/oTclRange);

  //GfOut("Friction: %g\n",CarSeg->surface->kFriction);
  if ((CarSeg->surface->kFriction < 1.0) && (fabs(CarSeg->radius) < 50))
  {
    if (Accel > oLastAccel * 1.20)
      Accel = MIN(Accel,MAX(0.1,oLastAccel*1.20));
  }
  else if (CarSeg->surface->kFriction < 1.0)
  {
    if (Accel > oLastAccel * 1.3)
      Accel = MIN(Accel,MAX(0.1,oLastAccel*1.3));
  }
  return Accel;
}
//==========================================================================*

//==========================================================================*
// Filter different segements
//--------------------------------------------------------------------------*
double TDriver::FilterDiff(double Accel)
{
  Accel *= oSideReduction;
//  if (Accel > oLastAccel * 1.005)
//    Accel = MIN(Accel,MAX(0.1,oLastAccel*1.005));
  if (Accel > oLastAccel * 1.05)
    Accel = MIN(Accel,MAX(0.1,oLastAccel*1.05));

  return Accel;
}
//==========================================================================*

//==========================================================================*
// Filter acceleration in curves
//--------------------------------------------------------------------------*
double TDriver::FilterCrv(double Accel)
{
  const double factor0 = 1.008 * 2.0;
  const double factor1 = 1.027 * 2.2;
  const double factor2 = 1.090 * 2.4; 
/**/
  if (fabs(oLanePoint.Crv) > 0.035)
  { 
    if (Accel > oLastAccel * factor0)
	{
      Accel = MIN(Accel,MAX(0.1,oLastAccel*factor0));
	  //if (strncmp(oBotName,"Lobo Malo",9) == 0)
	  //  GfOut("*");
	}
  }
  else if (fabs(oLanePoint.Crv) > 0.0325)
  {
    if (Accel > oLastAccel * factor1)
	{
      Accel = MIN(Accel,MAX(0.1,oLastAccel*factor1));
	  //if (strncmp(oBotName,"Lobo Malo",9) == 0)
	  //  GfOut("+");
	}
  }
  else if (fabs(oLanePoint.Crv) > 0.030)
  {
    if (Accel > oLastAccel * factor2)
	{
      Accel = MIN(Accel,MAX(0.1,oLastAccel*factor2));
	  //if (strncmp(oBotName,"Lobo Malo",9) == 0)
	  //  GfOut("-");
	}
  }
/**/
  return Accel;
}
//==========================================================================*

//==========================================================================*
// Filter Let Pass
//--------------------------------------------------------------------------*
double TDriver::FilterLetPass(double Accel)
{
  // If we should let another bot pass, decrease accelleration
  if (oLetPass) 
  {
    if (oTreatTeamMateAsLapper)
      Accel = MIN(Accel, 0.5);
  }
  return Accel;
}
//==========================================================================*

//==========================================================================*
// Filter Drifting
//--------------------------------------------------------------------------*
double TDriver::FilterDrifting(double Accel)
{
  // Decrease accelleration while drifting
  if (CarSeg->surface->kFriction < 1.0)
  {
    if((CarSpeedLong > oSlowSpeed) && (fabs(oDriftAngle) > 0.01))
//    if(fabs(oDriftAngle) > 0.01)
    {
	  Accel *= (float)
	    (0.10 + 0.90 * MAX(0.0,cos(oDriftAngle)*cos(oDriftAngle)*cos(oDriftAngle)));
    }
  }
  else
  {
    if((CarSpeedLong > oSlowSpeed) && (fabs(oDriftAngle) > 0.2))
//    if(fabs(oDriftAngle) > 0.2)
    {
	  Accel *= (float)
	  (0.25 + 5 * MAX(0.0,cos(oDriftAngle)*cos(oDriftAngle)));
    }
  }
  return Accel;
}
//==========================================================================*

//==========================================================================*
// Filter Track
//--------------------------------------------------------------------------*
double TDriver::FilterTrack(double Accel)
{
  if (DistanceRaced > oStartDistance)            // Except while starting
  {
	if (fabs(oDeltaOffset) > oTolerance)         // Check offset difference
	  Accel *= (float)                           //   Decrease acceleration
	    (MAX(1.0 - (fabs(oDeltaOffset) - oTolerance) * 0.2, 0.4));
  }
  return Accel;
}
//==========================================================================*

//==========================================================================*
// Detect obstacles
//--------------------------------------------------------------------------*
bool TDriver::IsStuck()
{
  TV2D Tmp;                                      // Holds Coordinates
  float Diff;                                    // Distance from old point
  
  if (DistanceRaced < 10)
    return false;

  if (oOppsNearInFront < oLastNearInFront)
  {
    oLastNearInFront = oOppsNearInFront;
	return false;
  }
  oLastNearInFront = oOppsNearInFront;

  if ((oStuckCounter > 3) && (oStuckCounter < 26))// Less then six ticks
	oCar->_brakeCmd = 1.0;                       //   left? stop driving back
  else
	oCar->_brakeCmd = 0.0;                       //

  if (oStuckCounter > 0)                         // Driving back?
  {                                              //   If so, clear buffer
    oSysFooStuckX->Reset();
    oSysFooStuckY->Reset();
    oStuckCounter--;                             //   decrement counter
    //GfOut("Driving back! %d\n",oStuckCounter);
    return true;                                 //   and drive
  }

  TV2D MyPos;                                    // Current position
  MyPos.x = CarPubGlobPosX;                      //
  MyPos.y = CarPubGlobPosY;                      //

  // Check motion
  Tmp.x = oSysFooStuckX->Faltung(float(MyPos.x));// X coordinate
  Tmp.y = oSysFooStuckY->Faltung(float(MyPos.y));// Y coordinate

  Diff = Dist(Tmp,MyPos);                        // Distance from old pos
  //GfOut("Dif: %g\n",Diff);
  if (Diff < 0.2)                                // If distance is to small
//  if (Diff < 0.5)                                // If distance is to small
  {                                              //   assume obstacle
	if (oStuckCounter == 0)
	{
      oStuckCounter = -UNSTUCK_COUNTER;          // Set counter
	  //GfOut("Set! %d\n",oStuckCounter);
	}

	if (oStanding)                               // But if flag is set
	{                                            //   it is planned!
	  //GfOut("Standing! %d\n",oStuckCounter);
	  oSysFooStuckX->Reset();                    // Clear buffers
	  oSysFooStuckY->Reset();                    //   of motion survey
      return false;                              //   and signal ok
	}
    else if (oUnstucking)                        // But if flag is set
	{                                            //   it is possible
      if (oStuckCounter < 0)                     //
	  {
        oSysFooStuckX->Reset();
        oSysFooStuckY->Reset();
        oStuckCounter++;                         // Increment counter
		if (oStuckCounter == 0)
	    {
          oStuckCounter = UNSTUCK_COUNTER;       // Set counter
  	      //GfOut("Stuck1! %d\n",oStuckCounter);
          return true;                           // give signal stuck
	    }
	    //GfOut("Unstucking! %d\n",oStuckCounter);
        return false;                            //   and signal ok
	  }
	  else                                       // still stuck
	  {
        if (oCurrSpeed > 1.0)
  	      return false;

        oStuckCounter = UNSTUCK_COUNTER;         // Set counter
  	    //GfOut("Stuck1! %d\n",oStuckCounter);
        return true;                             // give signal stuck
	  }
	}
    else                                         // if not
	{
      if (oCurrSpeed > 1.0)
	    return false;

      oStuckCounter = UNSTUCK_COUNTER;           // Set counter
	  //GfOut("Stuck! %d\n",oStuckCounter);
      return true;                               // give signal stuck
	}
  }
  else
    oStanding = false;                           // Reset flag

  return false;                                  // No obstacle
}
//==========================================================================*
/**/
//==========================================================================*
// Get target point 
//--------------------------------------------------------------------------*
TVec2d TDriver::GetTargetPoint(float LookAhead)
{
  double AheadPos = CarDistFromStart + LookAhead;
  return oRacingLine[RL_FREE].GetLaneTarget(AheadPos);
}
//==========================================================================*

//==========================================================================*
// Get avoid target point 
//--------------------------------------------------------------------------*
TVec2d TDriver::GetAvoidTargetPoint(double AvoidSide, float LookAhead)
{
  double AheadPos = CarDistFromStart + LookAhead;
  return oRacingLine[RL_FREE].GetAvoidLaneTarget(AvoidSide,AheadPos);
}
//==========================================================================*
/**/
//==========================================================================*
// Unstuck
//--------------------------------------------------------------------------*
void TDriver::Unstuck()
{
  TVec2d oTarget = GetTargetPoint(20.0);         // Target to steer to      

  // Direction to steer to target
  oSteerAngle = (float) atan2(oTarget.y - CarPosY, oTarget.x - CarPosX);
  oSteerAngle -= (float) CarYaw;				 // Car's rotation (z-axis) 
  FLOAT_NORM_PI_PI(oSteerAngle);				 // Normalize to -PI,+PI

  oAngle = CarSteerCmd = 
	(float) -(oSteerAngle/CarSteerLock);	     // Counter steering

  CarGearCmd = -1;                               // Reverse gear

  float Angle = oTrackAngle - CarYaw;
  FLOAT_NORM_PI_PI(Angle);
  if (fabs(Angle) < PI/2)
  {
    if (oOppsNearBehind < 1)
    {
      CarBrakeCmd = 0.0;                         // Unlock brake
      CarAccelCmd = 1.0;                         // Open the throttle
    }
    else
    {
      CarBrakeCmd = 1.0;                         // Lock brake
      CarAccelCmd = 0.0;                         // Wait
    }
  }
  else
  {
    CarBrakeCmd = 0.0;                           // Unlock brake
    CarAccelCmd = 0.5;                           // Open the throttle
  }

  CarClutchCmd = 0.0;                            // Release clutch

  oUnstucking = true;                            // Set flag
  oUseFilterDiff = true;                         // Set flag
}
//==========================================================================*

//==========================================================================*
// Protokoll to file
//--------------------------------------------------------------------------*
bool TDriver::SaveToFile()
{
  FILE* F = 0;
  char* PathFilename = PathStatFilenameBuffer;

  snprintf(PathLocalRobotDir, BUFLEN, "%sdrivers/%s", 
	GetLocalDir(),MyBotName);

  if (GfCreateDir(PathLocalRobotDir) == GF_DIR_CREATED)
  {
    snprintf(PathLocalRobotDir, BUFLEN, "%sdrivers/%s/%s_%d.txt", 
	  GetLocalDir(),MyBotName,MyBotName,oCar->_driverIndex);
    PathFilename = PathLocalRobotDir;
  }
  else
  {
    snprintf(PathStatFilenameBuffer,BUFLEN,        // Build path to
      "%s/%s_%d.txt"                               // own robot dir and
	  ,ROBOT_DIR,MyBotName,oCar->_driverIndex);    // name of own robot
	PathFilename = PathStatFilenameBuffer;
  }

//  GfOut("PathFilename: %s\n",PathFilename);

  F = fopen(PathFilename, "w");

  if (F == 0)
    return false;

  fprintf(F, "%s: %7.2f km/h ( %7.2f m/s / %d laps / %g m / %15.2f s)\n",
	  oCar->_name,oCar->_distRaced/CurrSimTime/1000*3600,oCar->_distRaced/CurrSimTime,oCar->_laps,oCar->_distRaced,CurrSimTime);
  fprintf(F, "Fuel consumtion: %.2f kg/100km (Fuel remaining: %.2f kg / Fuel filled in: %.2f kg / Fuel consumed: %.2f kg)\n",
	  (oFuelNeeded - oCar->_fuel)/oCar->_distRaced*100000, oCar->_fuel, oFuelNeeded,oFuelNeeded - oCar->_fuel);
  fprintf(F, "Dammages: %.0f (%.0f per lap / Repair: %.0f / Dammage remaining: %d)\n ",
	  oCar->_dammage + oRepairNeeded, (oCar->_dammage + oRepairNeeded)/oCar->_laps, oRepairNeeded, oCar->_dammage);

  fprintf(F, "\nBrakeCoeffs\n");
  double Sum = 0.0;
  for (int I = 0; I < NBR_BRAKECOEFF; I++)
  {
    fprintf(F, "%f\n",oBrakeCoeff[I]);
    Sum += oBrakeCoeff[I];
  }
  Sum /= NBR_BRAKECOEFF;
  fprintf(F, "Mean: %f\n",Sum);

  fclose(F);
/*
  GfOut("\n\n\n TDriver::SaveToFile() ... \n\n\n");

  snprintf(PathLocalRobotDir, BUFLEN, "%sdrivers/%s", 
	GetLocalDir(),MyBotName);

  if (GfCreateDir(PathLocalRobotDir) == GF_DIR_CREATED)
  {
	snprintf(PathLocalRobotDir, BUFLEN, "%sdrivers/%s/%s_%d.brk", 
	  GetLocalDir(),MyBotName,MyBotName,oCar->_driverIndex);
    PathFilename = PathLocalRobotDir;
  }
  else
  {
    snprintf(PathStatFilenameBuffer,BUFLEN,        // Build path to
	  "%s/%s.brk"                                  // own robot dir and
	  ,ROBOT_DIR,MyBotName);                       // name of own robot
	PathFilename = PathStatFilenameBuffer;
  }

  // Save brake coeffs
  GfOut("PathFilename: %s\n",PathFilename);

  F = fopen(PathFilename, "wb");
  if (F == 0)
    return false;

  int N = NBR_BRAKECOEFF;
  fwrite(&N,sizeof(int),1,F);

  for (int I = 0; I < NBR_BRAKECOEFF; I++)
    fwrite(&(oBrakeCoeff[I]),sizeof(double),1,F);
  fclose(F);

  GfOut("\n\n\n ... TDriver::SaveToFile()\n\n\n");
*/
  return true;
}
//==========================================================================*

//==========================================================================*
// Save brake coeffs
//--------------------------------------------------------------------------*
bool TDriver::SaveBrakeCoeffsToFile()
{
  FILE* F = 0;

  char* PathFilename = PathStatFilenameBuffer;
  snprintf(PathStatFilenameBuffer,BUFLEN,        // Build path to
    "%s/%s.brk"                                  // own robot dir and
	,ROBOT_DIR,MyBotName);                       // name of own robot

  F = fopen(PathFilename, "wb");
  if (F == 0)
    return false;

  int N = NBR_BRAKECOEFF;
  fwrite(&N,sizeof(int),1,F);

  for (int I = 0; I < NBR_BRAKECOEFF; I++)
    fwrite(&(oBrakeCoeff[I]),sizeof(double),1,F);
  fclose(F);

  return true;
}
//==========================================================================*

//==========================================================================*
// Load brake coeffs
//--------------------------------------------------------------------------*
bool TDriver::LoadBrakeCoeffsFromFile()
{
  FILE* F = 0;

  char* PathFilename = PathStatFilenameBuffer;
  snprintf(PathStatFilenameBuffer,BUFLEN,        // Build path to
    "%s/%s.brk"                                  // own robot dir and
	,ROBOT_DIR,MyBotName);                       // name of own robot

  F = fopen(PathFilename, "rb");
  if (F == 0)
    return false;

  int N;
  fread(&N,sizeof(int),1,F);

  N = MIN(N,NBR_BRAKECOEFF);

  for (int I = 0; I < N; I++)
    fread(&(oBrakeCoeff[I]),sizeof(double),1,F);
  fclose(F);

  return true;
}
//==========================================================================*

//==========================================================================*
// Drive slower
//--------------------------------------------------------------------------*
void TDriver::DriveSlower(double Scale, int Length)
{
	if (oLoadedLearned)
		return;

	if ((oExcludeFrom < oSecIndex) && (oExcludeTill > oSecIndex))
		return;

	int N = oRacingLine[oRL_FREE].oCount;

	double Den = MAX(1.0,oLapsLearned);
//	Scale *= 1.0/Den;
	Scale *= (50.0 - Den)/100.0;
	Scale = MAX(0.01,MIN(10.0,Scale));
	Scale *= oCorrFactor;

	int K = (oSecIndex + N - Length) % N;
	// We have to drive slower
	for (int I = 0; I < Length; I++)
	{
		K = (K + 1) % N;
		oRacingLine[oRL_FREE].oPathPoints[K].SpeedFrictionFactor *= 
			(float) ((100 - 0.2 * Scale)/100.0);
		oRacingLine[oRL_FREE].oPathPoints[K].SpeedFrictionFactor = 
			(float) MAX(oMinSpeedFrictionFactor,MIN(oMaxSpeedFrictionFactor
			,oRacingLine[oRL_FREE].oPathPoints[K].SpeedFrictionFactor));
	}
	// GfOut("%4.4d -S: %.2f(%.4f)\n",oLastSecIndex,oDifference,PPP->SpeedFrictionFactor);
}
//==========================================================================*

//==========================================================================*
// Drive faster
//--------------------------------------------------------------------------*
void TDriver::DriveFaster(double Scale)
{
	if (oLoadedLearned)
		return;

	if ((oExcludeFrom < oSecIndex) && (oExcludeTill > oSecIndex))
		return;
	
	int N = oRacingLine[oRL_FREE].oCount;

	double Den = MAX(1.0,oLapsLearned);
//	Scale *= 1.0/Den;
	Scale *= (50.0 - Den)/100.0;
	Scale = MAX(0.01,MIN(10.0,Scale));
	Scale *= oCorrFactor;

	int K = (oSecIndex + N - 20) % N;
	for (int I = 0; I < 20; I++)
	{
		K = (K + 1) % N;
		oRacingLine[oRL_FREE].oPathPoints[K].SpeedFrictionFactor *= 
			(float) ((100 + 0.1 * Scale)/100.0);
		oRacingLine[oRL_FREE].oPathPoints[K].SpeedFrictionFactor = 
			(float) MAX(oMinSpeedFrictionFactor,MIN(oMaxSpeedFrictionFactor
			,oRacingLine[oRL_FREE].oPathPoints[K].SpeedFrictionFactor));
	}
	//GfOut("%4.4d +S: %.2f(%.4f)\n",oLastSecIndex,oDifference,PPP->SpeedFrictionFactor);
}
//==========================================================================*

//==========================================================================*
// Check Tolerance
//--------------------------------------------------------------------------*
void TDriver::CheckTolerance()
{
	double Crv = oRacingLine[oRL_FREE].oPathPoints[oSecIndex].Crv;
	double Offset = oRacingLine[oRL_FREE].oPathPoints[oSecIndex].Offset;

	TVec2d Target = GetTargetPoint(0.0);
	TVec2d Middle(oRacingLine[oRL_FREE].oPathPoints[oSecIndex].Center.GetXY()); 

	oDifference = 
		SIGN(Crv)*(SIGN(Offset)*(Middle-Target).len() + CarToMiddle);

	if (fabs(Crv) < 0.001)
		oDifference = 0.0;

	double Dist = fabs(CarToMiddle);
	if (fabs(oDriftAngle) > PI/8)
	{
//		DriveSlower(2.0);
		DriveSlower(1.5);
	}
//	else if (Dist > (oTrack->width - CarWidth)/2 + 1.0)
	else if (Dist > (oTrack->width - CarWidth)/2)
	{
		//DriveSlower(-oDifference);
		DriveSlower(-0.75*oDifference);
	}
	else if (oDifference < -2.5)
	{
		//DriveSlower(-0.5*oDifference);
		DriveSlower(-0.35*oDifference);
	}
	else if (oDifference > -1.0)
	{
		//DriveFaster(MAX(2.0,fabs(oDifference)));
		DriveFaster(MAX(0.5,fabs(oDifference)));
	}
}
//==========================================================================*

//==========================================================================*
// Start braking earlier
//--------------------------------------------------------------------------*
void TDriver::BrakeEarlier(double Scale)
{
	if (oLoadedLearned)
		return;

	if ((oExcludeFrom < oSecIndex) && (oExcludeTill > oSecIndex))
		return;

	double Den = MAX(1.0,oLapsLearned);
//	Scale *= 1.0/Den;
	Scale *= (50.0 - Den)/100.0;
	Scale = MAX(0.01,MIN(30.0,Scale));
	Scale *= oCorrFactor;

	int N = oRacingLine[oRL_FREE].oCount;
//	int K = (oSecIndex + N - 15) % N;
	int K = (oSecIndex + N - 8) % N;
	
//	for (int I = 0; I < 15; I++)
	for (int I = 0; I < 8; I++)
	{
		K = (K + 1) % N;
		oRacingLine[oRL_FREE].oPathPoints[K].BrakeFrictionFactor *= 
			(float) ((100 - Scale)/100.0);
		oRacingLine[oRL_FREE].oPathPoints[K].BrakeFrictionFactor = 
			(float) MAX(oMinBrakeFrictionFactor,MIN(oMaxBrakeFrictionFactor
			,oRacingLine[oRL_FREE].oPathPoints[K].BrakeFrictionFactor));
	}
}
//==========================================================================*

//==========================================================================*
// Start braking later
//--------------------------------------------------------------------------*
void TDriver::BrakeLater(double Scale)
{
	if (oLoadedLearned)
		return;

	if ((oExcludeFrom < oSecIndex) && (oExcludeTill > oSecIndex))
		return;

	double Den = MAX(1.0,oLapsLearned);
//	Scale *= 1.0/Den;
	Scale *= (50.0 - Den)/100.0;
	Scale = MAX(0.01,MIN(30.0,Scale));
	Scale *= oCorrFactor;

	int N = oRacingLine[oRL_FREE].oCount;
//	int K = (oSecIndex + N - 15) % N;
	int K = (oSecIndex + N - 8) % N;
	
//	for (int I = 0; I < 15; I++)
	for (int I = 0; I < 8; I++)
	{
		K = (K + 1) % N;
		oRacingLine[oRL_FREE].oPathPoints[K].BrakeFrictionFactor *= 
			(float) ((100 + Scale)/100.0);
		oRacingLine[oRL_FREE].oPathPoints[K].BrakeFrictionFactor = 
			(float) MAX(oMinBrakeFrictionFactor,MIN(oMaxBrakeFrictionFactor
			,oRacingLine[oRL_FREE].oPathPoints[K].BrakeFrictionFactor));
	}
}
//==========================================================================*

//==========================================================================*
// Check speed, control brake press
//--------------------------------------------------------------------------*
void TDriver::SpeedController()
{
	double Diff = oCurrSpeed - oLastTargetSpeed - 2.0;
	if (oLastBrake > 0)
	{
		if (Diff > 2.0)
		{
			BrakeEarlier(2*Diff);
		}
//		else if (Diff < -1.0)
		else if (Diff < 0.5)
		{
			// We can brake later
			BrakeLater(-0.5*MAX(0.5,Diff));
		}
	}

	Diff = oCurrSpeed - oTargetSpeed;
	if (Diff > 0.0)
	{
		oAccel = 0;
		oBrake = (float) MIN(oBrakeForceMax,Diff * Diff / oBrakeScale);
	}
}
//==========================================================================*

//==========================================================================*
// Update target speed
//--------------------------------------------------------------------------*
void TDriver::UpdateTargetSpeed()
{
	if (!oLoadedLearned)
	{
		oRacingLine[oRL_FREE].LapsLearned = ++oLapsLearned;

//		if ((oCar->race.deltaBestLapTime < 0) || (CarLaps < 5))
		if ((oCar->race.deltaBestLapTime < 0) || (CarLaps < 1))
		{
			oBestLapTime = oCar->race.bestLapTime;
			oRacingLine[oRL_FREE].StoreLearned();
			GfOut("# StoreLearned (Laps: %d)\n",oLapsLearned);
			oCorrFactor = 1.0;
		}
		else
		{
			oRacingLine[oRL_FREE].RestoreLearned();
			oCorrFactor *= 1.05;

		}
	}

	Param.Update();

	for (int I = 0; I < NBRRL; I++)
	{
      oRacingLine[I].CalcMaxSpeeds(1);
	  oRacingLine[I].PropagateBreaking(1);
      oRacingLine[I].PropagateAcceleration(1);
	}

	oLastLap++; 
}
//==========================================================================*

//--------------------------------------------------------------------------*
// end of file unitdriver.cpp
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
