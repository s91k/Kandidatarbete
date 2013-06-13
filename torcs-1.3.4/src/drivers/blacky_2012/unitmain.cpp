//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitmain.cpp
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter für TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Interface zu TORCS
//
// Datei    : unitmain.cpp
// Erstellt : 14.12.2008
// Stand    : 10.06.2012
// Copyright: © 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.04.000 (Championship 2012 Alpine-1)
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
#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "unitglobal.h"
#include "unitcommon.h"

#include "unitdriver.h"

//==========================================================================*
// Consts, types and variables, visible outside of this unit
// Konstanten, Typen und Variablen, die außerhalb dieser Unit sichtbar sind
//--------------------------------------------------------------------------*
// Default names of the drivers/roboters
// Namen und Beschreibung der Roboter
static const char* Default_BotName[MAX_NBBOTS] = {
  "car1-trb1_0",
  "car1-trb1_1",
  "car2-trb1_0",
  "car2-trb1_1",
  "car3-trb1_0",
  "car3-trb1_1",
  "car4-trb1_0",
  "car4-trb1_1",
  "car5-trb1_0",
  "car5-trb1_1"
};
static const char* Default_BotDesc[MAX_NBBOTS] = {
  "car1-trb1_0",
  "car1-trb1_1",
  "car2-trb1_0",
  "car2-trb1_1",
  "car3-trb1_0",
  "car3-trb1_1",
  "car4-trb1_0",
  "car4-trb1_1",
  "car5-trb1_0",
  "car5-trb1_1"
};

//  Robot of this modul
//  Roboter des Moduls
static const char** BotName = Default_BotName;
static const char** BotDesc = Default_BotDesc;

static TDriver *cRobot[MAX_NBBOTS];
static TCommonData gCommonData;

// Time analisys ...
static double cTicks;
static double cMinTicks;
static double cMaxTicks;
static int cTickCount;
static int cLongSteps;
static int cCriticalSteps;
static int cUnusedCount;
// ... Time analisys

// For use of Rt-Timer functions ...
#if defined(WIN32) || defined(_WIN32)
static bool cUsePerformanceCounter;				// Hardware exists?
#else
#include "sys/time.h"
#endif
double RtTicksPerSec = 1000.0;					// Ticks per second
// ... for use of Rt-Timer functions
//==========================================================================*

//==========================================================================*
// Buffers
// Puffer
//--------------------------------------------------------------------------*
#define BIGBUFLEN 256
static char FilenameBuffer[BIGBUFLEN];           // for path and filename
#define BUFLEN 20
static char Drivers[BUFLEN * MAX_NBBOTS];        // Driver names
static bool Footprint = false;                   // Never called yet
static int InitDriver = -1;                      // None initialized yet
//==========================================================================*

//==========================================================================*
// Prototypes of routines(functions/procedures), provided
// for communication with TORCS
// Prototypen der Routinen(Funktionen/Prozeduren), die wir für die
// Kommunikation mit TORCS bereitstellen
//--------------------------------------------------------------------------*
static void InitTrack
  (int index,
  tTrack* track,
  void *carHandle,
  void **carParmHandle,
  tSituation *s);
static void NewRace
  (int index,
  tCarElt* car,
  tSituation *s);
static void Drive
  (int index,
  tCarElt* car,
  tSituation *s);
static int PitCmd
  (int index,
  tCarElt* car,
  tSituation *s);
static void Shutdown
  (int index);
static int InitFuncPt
  (int index,
  void *pt);
static void EndRace
  (int index,
  tCarElt *car,
  tSituation *s);
static void Shutdown
  (int index);
//==========================================================================*

//==========================================================================*
// Init Timer
//--------------------------------------------------------------------------*
void RtInitTimer()
{
#if defined(WIN32) || defined(_WIN32)
  ULONGLONG TicksPerSec;
  if (!QueryPerformanceFrequency((LARGE_INTEGER*)&TicksPerSec)) 
  {
	  GfOut("\n\n#\n# Performance Counter nicht vorhanden\n#\n\n"); 
	  cUsePerformanceCounter = false;
  }
  else
  {
	  RtTicksPerSec = (double) TicksPerSec;
	  GfOut("\n\n#\n# Frequency for Performance Counter: %g GHz (= 1 / %u)\n",(1000000000.0/RtTicksPerSec),TicksPerSec); 
	  GfOut("# Resolution for Performance Counter: %g nanosec\n#\n\n",TicksPerSec/1000000000.0); 
	  cUsePerformanceCounter = true;
  }
#endif
}
//==========================================================================*

//==========================================================================*
// Get timer frequency [HZ]
//--------------------------------------------------------------------------*
double RtTimerFrequency()
{
	return 1.0 / RtTicksPerSec;
}
//==========================================================================*

//==========================================================================*
// Get timestamp [msec]
//--------------------------------------------------------------------------*
double RtTimeStamp()
{
#if defined(WIN32) || defined(_WIN32)
	if (cUsePerformanceCounter)
	{
		static ULONGLONG TickCount; 
//		DWORD_PTR oldmask = ::SetThreadAffinityMask(::GetCurrentThread(), 0);
		QueryPerformanceCounter((LARGE_INTEGER*)&TickCount); 
//		::SetThreadAffinityMask(::GetCurrentThread(), oldmask);
		return (1000.0 * TickCount)/RtTicksPerSec;
	}
	else
	{
  	  clock_t StartTicks = clock();
	  return (1000.0 * StartTicks)/CLOCKS_PER_SEC;
	}
#else
	struct timeval tv;
	gettimeofday(&tv, NULL); 
	return tv.tv_usec/1000.0;
#endif
}
//==========================================================================*

//==========================================================================*
// Get Duration [msec]
//--------------------------------------------------------------------------*
double RtDuration(double StartTimeStamp)
{
	double Duration = RtTimeStamp() - StartTimeStamp;

#if defined(WIN32) || defined(_WIN32)
#else
	if (Duration < 0.0)							// For Linux add milli sec
		Duration += 1000.0;						//   if microsec is < 0!
#endif

	return Duration;
}
//==========================================================================*

//==========================================================================*
// Prepare names
// Aufbereitung der Namen
//--------------------------------------------------------------------------*
bool Prepare()
{
  InitDriver++;       // Count initialized drivers
  if (Footprint)      // Check wether we have done it before
    return false;     //   If so, return false,
  else
    Footprint = true; // else set flag

  // Initialize the base param path.
  //char* BaseParamPath = TDriver::ROBOT_DIR;      // Depends on robots name
  char* PathFilename = FilenameBuffer;           // Pointer to buffer

  memset(&Drivers[0],0,BUFLEN * TDriver::NBBOTS);// Clear buffer

  snprintf(FilenameBuffer,BIGBUFLEN,             // Build path to
    "drivers/%s/%s.xml"                          // own robot from
	,TDriver::MyBotName,TDriver::MyBotName);     // name of robot

  void* RobotSettings = GfParmReadFile           // Open team setup file
    (PathFilename,GFPARM_RMODE_STD);

  if (RobotSettings)                             // If file opened
  {
    char SectionBuffer[256];                     // Buffer for section name
    char* Section = SectionBuffer;               // Adjust Pointer

    for (int I = 0; I < TDriver::NBBOTS; I++)    // Loop all drivers
    {
      snprintf(SectionBuffer,BUFLEN,             // Build name of
        "%s/%s/%d"                               // section from
	    ,ROB_SECT_ROBOTS,ROB_LIST_INDEX,I);      // Index of driver

	  const char* DriverName = GfParmGetStr      // Get pointer to
        (RobotSettings                           // drivers name
        , Section                                // defined in corresponding
        , (char *) ROB_ATTR_NAME                 // section,
        , (char *) BotName[I]);                  // BotName[I] as default

	  snprintf(&Drivers[I*BUFLEN],BUFLEN-1,DriverName);
    }
  }
  else
  { // This should never happen! But give user a chance to read it!
	GfOut("\n\n\n FATAL ERROR: File '%s' not found\n\n",PathFilename);
	for (int I = 0; I < TDriver::NBBOTS; I++)
      snprintf(&Drivers[I*BUFLEN],BUFLEN-1,BotName[I]);
  }
  
  RtInitTimer(); // Check existance of Performance Counter Hardware

  return Footprint;
};
//==========================================================================*

//==========================================================================*
// Get name of driver/robot
// Namen des Fahrers/Roboters holen
//--------------------------------------------------------------------------*
char* GetBotName(int Index)
{
  return &Drivers[Index*BUFLEN];
};
//==========================================================================*

//==========================================================================*
// Tells TORCS, who we are, how we want to be called and
// what we are able to do.
// Teilt TORCS mit, wer wir sind, wie wir angesprochen werden wollen und
// was wir können.
//--------------------------------------------------------------------------*
extern "C" int wdbee(tModInfo *ModInfo)
{
  if (Prepare()) // Check Footprint and prepare names
  { // Run once only: Clear memory provided
    memset(ModInfo, 0, 10 * sizeof(tModInfo));
  }

  int I;
  for (I = 0; I < TDriver::NBBOTS; I++)
  {
    ModInfo[I].name    =  strdup(GetBotName(I));          // Tell customisable name
    ModInfo[I].desc    =  strdup((char *) BotDesc[I]);    // Tell customisable desc.
    ModInfo[I].fctInit = InitFuncPt;             // Common used functions
    ModInfo[I].gfId    = ROB_IDENT;              // Robot identity
    ModInfo[I].index   = I;                      // Drivers index
  }
  return 0;
}
//==========================================================================*

//==========================================================================*
// TORCS: Initialization
// TOCRS: Initialisierung
//
// After clarification of the general calling (calling this func.),
// we tell TORCS our functions to provide the requested services:
//
// Nach Klärung der generellen Ansprache (Aufruf dieser Fkt), teilen wir
// TORCS nun noch mit, mit welchen Funktionen wir die angeforderten
// Leistungen erbringen werden:
//
// Die geforderten Leistungen müssen erbracht werden ...
// RbNewTrack: ... wenn Torcs eine neue Rennstrecke bereitstellt
// RbNewRace:  ... wenn Torcs ein neues Rennen startet
// RbDrive:    ... wenn das Rennen gefahren wird
// RbPitCmd:   ... wenn wir einen Boxenstop machen
// RbEndRace:  ... wenn das Rennen ist beendet
// RbShutDown: ... wenn der ggf. angefallene Schrott beseitigt werden muss
//--------------------------------------------------------------------------*
static int InitFuncPt(int Index, void *Pt)
{
  tRobotItf *Itf = (tRobotItf *)Pt;              // Get typed pointer

  Itf->rbNewTrack = InitTrack;                   // Store function pointers
  Itf->rbNewRace  = NewRace;
  Itf->rbDrive    = Drive;
  Itf->rbPitCmd   = PitCmd;
  Itf->rbEndRace  = EndRace;
  Itf->rbShutdown = Shutdown;
  Itf->index      = Index;                       // Store index

  cRobot[Index] = new TDriver(Index);            // Create a driver
  cRobot[Index]->SetBotName(GetBotName(Index));  // Store customized name

  return 0;
}
//==========================================================================*

//==========================================================================*
// TORCS: New track
// TOCRS: Neue Rennstrecke
//--------------------------------------------------------------------------*
static void InitTrack(int Index,
  tTrack* Track,void *CarHandle,void **CarParmHandle, tSituation *S)
{
  cRobot[Index]->SetCommonData(&gCommonData);    // Init common used data
  cRobot[Index]->InitTrack(Track,CarHandle,CarParmHandle, S);
}
//==========================================================================*

//==========================================================================*
// TORCS: New Race starts
// TOCRS: Neues Rennen beginnt
//--------------------------------------------------------------------------*
static void NewRace(int Index, tCarElt* Car, tSituation *S)
{
  cTicks = 0.0;                                  // Initialize counters
  cMinTicks = DBL_MAX;                           // and time data
  cMaxTicks = 0.0;
  cTickCount = 0;
  cLongSteps = 0;
  cCriticalSteps = 0;
  cUnusedCount = 0;

  cRobot[Index]->NewRace(Car, S);
}
//==========================================================================*

//==========================================================================*
// TORCS-Callback: Drive
// TOCRS-Callback: Rennen fahren
//
// Attention: This procedure is called very frequent and fast in succession!
// Therefore we don't throw debug messages here!
// To find basic bugs, it may be usefull to do it anyhow!

// Achtung: Diese Prozedur wird sehr häufig und schnell nacheinander
// aufgerufen. Deshalb geben wir hier in der Regel keine Debug-Texte aus!
// Zur Fehlersuche kann das aber mal sinnvoll sein.
//--------------------------------------------------------------------------*
static void Drive(int Index, tCarElt* Car, tSituation *S)
{
  //GfOut(">>> TDriver::Drive\n");
  if (cRobot[Index]->CurrSimTime != S->currentTime)
  {
    double StartTimeStamp = RtTimeStamp();

    cRobot[Index]->CurrSimTime = S->currentTime; // Update current time
    cRobot[Index]->Update(Car,S);                // Update info about opp.
    cRobot[Index]->Drive();                      //   Drive

    double Duration = RtDuration(StartTimeStamp); 

	if (cTickCount > 0)                          // Collect used time
	{
	  if (Duration > 1.0)
        cLongSteps++;
	  if (Duration > 2.0)
        cCriticalSteps++;
	  if (cMinTicks > Duration)
	    cMinTicks = Duration;
	  if (cMaxTicks < Duration)
	    cMaxTicks = Duration;
	}
	cTickCount++;
  	cTicks += Duration;
  }
  else
    cUnusedCount++;
  //GfOut("<<< TDriver::Drive\n");
}
//==========================================================================*

//==========================================================================*
// TORCS: Pitstop (Car is in pit!)
// TOCRS: Boxenstop (Wagen steht in der Box!)
//--------------------------------------------------------------------------*
static int PitCmd(int Index, tCarElt* Car, tSituation *S)
{
  // Dummy: use parameters
  if ((Index < 0) || (Car == NULL) || (S == NULL))
    printf("PitCmd\n");
  return cRobot[Index]->PitCmd();
}
//==========================================================================*

//==========================================================================*
// TORCS: Race ended
// TOCRS: Rennen ist beendet
//--------------------------------------------------------------------------*
static void EndRace(int Index, tCarElt *Car, tSituation *S)
{
  // Dummy: use parameters
  if ((Index < 0) || (Car == NULL) || (S == NULL))
    printf("EndRace\n");
  cRobot[Index]->EndRace();
}
//==========================================================================*

//==========================================================================*
// TORCS: Cleanup
// TOCRS: Aufräumen
//--------------------------------------------------------------------------*
static void Shutdown(int Index)
{
  cRobot[Index]->Shutdown();
  delete cRobot[Index];

  GfOut("\n\nClock\n");
  GfOut("Total Time used: %g sec\n",cTicks/1000.0);
  GfOut("Min   Time used: %g msec\n",cMinTicks);
  GfOut("Max   Time used: %g msec\n",cMaxTicks);
  GfOut("Mean  Time used: %g msec\n",cTicks/cTickCount);
  GfOut("Long Time Steps: %d\n",cLongSteps);
  GfOut("Critical Steps : %d\n",cCriticalSteps);
  GfOut("Unused Steps   : %d\n",cUnusedCount);

  GfOut("\n\n");
}
//==========================================================================*

//==========================================================================*
// Schismatic entry point for cavallo_2012
//--------------------------------------------------------------------------*
extern "C" int cavallo_2012(tModInfo *ModInfo)
{
  TDriver::NBBOTS = 2;                                    // use 2 cars
  TDriver::MyBotName = (char *) "cavallo_2012";           // Name of this bot
  TDriver::ROBOT_DIR = (char *) "drivers/cavallo_2012";   // Sub path to dll
  TDriver::SECT_PRIV = (char *) "wdbee private";          // Private section
  TDriver::DEFAULTCARTYPE  = (char *) "car1-trb1";        // Default car type
  return wdbee(ModInfo);
};
//==========================================================================*

//==========================================================================*
// Schismatic entry point for boxer_2012
//--------------------------------------------------------------------------*
extern "C" int boxer_2012(tModInfo *ModInfo)
{
  TDriver::NBBOTS = 2;                                    // use 2 cars
  TDriver::MyBotName = (char *) "boxer_2012";             // Name of this bot
  TDriver::ROBOT_DIR = (char *) "drivers/boxer_2012";     // Sub path to dll
  TDriver::SECT_PRIV = (char *) "wdbee private";          // Private section
  TDriver::DEFAULTCARTYPE  = (char *) "car2-trb1";        // Default car type
  return wdbee(ModInfo);
};
//==========================================================================*

//==========================================================================*
// Schismatic entry point for jumper_2012
//--------------------------------------------------------------------------*
extern "C" int jumper_2012(tModInfo *ModInfo)
{
  TDriver::NBBOTS = 2;                                    // use 2 cars
  TDriver::MyBotName = (char *) "jumper_2012";            // Name of this bot
  TDriver::ROBOT_DIR = (char *) "drivers/jumper_2012";    // Sub path to dll
  TDriver::SECT_PRIV = (char *) "wdbee private";          // Private section
  TDriver::DEFAULTCARTYPE  = (char *) "car3-trb1";        // Default car type
  return wdbee(ModInfo);
};
//==========================================================================*

//==========================================================================*
// Schismatic entry point for blacky_2012
//--------------------------------------------------------------------------*
extern "C" int blacky_2012(tModInfo *ModInfo)
{
  TDriver::NBBOTS = 2;                                    // use 2 cars
  TDriver::MyBotName = (char *) "blacky_2012";            // Name of this bot
  TDriver::ROBOT_DIR = (char *) "drivers/blacky_2012";    // Sub path to dll
  TDriver::SECT_PRIV = (char *) "wdbee private";          // Private section
  TDriver::DEFAULTCARTYPE  = (char *) "car4-trb1";        // Default car type
  return wdbee(ModInfo);
};
//==========================================================================*

//==========================================================================*
// Schismatic entry point for trotter_2012
//--------------------------------------------------------------------------*
extern "C" int trotter_2012(tModInfo *ModInfo)
{
  TDriver::NBBOTS = 2;                                    // use 2 cars
  TDriver::MyBotName = (char *) "trotter_2012";           // Name of this bot
  TDriver::ROBOT_DIR = (char *) "drivers/trotter_2012";   // Sub path to dll
  TDriver::SECT_PRIV = (char *) "wdbee private";          // Private section
  TDriver::DEFAULTCARTYPE  = (char *) "car5-trb1";        // Default car type
  return wdbee(ModInfo);
};
//==========================================================================*

//==========================================================================*
// Schismatic entry point for wdbee_2012
//--------------------------------------------------------------------------*
extern "C" int wdbee_2012(tModInfo *ModInfo)
{
  TDriver::NBBOTS = 2;                                    // use 2 cars
  TDriver::MyBotName = (char *) "wdbee_2012";             // Name of this bot
  TDriver::ROBOT_DIR = (char *) "drivers/wdbee_2012";     // Sub path to dll
  TDriver::SECT_PRIV = (char *) "wdbee private";          // Private section
  TDriver::DEFAULTCARTYPE  = (char *) "car6-trb1";        // Default car type
  return wdbee(ModInfo);
};
//==========================================================================*

//==========================================================================*
// Schismatic entry point for hunter_2012
//--------------------------------------------------------------------------*
extern "C" int hunter_2012(tModInfo *ModInfo)
{
  TDriver::NBBOTS = 2;                                    // use 2 cars
  TDriver::MyBotName = (char *) "hunter_2012";            // Name of this bot
  TDriver::ROBOT_DIR = (char *) "drivers/hunter_2012";    // Sub path to dll
  TDriver::SECT_PRIV = (char *) "wdbee private";          // Private section
  TDriver::DEFAULTCARTYPE  = (char *) "car7-trb1";        // Default car type
  return wdbee(ModInfo);
};
//==========================================================================*

//==========================================================================*
// Schismatic entry point for ring_2012
//--------------------------------------------------------------------------*
extern "C" int ring_2012(tModInfo *ModInfo)
{
  TDriver::NBBOTS = 2;                                    // use 2 cars
  TDriver::MyBotName = (char *) "ring_2012";              // Name of this bot
  TDriver::ROBOT_DIR = (char *) "drivers/ring_2012";      // Sub path to dll
  TDriver::SECT_PRIV = (char *) "wdbee private";          // Private section
  TDriver::DEFAULTCARTYPE  = (char *) "car8-trb1";        // Default car type
  return wdbee(ModInfo);
};
//==========================================================================*

//--------------------------------------------------------------------------*
// end of file unitmain.cpp
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
