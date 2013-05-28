/***************************************************************************

    file                 : dandroid_2012.cpp
    created              : 2006-06-11 17:14:57 UTC 2006
    copyright            : (C) Daniel Schellhammer

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "driver.h"

#define NBBOTS 2

static char* botname[NBBOTS] = {(char*)"dandroid_2012 1", (char*)"dandroid_2012 2"};
static char* botdesc[NBBOTS] = {(char*)"dandroid_2012 1", (char*)"dandroid_2012 2"};

static TDriver *driver[NBBOTS];

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static int pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);


// Module entry point.
extern "C" int dandroid_2012(tModInfo *modInfo)
{
  int i;

  // Clear all structures.
  memset(modInfo, 0, 10*sizeof(tModInfo));

  for (i = 0; i < NBBOTS; i++) {
    modInfo[i].name    = strdup(botname[i]);  // name of the module (short).
    modInfo[i].desc    = strdup(botdesc[i]);  // Description of the module (can be long).
    modInfo[i].fctInit = InitFuncPt;  // Init function.
    modInfo[i].gfId    = ROB_IDENT;   // Supported framework version.
    modInfo[i].index   = i;           // Indices from 0 to 9.
  }
  return 0;
}


// Module interface initialization.
static int InitFuncPt(int index, void *pt)
{
  tRobotItf *itf = (tRobotItf *)pt;

  // Create robot instance for index.
  driver[index] = new TDriver(index);
  itf->rbNewTrack = initTrack;  // Give the robot the track view called.
  itf->rbNewRace  = newRace;    // Start a new race.
  itf->rbDrive    = drive;      // Drive during race.
  itf->rbPitCmd   = pitcmd;     // Pit commands.
  itf->rbEndRace  = endRace;    // End of the current race.
  itf->rbShutdown = shutdown;   // Called before the module is unloaded.
  itf->index      = index;      // Index used if multiple interfaces.
  return 0;
}


// Called for every track change or new race.
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
  driver[index]->InitTrack(track, carHandle, carParmHandle, s);
}


// Start a new race.
static void newRace(int index, tCarElt* car, tSituation *s)
{
  driver[index]->NewRace(car, s);
}


// Drive during race.
static void drive(int index, tCarElt* car, tSituation *s)
{
  driver[index]->Drive();
}


// Pitstop callback.
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
  return driver[index]->PitCmd();
}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s)
{
  driver[index]->EndRace();
}


// Called before the module is unloaded.
static void shutdown(int index)
{
  driver[index]->Shutdown();
  delete driver[index];
}

