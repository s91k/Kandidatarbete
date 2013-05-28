/***************************************************************************

    file                 : danpath.h
    created              : 2011-11-14 07:39:00 UTC
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

#ifndef _DANPATH_H_
#define _DANPATH_H_

#include "torcs_or_sd.h"
#include "globaldefinitions.h"

#include <stdlib.h>
#include <math.h>
#include <vector>

#include "d_linalg.h"
#include "cubic.h"
#include "path.h"

#ifdef TARGET_SPEEDDREAMS
#include "path1.h"
#endif

class DanPoint {
  public:
  int  line;
  int  index;
  v2d  pos;
  int  type;
  double fromstart;
  double tomiddle;
  double radius;
  double yaw;
  double angletotrack;
};


class DanSector {
  public:
  int sector;
  double fromstart;
  double speedfactor;
  double time;
  double bestspeedfactor;
  double besttime;
  int learned;
};


class DanLine {
  public:
  DanLine(PTrack t);
  void addDanPoint(DanPoint danpoint);
  bool calcParam();
  void createSectors();
  bool getDanPos(double fromstart, DanPoint& danpoint);
  bool getLocalPos(PTrackSeg segment, DanPoint danpoint, tTrkLocPos* locpos);
  DanPoint nextPos(DanPoint danpoint);
  DanPoint prevPos(DanPoint danpoint);
  std::vector <DanSector> vDansect;
  private:
  // Data
  double MAX_RADIUS;
  PTrack mTrack;
  PTrackSeg myseg; // Needed in fromStart() because of problem with crossing tracks
  std::vector <DanPoint> vDanpath;
  // Functions
  DanPoint getPos(int index);
  double calcRadius(int index);
  double calcYaw(DanPoint danpoint);
  double calcTrackYaw(DanPoint danpoint, double& trackyaw);
  bool fromStart(v2d pos, double& fromstart);
  bool toMiddle(v2d pos, double& tomiddle);
  void printData();
  int getIndex(double fromstart);
  v2d getNearestPoint(int index, double fromstart);
  double getToMiddle(double fromstart);
  double getDistDiff(double fromstart1, double fromstart2);
};


class DanPath {
  public:
  DanPath(PTrack t, double dist_ext, double dist_int);
  ~DanPath();
  bool getDanPos(int line, double fromstart, DanPoint& danpoint);
  bool getLocalPos(PTrackSeg segment, DanPoint danpoint, tTrkLocPos* locpos);
  DanPoint nextPos(DanPoint danpoint);
  DanLine* mDanLine[NUM_LINES];
  private:
  PTrack mTrack;
  double mDistExt;
  double mDistInt;
  void getPath();
#ifdef TARGET_SPEEDDREAMS
  void getPath1();
#endif
};


#endif // _DANPATH_H_
