/***************************************************************************

    file                 : path.h
    created              : Sun Dec 17 07:39:00 UTC 2006
    copyright            : (C) 2006 Daniel Schellhammer, 2000 Remi Coulom

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _PATH_H_
#define _PATH_H_

#include <iostream>
#include <vector>

#include "globaldefinitions.h"
#include "d_linalg.h"


enum PathLine{IDEAL_LINE, LEFT_LINE, RIGHT_LINE, NUM_LINES};


class PathPoint {
  public:
  v2d  pos;
  v2d  left;
  v2d  right;
  double tLane;
};


class PathK1999 {
  public:
  PathK1999(PathLine line, PTrack track, double dist_ext, double dist_int);
  std::vector <PathPoint> mPath;
  private:
  PathLine mLine;
  void UpdateTxTy(int i);
  void SplitTrack(PTrack ptrack);
  double GetRInverse(int prev, double x, double y, int next);
  void AdjustRadius(int prev, int i, int next, double TargetRInverse, double Security);
  void Smooth(int step);
  void StepInterpolate(int iMin, int iMax, int step);
  void Interpolate(int step);
  void InitTrack(PTrack track);

  // variables
  int mDivs;
  double mWidth;
  double SideDistExt;
  double SideDistInt;

  // constants
  int Iterations;
  double DivLength;
};


#endif // _PATH_H_
