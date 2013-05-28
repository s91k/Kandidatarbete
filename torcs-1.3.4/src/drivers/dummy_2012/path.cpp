/***************************************************************************

    file                 : path.cpp
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


#include "path.h"


//
// Debugging options
//
//#define VERBOSE


// Some utility macros and functions
#ifdef VERBOSE
#define OUTPUT(x) do {(cout << "Path K1999: " << x << '\n').flush();} while(0)
#else
#define OUTPUT(x)
#endif


PathK1999::PathK1999(PathLine line, PTrack track, double dist_ext, double dist_int)
{
 mLine = line;
 Iterations = 100;  // Number of smoothing operations
 DivLength = 3.0;   // Length of path elements in meters
 SideDistExt = dist_ext;
 SideDistInt = dist_int;
 InitTrack(track);
}


void PathK1999::UpdateTxTy(int i)
{
 mPath[i].pos.x = mPath[i].tLane * mPath[i].right.x + (1 - mPath[i].tLane) * mPath[i].left.x;
 mPath[i].pos.y = mPath[i].tLane * mPath[i].right.y + (1 - mPath[i].tLane) * mPath[i].left.y;
}


// Split the track into small elements ??? constant width supposed
void PathK1999::SplitTrack(PTrack ptrack)
{
 OUTPUT("Analyzing track...");
 const tTrackSeg *psegCurrent = ptrack->seg;

 double Length = 0;
 double Angle = psegCurrent->angle[TR_ZS];
 v2d pos;
 pos.x = (psegCurrent->vertex[TR_SL].x + psegCurrent->vertex[TR_SR].x) / 2;
 pos.y = (psegCurrent->vertex[TR_SL].y + psegCurrent->vertex[TR_SR].y) / 2;
 if (mLine == LEFT_LINE) {
  pos.x = (pos.x + psegCurrent->vertex[TR_SL].x) / 2;
  pos.y = (pos.y + psegCurrent->vertex[TR_SL].y) / 2;
 } else if (mLine == RIGHT_LINE) {
  pos.x = (pos.x + psegCurrent->vertex[TR_SR].x) / 2;
  pos.y = (pos.y + psegCurrent->vertex[TR_SR].y) / 2;
 }
 do {
  int Divisions = 1 + int(psegCurrent->length / DivLength);
  double Step = psegCurrent->length / Divisions;

  for (int j = Divisions; --j >= 0;) {
   double cosine = cos(Angle);
   double sine = sin(Angle);

   if (psegCurrent->type == TR_STR) {
    pos.x += cosine * Step;
    pos.y += sine * Step;
   } else {
    double r = psegCurrent->radius;
    if (mLine == LEFT_LINE) {
     r = (psegCurrent->radius + psegCurrent->radiusl) / 2;
    } else if (mLine == RIGHT_LINE) {
     r = (psegCurrent->radius + psegCurrent->radiusr) / 2;
    }
    double Theta = psegCurrent->arc / Divisions;
    double L = 2 * r * sin(Theta / 2);
    double x = L * cos(Theta / 2);
    double y;
    if (psegCurrent->type == TR_LFT) {
     Angle += Theta;
     y = L * sin(Theta / 2);
    } else {
     Angle -= Theta;
     y = -L * sin(Theta / 2);
    }
    pos.x += x * cosine - y * sine;
    pos.y += x * sine + y * cosine;
   }

   mWidth = psegCurrent->width;
   if (mLine != IDEAL_LINE) {
    mWidth = psegCurrent->width / 2;
   }

   v2d delta;
   delta.x = -mWidth * sin(Angle) / 2;
   delta.y = mWidth * cos(Angle) / 2;

   PathPoint pathPoint;
   pathPoint.left = pos + delta;
   pathPoint.right = pos - delta;
   pathPoint.tLane = 0.5;
   mPath.push_back(pathPoint);
   UpdateTxTy(mPath.size() - 1);

   Length += Step;
  }

  psegCurrent = psegCurrent->next;
 }
 while (psegCurrent != ptrack->seg);

 mDivs = mPath.size();

 OUTPUT("Position of the last point (should be (0, 0))");
 OUTPUT("pos.x = " << pos.x);
 OUTPUT("pos.y = " << pos.y);
 OUTPUT("Number of path elements : " << mDivs);
 OUTPUT("Track length : " << Length);
 OUTPUT("Width : " << mWidth);
}


// Compute the inverse of the radius
double PathK1999::GetRInverse(int prev, double x, double y, int next)
{
 double x1 = mPath[next].pos.x - x;
 double y1 = mPath[next].pos.y - y;
 double x2 = mPath[prev].pos.x - x;
 double y2 = mPath[prev].pos.y - y;
 double x3 = mPath[next].pos.x - mPath[prev].pos.x;
 double y3 = mPath[next].pos.y - mPath[prev].pos.y;

 double det = x1 * y2 - x2 * y1;
 double n1 = x1 * x1 + y1 * y1;
 double n2 = x2 * x2 + y2 * y2;
 double n3 = x3 * x3 + y3 * y3;
 double nnn = sqrt(n1 * n2 * n3);

 return 2 * det / nnn;
}


// Change lane value to reach a given radius
void PathK1999::AdjustRadius(int prev, int i, int next, double TargetRInverse, double Security)
{
 double OldLane = mPath[i].tLane;

 // Start by aligning points for a reasonable initial lane
 mPath[i].tLane = (-(mPath[next].pos.y - mPath[prev].pos.y) * (mPath[i].left.x - mPath[prev].pos.x) +
              (mPath[next].pos.x - mPath[prev].pos.x) * (mPath[i].left.y - mPath[prev].pos.y)) /
            ( (mPath[next].pos.y - mPath[prev].pos.y) * (mPath[i].right.x - mPath[i].left.x) -
              (mPath[next].pos.x - mPath[prev].pos.x) * (mPath[i].right.y - mPath[i].left.y));
 if (mPath[i].tLane < -0.2) {
  mPath[i].tLane = -0.2;
 } else if (mPath[i].tLane > 1.2) {
  mPath[i].tLane = 1.2;
 }
 UpdateTxTy(i);

 // Newton-like resolution method
 const double dLane = 0.0001;
 v2d delta;
 delta = dLane * (mPath[i].right - mPath[i].left);
 double dRInverse = GetRInverse(prev, mPath[i].pos.x + delta.x, mPath[i].pos.y + delta.y, next);

 if (dRInverse > 0.000000001) {
  mPath[i].tLane += (dLane / dRInverse) * TargetRInverse;
  double ExtLane = (SideDistExt + Security) / mWidth;
  double IntLane = (SideDistInt + Security) / mWidth;
  if (ExtLane > 0.5) {
   ExtLane = 0.5;
  }
  if (IntLane > 0.5) {
   IntLane = 0.5;
  }
  if (TargetRInverse >= 0.0) {
   if (mPath[i].tLane < IntLane) {
    mPath[i].tLane = IntLane;
   }
   if (1 - mPath[i].tLane < ExtLane) {
    if (1 - OldLane < ExtLane) {
     mPath[i].tLane = MIN(OldLane, mPath[i].tLane);
    } else {
     mPath[i].tLane = 1 - ExtLane;
    }
   }
  } else {
   if (mPath[i].tLane < ExtLane) {
    if (OldLane < ExtLane) {
     mPath[i].tLane = MAX(OldLane, mPath[i].tLane);
    } else {
     mPath[i].tLane = ExtLane;
    }
   }
   if (1 - mPath[i].tLane < IntLane) {
    mPath[i].tLane = 1 - IntLane;
   }
  }
 }

 UpdateTxTy(i);
}


void PathK1999::Smooth(int step)
{
 int prev = mDivs - step;
 int prevprev = prev - step;
 int next = step;
 int nextnext = next + step;

 for (int i = 0; i <= mDivs - step; i += step) {
  double ri0 = GetRInverse(prevprev, mPath[prev].pos.x, mPath[prev].pos.y, i);
  double ri1 = GetRInverse(i, mPath[next].pos.x, mPath[next].pos.y, nextnext);
  double lPrev = (mPath[i].pos - mPath[prev].pos).len();
  double lNext = (mPath[i].pos - mPath[next].pos).len();
  double TargetRInverse = 1.006*(lNext * ri0 + lPrev * ri1) / (lNext + lPrev);

  double SecurityR = 100.0;
  double Security = lPrev * lNext / (8 * SecurityR);
  AdjustRadius(prev, i, next, TargetRInverse, Security);

  prevprev = prev;
  prev = i;
  next = nextnext;
  nextnext = next + step;
  if (nextnext > mDivs - step) {
   nextnext = 0;
  }
 }
}


// Interpolate between two control points
void PathK1999::StepInterpolate(int iMin, int iMax, int step)
{
 int next = (iMax + step) % mDivs;
 if (next > mDivs - step) {
  next = 0;
 }

 int prev = (((mDivs + iMin - step) % mDivs) / step) * step;
 if (prev > mDivs - step) {
  prev -= step;
 }

 double ir0 = GetRInverse(prev, mPath[iMin].pos.x, mPath[iMin].pos.y, iMax % mDivs);
 double ir1 = GetRInverse(iMin, mPath[iMax % mDivs].pos.x, mPath[iMax % mDivs].pos.y, next);
 for (int k = iMax; --k > iMin;) {
  double x = double(k - iMin) / double(iMax - iMin);
  double TargetRInverse = x * ir1 + (1 - x) * ir0;
  AdjustRadius(iMin, k, iMax % mDivs, TargetRInverse, 0);
 }
}


// Calls to StepInterpolate for the full path
void PathK1999::Interpolate(int step)
{
 if (step > 1) {
  int i;
  for (i = step; i <= mDivs - step; i += step) {
   StepInterpolate(i - step, i, step);
  }
  StepInterpolate(i - step, mDivs, step);
 }
}


void PathK1999::InitTrack(PTrack track)
{
 SplitTrack(track);
 for (int step = 128; (step /= 2) > 0;) {
  OUTPUT("Step = " << step);
  for (int i = Iterations * int(sqrt((double)step)); --i >= 0;) {
   Smooth(step);
  }
  Interpolate(step);
 }
}

