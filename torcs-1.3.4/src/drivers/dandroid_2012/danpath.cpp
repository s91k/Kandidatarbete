/***************************************************************************

    file                 : danpath.cpp
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


#define K1999Path 1

#include "danpath.h"


DanLine::DanLine(PTrack t)
{
  MAX_RADIUS = 10000.0;
  mTrack = t;
  myseg = mTrack->seg;
}


void DanLine::addDanPoint(DanPoint danpoint)
{
  vDanpath.push_back(danpoint);
}


bool DanLine::calcParam()
{
  int i;
  for (i = 0; i < (int)vDanpath.size(); i++) {
    if (fromStart(vDanpath[i].pos, vDanpath[i].fromstart)) {
      if (!toMiddle(vDanpath[i].pos, vDanpath[i].tomiddle)) {
        return false;
      }
    } else {
      return false;
    }
  }
  for (i = 0; i < (int)vDanpath.size(); i++) {
    vDanpath[i].yaw = calcYaw(vDanpath[i]);
    double trackyaw;
    if (!calcTrackYaw(vDanpath[i], trackyaw)) {
      return false;
    }
    vDanpath[i].angletotrack = vDanpath[i].yaw - trackyaw;
    NORM_PI_PI(vDanpath[i].angletotrack);
  }
  for (i = 0; i < (int)vDanpath.size(); i++) {
    vDanpath[i].radius = calcRadius(i);
    if (fabs(vDanpath[i].radius) < MAX_RADIUS) {
      vDanpath[i].type = (SIGN(vDanpath[i].radius) > 0) ? TR_LFT : TR_RGT;
    } else {
      vDanpath[i].type = TR_STR;
    }
  }
//  printData();
  return true;
}


void DanLine::printData()
{
  if (vDanpath[0].line == 0) {
    for (int i = 0; i < (int)vDanpath.size(); i++) {
      std::cout << "ind:" << i << " fs:" << vDanpath[i].fromstart << " r:" << vDanpath[i].radius << std::endl;
    }
    for (int i = 0; i < (int)vDansect.size(); i++) {
      std::cout << "sector:" << vDansect[i].sector << " fs:" << vDansect[i].fromstart << " speedfactor::" << vDansect[i].speedfactor << std::endl;
    }
  }
}


void DanLine::createSectors()
{
  int sector = 0;
  DanSector dansect;
  dansect.sector = sector;
  dansect.fromstart = 0.0;
  dansect.speedfactor = 0.96;
  dansect.time = 0.0;
  dansect.bestspeedfactor = 0.96;
  dansect.besttime = 10000.0;
  dansect.learned = 0;
  vDansect.push_back(dansect);
  double lastfromstart = dansect.fromstart;
  bool radiuspeek = false;
  bool largeradius = false;
  for (int i = 1 ; i < (int)vDanpath.size(); i++) {
    if (SIGN(vDanpath[i-1].radius) != SIGN(vDanpath[i].radius) || fabs(vDanpath[i].radius) > 300.0) {
      if (!largeradius) {
        radiuspeek = true;
      }
      if (fabs(vDanpath[i].radius) > 300.0) {
        largeradius = true;
      }
    }
    if (fabs(vDanpath[i].radius) < 300.0) {
      largeradius = false;
    }

    if (radiuspeek) {
      if (vDanpath[i].fromstart - lastfromstart > 100.0
        && vDanpath[vDanpath.size()-1].fromstart - vDanpath[i].fromstart > 100.0) {
        sector++;
        dansect.sector = sector;
        dansect.fromstart = vDanpath[i].fromstart;
        lastfromstart = dansect.fromstart;
        vDansect.push_back(dansect);
        //std::cout << "fs:" << vDanpath[i].fromstart << " radius:" << fabs(vDanpath[i].radius) << " delta:" << fabs(vDanpath[i-1].radius) - fabs(vDanpath[i].radius) << std::endl;
      }
      radiuspeek = false;
    }
  }
}


bool DanLine::getDanPos(double fromstart, DanPoint& danpoint)
{
  if (!vDanpath.size()) {
    return false;
  }
  int index = getIndex(fromstart);
  danpoint = vDanpath[index];
  // Interpolate radius linear
  double seglength = getDistDiff(vDanpath[index].fromstart, nextPos(vDanpath[index]).fromstart);
  double poslength = getDistDiff(vDanpath[index].fromstart, fromstart);
  double invradius = 1.0 / vDanpath[index].radius + (poslength / seglength) * (1.0 / nextPos(vDanpath[index]).radius - 1.0 / vDanpath[index].radius);
  danpoint.radius = 1.0 / invradius;
  danpoint.tomiddle = getToMiddle(fromstart); // Interpolate cubic toMiddle
  danpoint.pos = getNearestPoint(danpoint.index, fromstart); // position (straight interpolation)
  danpoint.fromstart = fromstart;
  //danpoint.yaw = calcYaw(danpoint); // useless without interpolation
  return true;
}


bool DanLine::getLocalPos(PTrackSeg segment, DanPoint danpoint, tTrkLocPos* locpos)
{
  RtTrackGlobal2Local(segment, danpoint.pos.x, danpoint.pos.y, locpos, TR_LPOS_MAIN);
  locpos->toMiddle = danpoint.tomiddle;
  return true;
}


DanPoint DanLine::nextPos(DanPoint danpoint)
{
  danpoint.index++;
  return getPos(danpoint.index);
}


DanPoint DanLine::prevPos(DanPoint danpoint)
{
  danpoint.index--;
  return getPos(danpoint.index);
}


DanPoint DanLine::getPos(int index)
{
  if (index < 0) {
    return vDanpath[vDanpath.size() - 1];
  } else if (index >= (int)vDanpath.size()) {
    return vDanpath[0];
  } else {
    return vDanpath[index];
  }
}


double DanLine::calcRadius(int index)
{
  // Do nothing if no track loaded
  if (!vDanpath.size()) {
    return 0.0;
  }
  DanPoint nextpoint = nextPos(vDanpath[index]);
  v2d nextvect = nextpoint.pos - vDanpath[index].pos;
  double nextyaw = v2d(0.0, 0.0).alpha(nextvect);
  double deltaangle = nextyaw - vDanpath[index].yaw;
  NORM_PI_PI(deltaangle);
  double radius = SIGN(deltaangle) * (nextvect.len()/2)/cos(PI/2 - fabs(deltaangle));
  if (fabs(radius) > MAX_RADIUS) {
    radius = SIGN(radius) * MAX_RADIUS;
  }
  return radius;
}


// Yaw angle of the line
double DanLine::calcYaw(DanPoint danpoint)
{
  v2d prev = danpoint.pos - prevPos(danpoint).pos;
  v2d next = nextPos(danpoint).pos - danpoint.pos;
  return v2d(0.0, 0.0).alpha(prev + next);
}


double DanLine::calcTrackYaw(DanPoint danpoint, double& trackyaw)
{
  tTrkLocPos locpos;
  RtTrackGlobal2Local(myseg, danpoint.pos.x, danpoint.pos.y, &locpos, TR_LPOS_MAIN);
  myseg = locpos.seg;
  trackyaw = RtTrackSideTgAngleL(&locpos);
  return true;
}


bool DanLine::fromStart(v2d pos, double& fromstart)
{
  tTrkLocPos locpos;
  RtTrackGlobal2Local(myseg, pos.x, pos.y, &locpos, TR_LPOS_MAIN);
  myseg = locpos.seg;
  fromstart = RtGetDistFromStart2(&locpos);
  return true;
}


bool DanLine::toMiddle(v2d pos, double& tomiddle)
{
  tTrkLocPos locpos;
  RtTrackGlobal2Local(myseg, pos.x, pos.y, &locpos, TR_LPOS_MAIN);
  myseg = locpos.seg;
  tomiddle = locpos.toMiddle;
  return true;
}


// Find nearest section on vDanpath
int DanLine::getIndex(double fromstart)
{
  double estpos = fromstart / mTrack->length;
  int i = floor(estpos * vDanpath.size());
  while (true) {
    if (i < 0) {
      i = vDanpath.size() - 1;
    } else if (i >= (int)vDanpath.size()) {
      i = 0;
    }
    double sectlen = getDistDiff(getPos(i).fromstart, getPos(i + 1).fromstart);
//    double poslen = getDistDiff(getPos(i).fromstart, fromstart);
    double poslen = getDistDiff(getPos(i).fromstart, fromstart+0.001);
    if (poslen >= 0.0 && poslen <= sectlen) {
//      std::cout << "poslen:" << poslen << " sectlen:" << sectlen << std::endl;
      break;
    }
    i += SIGN(poslen);
  }
  return i;
}


v2d DanLine::getNearestPoint(int index, double fromstart)
{
  v2d straight = getPos(index+1).pos - vDanpath[index].pos;
  double straightlen = getDistDiff(vDanpath[index].fromstart, getPos(index+1).fromstart);
  double poslen = getDistDiff(vDanpath[index].fromstart, fromstart);
  v2d pointonStraight = vDanpath[index].pos + straight * (poslen / straightlen);
  return pointonStraight;
}


double DanLine::getToMiddle(double fromstart)
{
  int index = getIndex(fromstart);
  TCubic ccurve(vDanpath[index].fromstart, vDanpath[index].tomiddle, vDanpath[index].angletotrack, nextPos(vDanpath[index]).fromstart, nextPos(vDanpath[index]).tomiddle, nextPos(vDanpath[index]).angletotrack);
  return ccurve.CalcOffset(fromstart);
}


double DanLine::getDistDiff(double fromstart1, double fromstart2)
{
  double diff = fromstart2 - fromstart1;
  diff = (diff >= 0.0) ? diff : diff + mTrack->length;
  return (diff <= mTrack->length / 2.0) ? diff : diff - mTrack->length;
}



DanPath::DanPath(PTrack t, double dist_ext, double dist_int)
{
  mTrack = t;
  mDistExt = dist_ext;
  mDistInt = dist_int;

  for (int i=0; i < NUM_LINES; i++) {
    mDanLine[i] = new DanLine(t);
  }
  if (K1999Path) {
    getPath();
  } else {
//    getPath1();
  }
  for (int i=0; i < NUM_LINES; i++) {
    if (!mDanLine[i]->calcParam()) {
      std::cout << "Error danpath: calcParam() failed" << std::endl;
    }
  }
  mDanLine[0]->createSectors();
}


DanPath::~DanPath()
{
  for (int i=0; i < NUM_LINES; i++) {
    delete mDanLine[i];
  }
}


bool DanPath::getDanPos(int line, double fromstart, DanPoint& danpoint)
{
  return mDanLine[line]->getDanPos(fromstart, danpoint);
}


bool DanPath::getLocalPos(PTrackSeg segment, DanPoint danpoint, tTrkLocPos* locpos)
{
  return mDanLine[danpoint.line]->getLocalPos(segment, danpoint, locpos);
}


DanPoint DanPath::nextPos(DanPoint danpoint)
{
  return mDanLine[danpoint.line]->nextPos(danpoint);
}


void DanPath::getPath()
{
  v2d point(0, 0);

  for (int l = 0; l < 3; l++) {
    PathK1999* path = new PathK1999((PathLine)l, mTrack, mDistExt, mDistInt);
    int index = 0;
    for (unsigned j = 0; j < path->mPath.size(); j++) {
      point.x = path->mPath[j].pos.x;
      point.y = path->mPath[j].pos.y;
      DanPoint p;
      p.line = l;
      p.index = index++;
      p.pos = point;
      p.type = 0;
      p.fromstart = 0;
      p.tomiddle = 0;
      p.radius = 0;
      p.yaw = 0;
      p.angletotrack = 0;
      mDanLine[l]->addDanPoint(p);
    }
    delete path;
  }
}


#ifdef TARGET_SPEEDDREAMS
void DanPath::getPath1()
{
  // Create track description
  TBaseTrack* oTrackDesc = new TBaseTrack();
  oTrackDesc->Initialize(mTrack);

  v2d point(0, 0);

  for (int l = 0; l < 3; l++) {
    float FixedOffset;
    if (l == 0) {
      FixedOffset = 0;
      FixedOffset = MAX((2.0-mTrack->width)/2, MIN((mTrack->width-2.0)/2,FixedOffset));
    } else if (l == 1) {
      FixedOffset = -mTrack->width/4;
      FixedOffset = MAX((2.0-mTrack->width)/2, MIN((mTrack->width-2.0)/2,FixedOffset));
    } else if (l == 2) {
      FixedOffset = mTrack->width/4;
      FixedOffset = MAX((2.0-mTrack->width)/2, MIN((mTrack->width-2.0)/2,FixedOffset));
    }
    TPath1* oLane = new TPath1();
    oLane->Initialize(oTrackDesc, FixedOffset, 1.0, 1.0, 1.0);
    int index = 0;
    for (int j = 0; j < oLane->Count; j++) {
      point.x = oLane->PPPub[j].Point.x;
      point.y = oLane->PPPub[j].Point.y ;
      DanPoint p;
      p.line = l;
      p.index = index++;
      p.pos = point;
      p.type = 0;
      p.fromstart = 0;
      p.tomiddle = 0;
      p.radius = 0;
      p.yaw = 0;
      p.angletotrack = 0;
      mDanLine[l]->addDanPoint(p);
    }
    delete oLane;
  }
 delete oTrackDesc;
}
#endif
