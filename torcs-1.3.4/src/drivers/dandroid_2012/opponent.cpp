/***************************************************************************

    file                 : opponent.cpp
    created              : Thu Aug 31 01:21:49 UTC 2006
    copyright            : (C) 2006 Daniel Schellhammer

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#include "opponent.h"


// constants
const double Opponent::FRONTRANGE = 200.0;    // [m] distance to check for other cars
const double Opponent::BACKRANGE = 500.0;    // [m] distance to check for other cars
const int Opponent::MAX_DAMAGE_DIFF = 1000;


Opponent::Opponent()
{
}


void Opponent::init(PTrack t, PSituation s, PCarElt c, PCarElt myc)
{
  car = c;
  mycar = myc;
  track = t;
  teammate = !strncmp(car->_teamname, mycar->_teamname, 10);
}


// Update the values in Opponent this
void Opponent::update(PSituation s)
{
  initState();
  // Check for cars out
  if (car->_state & RM_CAR_STATE_NO_SIMU) {
    return;
  }
  calcBasics();
  calcDist();
  calcSpeed();
  // is opponent in relevant range
  if (mDist > -BACKRANGE && mDist < FRONTRANGE) {
    // Detect backmarkers
    if ((car->_remainingLaps > mycar->_remainingLaps && !teammate)
    || (teammate && mycar->_dammage + MAX_DAMAGE_DIFF < car->_dammage && mycar->race.laps > 1)) {
      backmarker = true;
    }
    // Let opponent pass
    if ((!teammate && car->_remainingLaps < mycar->_remainingLaps)
    || (teammate && mycar->_dammage >= car->_dammage + MAX_DAMAGE_DIFF && mycar->race.laps > 1)
    || (teammate && mycar->_dammage < car->_dammage + MAX_DAMAGE_DIFF && mycar->_pos > car->_pos && !backmarker)) {
      letpass = true;
    }
    letpassaccel = backmarker && car->_accelCmd > 0.45 && car->_accelCmd < 0.85;
  }
}


void Opponent::initState()
{
  backmarker = false;
  letpass = false;
  mDist = DBL_MAX;
}


void Opponent::calcBasics()
{
  mAngleToTrack = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
  NORM_PI_PI(mAngleToTrack);
  mAngleToLeft = mAngleToTrack < 0.0 ? true : false;
  toMiddle = car->_trkPos.toMiddle;
  sidedist = toMiddle - mycar->_trkPos.toMiddle;
  borderdist = car->_trkPos.seg->width / 2.0 - fabs(toMiddle);
}


void Opponent::calcSpeed()
{
  if (fabs(mDist) < 20.0) {
    double angle = car->_yaw - mycar->_yaw;
    NORM_PI_PI(angle);
    if (fabs(angle) > 0.5) {
      speed = getSpeed(mycar->_yaw);
    } else {
      speed = car->_speed_x;
    }
  } else {
    speed = getSpeed(RtTrackSideTgAngleL(&(car->_trkPos)));
  }
}


// compute speed component parallel to angle
double Opponent::getSpeed(double angle)
{
  v2d speed, dir;
  speed.x = car->_speed_X;
  speed.y = car->_speed_Y;
  dir.x = cos(angle);
  dir.y = sin(angle);
  return speed * dir;
}


void Opponent::calcDist()
{
  // calculate distance
  mDist = car->_distFromStartLine - mycar->_distFromStartLine;
  if (mDist > track->length / 2.0) {
    mDist -= track->length;
  } else if (mDist < -track->length / 2.0) {
    mDist += track->length;
  }
  if (fabs(mDist) < 30.0) {
    double fraction = MAX(0.0, (fabs(mDist) - 15.0) / 15.0);
    double dX = car->_pos_X - mycar->_pos_X;
    double dY = car->_pos_Y - mycar->_pos_Y;
    mDist = fraction * mDist + (1.0 - fraction) * sqrt(dX * dX + dY * dY - sidedist * sidedist) * SIGN(mDist);
  }
  double COLLDIST = 0.97 * car->_dimension_x;
  mAside = false;
  if (mDist >= COLLDIST) {
    // opponent in front
    mDist -= COLLDIST;
  } else if (mDist <= -COLLDIST) {
    // opponent behind
    mDist += COLLDIST;
  } else {
    // opponent aside
    mDist = cornerDist();
    mAside = true;
  }
}


// For overlaping distances
double Opponent::cornerDist()
{
  Straight frontLine(
    mycar->_corner_x(FRNT_LFT),
    mycar->_corner_y(FRNT_LFT),
    mycar->_corner_x(FRNT_RGT) - mycar->_corner_x(FRNT_LFT),
    mycar->_corner_y(FRNT_RGT) - mycar->_corner_y(FRNT_LFT)
  );
  Straight rearLine(
    mycar->_corner_x(REAR_LFT),
    mycar->_corner_y(REAR_LFT),
    mycar->_corner_x(REAR_RGT) - mycar->_corner_x(REAR_LFT),
    mycar->_corner_y(REAR_RGT) - mycar->_corner_y(REAR_LFT)
  );
  Straight leftLine(
    mycar->_corner_x(FRNT_LFT),
    mycar->_corner_y(FRNT_LFT),
    mycar->_corner_x(REAR_LFT) - mycar->_corner_x(FRNT_LFT),
    mycar->_corner_y(REAR_LFT) - mycar->_corner_y(FRNT_LFT)
  );
  Straight rightLine(
    mycar->_corner_x(FRNT_RGT),
    mycar->_corner_y(FRNT_RGT),
    mycar->_corner_x(REAR_RGT) - mycar->_corner_x(FRNT_RGT),
    mycar->_corner_y(REAR_RGT) - mycar->_corner_y(FRNT_RGT)
  );
  double mindist = DBL_MAX;
  bool left[4];
  bool right[4];
  for (int i = 0; i < 4; i++) {
    v2d corner(car->_corner_x(i), car->_corner_y(i));
    double frontdist = frontLine.dist(corner);
    double reardist = rearLine.dist(corner);
    double leftdist = leftLine.dist(corner);
    double rightdist = rightLine.dist(corner);
    bool front = frontdist < reardist && reardist > mycar->_dimension_x ? true : false;
    bool rear = reardist < frontdist && frontdist > mycar->_dimension_x ? true : false;
    left[i] = leftdist < rightdist && rightdist > mycar->_dimension_y ? true : false;
    right[i] = rightdist < leftdist && leftdist > mycar->_dimension_y ? true : false;
    double dist = DBL_MAX;
    if (front) {
      dist = frontdist;
    } else if (rear) {
      dist = -reardist;
    }
    if (fabs(dist) < fabs(mindist)) {
      mindist = dist;
    }
  }
  if (fabs(mindist) > 3.0) {
    mindist -= SIGN(mindist) * 2.99;
  } else {
    mindist = SIGN(mindist) * 0.01;
  }
  bool lft = true;
  bool rgt = true;
  for (int j = 0; j < 4; j++) {
    if (!left[j]) {
      lft = false;
    }
  }
  for (int k = 0; k < 4; k++) {
    if (!right[k]) {
      rgt = false;
    }
  }
  if (lft || rgt) {
    // opponent aside
    mindist = 0.0;
  }
  return mindist;
}


// Initialize the list of opponents
Opponents::Opponents(PTrack t, PSituation s, PCarElt car)
{
  opponent = new Opponent[s->_ncars - 1];
  int i, j = 0;
  for (i = 0; i < s->_ncars; i++) {
    if (s->cars[i] != car) {
      opponent[j].init(t, s, s->cars[i], car);
      j++;
    }
  }
  nopponents = s->_ncars - 1;
}


Opponents::~Opponents()
{
  delete [] opponent;
}


// Updates all the Opponent instances
void Opponents::update(PSituation s, PCarElt mycar)
{
  oppnear = NULL;
  oppnear2 = NULL;
  double mindist = DBL_MAX;
  double mindist2 = DBL_MAX;
  double minside = DBL_MAX;
  double minside2 = DBL_MAX;
  oppletpass = NULL;
  double minletpass = -DBL_MAX;
  oppback = NULL;
  double minback = -DBL_MAX;
  int i;
  for (i = 0; i < nopponents; i++) {
    // Update opponent
    opponent[i].update(s);
    // Get nearest opponent
    double dist = opponent[i].mDist;
    double sidedist = opponent[i].sidedist;
    if (opponent[i].mAside) {
      if (fabs(sidedist) < fabs(minside)) {
        minside = sidedist;
        mindist = 0.0;
        oppnear = &opponent[i];
      }
    } else if (dist > -2.0 && fabs(dist) <= fabs(mindist) && fabs(sidedist) < 15.0) {
      mindist = dist;
      oppnear = &opponent[i];
    }
    // Get opponent to let passing
    if (opponent[i].letpass) {
      if (dist <= 0.0 && dist >= minletpass) {
        minletpass = dist;
        oppletpass = &opponent[i];
      }
    }
    // Get nearest back opponent
    if (dist < 0.0 && dist >= minback) {
      minback = dist;
      oppback = &opponent[i];
    }
  }
  // Get 2. nearest opponent
  for (i = 0; i < nopponents; i++) {
    double dist = opponent[i].mDist;
    double sidedist = opponent[i].sidedist;
    if (opponent[i].mAside) {
      if (fabs(sidedist) > fabs(minside) && fabs(sidedist) < fabs(minside2)) {
        minside2 = sidedist;
        mindist2 = 0.0;
        oppnear2 = &opponent[i];
      }
    } else if (dist > -2.0 && fabs(dist) > fabs(mindist) && fabs(dist) < fabs(mindist2) && fabs(sidedist) < 15.0) {
      mindist2 = dist;
      oppnear2 = &opponent[i];
    }
  }
}
