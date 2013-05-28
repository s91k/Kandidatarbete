/***************************************************************************

    file                 : driver.cpp
    created              : 2006-08-31 01:21:49 UTC
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


#include "driver.h"
#include <string>

//#define DRIVER_DEBUG
//#define TIME_ANALYSIS // For Linux only

#ifdef TIME_ANALYSIS
#include <sys/time.h>
#endif

#define Gravity 9.81

//==========================================================================*
// Constructor
//--------------------------------------------------------------------------*
#ifdef TARGET_TORCS
TDriver::TDriver(int index)
#else
TDriver::TDriver() : TBaseDriver()
#endif
{
  // Constants
#ifdef TARGET_TORCS
  RobotName = ROBOT_NAME;
  oCar = NULL;
#endif
  mLOOKAHEAD_CONST = 4.0;         // [m]
  mFRONTCOLL_MARGIN = 2.0;        // [m]
  mOVT_FRONTSPACE = 20.0;         // [m]
  mOVT_FRONTMARGIN = 5.0;         // [m]

  // Variables
  mTrack = NULL;
  mPrevgear = 0;
  mAccel = 0.0;
  mDrvState = DRV_RACE;
  mDrvLane = DRV_O;
  mTenthTimer = false;
  mStuck = false;
  mStuckcount = 0;
  mOldTimer = 0.0;
  mAbsFactor = 0.5;
  mTclFactor = 0.5;
  mCatchedRaceLine = false;
  mClutchtime = 0.0;
  mOldTargetToMiddle = 0.0;
  mSlipSideFront = 0.0;
  mSlipSideRear = 0.0;
  mPrevTargetdiff = 0.0;
  mOppInFrontspace = false;
  mOvtSector = false;
  mDangerSector = false;
  mPath[DRV_O].danpoint.radius = 1000.0;
  mCentifugal = 0.0;
  mSpeedfactor = 1.0;
  mLastDamage = 0;
  mOfftracksector = -1;
  mMaxSteerAngle = false;
  mOvertake = false;
}


//==========================================================================*

//==========================================================================*
// destructor
//--------------------------------------------------------------------------*
TDriver::~TDriver()
{
  delete mDanPath;
  delete mOpponents;
  delete mPit;
}
//==========================================================================*


void TDriver::InitTrack(PTrack Track, PCarHandle CarHandle, PCarSettings *CarParmHandle, PSituation Situation)
{
  mTrack = Track;
  *CarParmHandle = NULL;
  // Get file handles
  char* trackname = strrchr(Track->filename, '/') + 1;
  char buffer[256];
  switch (Situation->_raceType) {
    case RM_TYPE_QUALIF:
      sprintf(buffer, "drivers/%s/setups/qualifying/%s", RobotName, trackname);
      *CarParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
      break;
    default:
      break;
  }
  if (*CarParmHandle == NULL) {
    sprintf(buffer, "drivers/%s/setups/%s", RobotName, trackname);
    *CarParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
  }
  if (*CarParmHandle == NULL) {
    sprintf(buffer, "drivers/%s/setups/default.xml", RobotName);
    *CarParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
  }
  // Initial fuel
  mTankvol = GfParmGetNum(CarHandle, SECT_CAR, PRM_TANK, (char*)NULL, 50);
  mLearning = GfParmGetNum(*CarParmHandle, "private", "learning", (char*)NULL, 0.0);
  bool testpitstop = GfParmGetNum(*CarParmHandle, "private", "test pitstop", (char*)NULL, 0.0);
  mTestLine = GfParmGetNum(*CarParmHandle, "private", "test line", (char*)NULL, 0.0);
  mDriverMsgLevel = GfParmGetNum(*CarParmHandle, "private", "driver message", (char*)NULL, 0.0);
  mFuelPerMeter = GfParmGetNum(*CarParmHandle, "private", "fuelpermeter", (char*)NULL, 0.0008);
  double distance = Situation->_totLaps * mTrack->length;
  if (testpitstop) {
    distance = 0.8 * mTrack->length;
  }
  double fuel = getFuel(distance);
  GfParmSetNum(*CarParmHandle, SECT_CAR, PRM_FUEL, (char*)NULL, MIN(fuel, mTankvol));
}


void TDriver::NewRace(PtCarElt Car, PSituation Situation)
{
  oCar = Car;
  oSituation = Situation;
  initCa();
  initCw();
  readSpecs();
  readPrivateSection();
  mDanPath = new DanPath(mTrack, mEXTMARGIN, mINTMARGIN);
  mOpponents = new Opponents(mTrack, Situation, Car);
  mPit = new Pit(mTrack, Situation, Car, mPITDAMAGE, mPITENTRYMARGIN);
  if (!readSectorSpeeds()) {
    vSect = mDanPath->mDanLine[0]->vDansect;
  }
  mLearnSectTime = true;
  mLearnLap = true;
}


#ifdef TARGET_SPEEDDREAMS
void TDriver::Update(PtCarElt Car, PSituation Situation)
{
  oCar = Car;                                  // Update pointers
  oSituation = Situation;
}


bool TDriver::IsStuck()
{
  return false;
}


void TDriver::Unstuck()
{
}
#endif


void TDriver::Drive()
{
#ifdef TIME_ANALYSIS
  struct timeval tv;
  gettimeofday(&tv, NULL); 
  double usec1 = tv.tv_usec;
#endif
  driverMsg(4, "vmax", 3.6 * mMaxspeed);
#ifdef TARGET_TORCS
  clearControls();
  updateTime();
#endif
  updateTimer();
  updateBasics();
  updateOpponents();
  updatePath();
  calcDrvState();
  calcTarget();
  calcMaxspeed();
  setControls();
#ifdef TIME_ANALYSIS
  gettimeofday(&tv, NULL); 
  double usec2 = tv.tv_usec;
  std::cout << usec2 - usec1 << " useconds" << std::endl;
#endif
}


int TDriver::PitCmd()                               // Handle pitstop
{
  mPit->pitCommand();
  return ROB_PIT_IM;  // Ready to be serviced
}


void TDriver::EndRace()                             // Stop race
{
  // This is never called by TORCS! Don't use it!
}


void TDriver::Shutdown()                            // Cleanup
{
}


/***************************************************************************
 *
 * utility functions
 *
***************************************************************************/


#ifdef TARGET_TORCS
void TDriver::clearControls()
{
  memset(&oCar->ctrl, 0, sizeof(tCarCtrl));
}


void TDriver::updateTime()
{
  oCurrSimTime = oSituation->currentTime;
}
#endif


void TDriver::updateTimer()
{
  double diff = oCurrSimTime - mOldTimer;
  if (diff >= 0.1) {
    mOldTimer += 0.1;
    mTenthTimer = true;
  } else {
    mTenthTimer = false;
  }
}


void TDriver::updateBasics()
{
  mPit->update();
  mMass = mCARMASS + mFUELWEIGHTFACTOR * oCar->_fuel;
  mSpeed = oCar->_speed_x;
  mFromStart = oCar->_distFromStartLine;
  mToMiddle = oCar->_trkPos.toMiddle;
  mOnLeftSide = mToMiddle > 0.0 ? true : false;
  mBorderdist = oCar->_trkPos.seg->width / 2.0 - fabs(mToMiddle) - oCar->_dimension_y / 2.0;
  mWalldist = oCar->_trkPos.seg->width / 2.0 - fabs(mToMiddle) - oCar->_dimension_x / 2.0;
  if (oCar->_trkPos.seg->side[mOnLeftSide] != NULL) {
    mWalldist += oCar->_trkPos.seg->side[mOnLeftSide]->width - 1.0;
    if (oCar->_trkPos.seg->side[mOnLeftSide]->side[mOnLeftSide] != NULL) {
      mWalldist += oCar->_trkPos.seg->side[mOnLeftSide]->side[mOnLeftSide]->width;
    }
  }
  mGlobalCarPos.x = oCar->_pos_X;
  mGlobalCarPos.y = oCar->_pos_Y;
  mAngleToTrack = RtTrackSideTgAngleL(&(oCar->_trkPos)) - oCar->_yaw;
  NORM_PI_PI(mAngleToTrack);
  mAngleToLeft = mAngleToTrack < 0.0 ? true : false;
  mPointingToWall = (mAngleToLeft == mOnLeftSide) ? true : false;
  mMu = oCar->_trkPos.seg->surface->kFriction;
  mFriction = mMu * (mMass * Gravity + mCA * mSpeed * mSpeed);
  mCentifugal = mMass * mSpeed * mSpeed / mPath[DRV_O].danpoint.radius;
  mBrakeFriction = sqrt(mFriction * mFriction - 0.2 * mCentifugal * mCentifugal);
  mBrakeforce = mBRAKEFORCEFACTOR * (mBrakeFriction / mBRAKEPRESS);
  updateDanSectorId();
  learnSpeedFactors();
  getSpeedFactors();
  getOvtSectors();
  getDangerSectors();
  updateStuck();
  updateAttackAngle();
}


void TDriver::updateOpponents()
{
  mOpponents->update(oSituation, oCar);
  mOppNear = mOpponents->oppNear();
  mOppNear2 = mOpponents->oppNear2();
  mOppBack = mOpponents->oppBack();
  mOppLetPass = mOpponents->oppLetPass();
  mOpp = mOppNear;
  // Second Oppenent near?
  mBackmarkerInFrontOfTeammate = false;
  mTwoOppsAside = false;
  if (mOppNear2 != NULL) {
    // Watch for backmarkers in front of teammate
    if (mOppNear2->backmarker && mOpp->teammate
    && mOpp->speed > 15.0
    && mOpp->mDist > 1.0
    && mOppNear2->mDist < 2.0 * mOVT_FRONTSPACE) {
      mBackmarkerInFrontOfTeammate = true;
    }
    // Check if 2. Opponent aside
    if (mOppNear2->mAside) {
      mTwoOppsAside = true;
    }
  }
  // Distances
  mOppDist = DBL_MAX;
  mOppSidedist = DBL_MAX;
  mOppAside = false;
  if (mOpp != NULL) {
    mOppDist = mOpp->mDist;
    if (mOpp->mAside) {
      mOppDist = 0.0;
      mOppSidedist = mOpp->sidedist;
      mOppAside = true;
    }
    mOppLeft = (mOpp->toMiddle > 0.0) ? true : false;
    mOppLeftHyst = hysteresis(mOppLeftHyst, mOpp->toMiddle, 0.5);
    mOppLeftOfMe = (mOpp->toMiddle - mToMiddle > 0.0) ? true : false;
    mOppLeftOfMeHyst = hysteresis(mOppLeftOfMeHyst, mOpp->toMiddle - mToMiddle, 0.3);
    mOppInFrontspace = (mOppDist < mOVT_FRONTSPACE && mOppDist >= 0.0) ? true : false;
  }
}


void TDriver::updatePath()
{
  for (int path = 0; path < 3; path++) {
    updatePathCar(path);
    updatePathTarget(path);
    updatePathOffset(path);
    updatePathSpeed(path);
  }
  updateDrivingFast();
  updateCatchedRaceLine();
}


void TDriver::calcTarget()
{
  calcTargetToMiddle();
  calcGlobalTarget();
  calcTargetAngle();
}


void TDriver::setControls()
{
  oCar->_steerCmd = getSteer();
  oCar->_gearCmd = getGear();
  oCar->_clutchCmd = getClutch();  // must be after gear
  oCar->_brakeCmd = filterABS(getBrake(mMaxspeed));
  oCar->_accelCmd = filterTCL(getAccel(mMaxspeed));  // must be after brake
}


double TDriver::getMaxSpeed(DanPoint danpoint)
{
  double maxlookaheaddist = brakeDist1(mSpeed, 0.0);
  double lookaheaddist = 0.0;
  double nextradius;
  double maxspeed = DBL_MAX;
  double lowest = DBL_MAX;
  double radius;
  radius = fabs(danpoint.radius);
  while (lookaheaddist < maxlookaheaddist) {
    danpoint = mDanPath->nextPos(danpoint);
    nextradius = fabs(danpoint.radius);
    lookaheaddist = fromStart(danpoint.fromstart - mFromStart);
    maxspeed = brakeSpeed(mSpeed, radius, lookaheaddist, nextradius);
    if (lowest > maxspeed) {
      lowest = maxspeed;
    }
  }
  return MIN(1000, MIN(lowest, curveSpeed(radius)));
}


double TDriver::getPitSpeed()
{
  double speedEntryExit = mPit->getPitstop() ? mPITENTRYSPEED : mPITEXITSPEED;
  double pitlimitdist = fromStart(mPit->getLimitEntry() - mFromStart);
  double maxspeed = speedEntryExit;
  if (pitlimitdist < brakeDist1(mSpeed, mPit->getSpeedlimit())
  || mPit->isPitlimit(mFromStart)) {
    maxspeed = mPit->getSpeedlimit();
  }
  double brakespeed = brakeSpeed1(mPit->getDist(), 0.0);
  maxspeed = MIN(maxspeed, brakespeed);
  return maxspeed;
}


double TDriver::curveSpeed(double radius)
{
  double curvespeedlimit = DBL_MAX;
  if (radius) {
    curvespeedlimit = sqrt(mMu * Gravity * radius / (1.0 - MIN(0.99, radius * mCA * mMu / mMass)));
  }
  return curvespeedlimit;
}


double TDriver::brakeSpeed1(double nextdist, double nextspeed)
{
  return sqrt(nextspeed * nextspeed + nextdist * 2.0 * mMu * Gravity);
}


double TDriver::brakeSpeed(double speed, double radius, double nextdist, double nextradius)
{
  double brakespeedlimit = DBL_MAX;
  if (nextradius) {
    double nextspeed = curveSpeed(nextradius);
    double brakedistfactor = mBRAKEDISTFACTOR;
    if (!(mDrvState == DRV_RACE && mCatchedRaceLine) || mOppInFrontspace) {
      brakedistfactor *= 1.2;
    }
    if ((brakedistfactor * brakeDist(radius, speed, nextspeed)) > nextdist) {
      brakespeedlimit = nextspeed;
    }
  }
  return brakespeedlimit;
}


double TDriver::getBrake(double maxspeed)
{
  double brakeforce = 0.0;
  if (mSpeed > maxspeed + 3.0) {
    brakeforce = mBrakeforce;
  } else if (mSpeed > maxspeed) {
    brakeforce = mBrakeforce / 4.0;
  }
  if (mDrvState == DRV_OFFTRACK) {
    brakeforce *= 0.2;
  }
  if (mDrvState == DRV_PIT) {
    if (mSpeed > maxspeed) {
      brakeforce = mBrakeforce;
    } else if (mSpeed > maxspeed - 0.1) {
      brakeforce = 0.05;
    }
  }
  double collbrakeforce = 0.0;
  if (onCollision()) {
    double factor = 1.0;
    if (mOvtSector) {
      factor = 0.3;
    }
    collbrakeforce = 1.1 * mBrakeforce + 0.1;
    if (mSpeed > 10.0) {
      collbrakeforce = MIN(collbrakeforce, mBrakeforce * factor * ((mSpeed + 0.5) - mCollOppSpeed));
    }
  }
  brakeforce = MAX(collbrakeforce, brakeforce);
  brakeforce = MIN(1.0, brakeforce);

  if (mDrvState == DRV_STUCK) {
    brakeforce = 0.0;
  }

  return brakeforce;
}


double TDriver::getAccel(double maxspeed)
{
  double accel;
  if (oCar->ctrl.brakeCmd > 0.0
  || fabs(mAttackAngle) > 0.3
  || (mMaxSteerAngle && mDrivingFast)) {
    accel = 0.0;
    mAccel = 0.5;
  } else {
    controlSpeed(mAccel, maxspeed);
    if (mDrvState == DRV_LETPASS) {
      mAccel *= 0.7;
    }
    accel = mAccel;
  }
  return accel;
}


double TDriver::getSteer()
{
  if (mDrvState == DRV_STUCK) {
    mTargetAngle = -mAngleToTrack;
  }
  limitSteerAngle(mTargetAngle);
  controlOffset(mTargetAngle);
  controlAttackAngle(mTargetAngle);
  controlYawRate(mTargetAngle);
  return mTargetAngle / oCar->_steerLock;
}


int TDriver::getGear()
{
  const double SHIFT_UP = 0.99;          // [-] (% of rpmredline)
  const double SHIFT_DOWN_MARGIN = 120.0;    // [rad/s] down from rpmredline
  if (mDrvState == DRV_STUCK) {
    return -1;
  }
  if (oCar->_gear <= 0) {
    return 1;
  }
  if (oCar->_enginerpm / oCar->_enginerpmRedLine > SHIFT_UP) {
    return oCar->_gear + 1;
  } else {
    double ratiodown = oCar->_gearRatio[oCar->_gear + oCar->_gearOffset - 1] / oCar->_gearRatio[oCar->_gear + oCar->_gearOffset];
    if (oCar->_gear > 1 && (oCar->_enginerpmRedLine - SHIFT_DOWN_MARGIN) / oCar->_enginerpm > ratiodown) {
      return oCar->_gear - 1;
    }
  }
  return oCar->_gear;
}


double TDriver::getClutch()
{
  if (oCar->_gear > 1 || mSpeed > 15.0) {
    if (oCar->_gear > mPrevgear) {
      mClutchtime = 0.6;
    }
    if (mClutchtime > 0.0) {
      mClutchtime -= 1.0 * RCM_MAX_DT_ROBOTS;
    }
    if (oCar->_gear < mPrevgear) {
      mClutchtime = 0.0;
    }
  } else if (oCar->_gear == 1) {
    // enginerpm are rad/sec
    if (oCar->_enginerpm > 700.0) {
      mClutchtime -= 0.01;
    } else {
      mClutchtime += 0.01;
    }
    if (fabs(mAngleToTrack) > 1.0 || mDrvState == DRV_OFFTRACK) {
      mClutchtime = 0.0;
    }
  } else if (oCar->_gear == -1) {
    // For the reverse gear.
    if (oCar->_enginerpm > 500.0) {
      mClutchtime -= 0.01;
    } else {
      mClutchtime += 0.01;
    }
  } else if (oCar->_gear == 0){
    // For a good start
    mClutchtime = 0.7;
  }
  mPrevgear = oCar->_gear;
  mClutchtime = MIN(MAX(0.0, mClutchtime), 1.0);
  return mClutchtime;
}


bool TDriver::stateStuck()
{
  if (mStuck) {
    driverMsg(3, "Stuck (speed):", mSpeed);
    return true;
  }
  return false;
}


bool TDriver::stateOfftrack()
{
  if (mDrvState != DRV_PIT) {
    if (mBorderdist < -2.2 || (mSpeed < 15.0 && mBorderdist < -1.8)) {
      driverMsg(3, "Offtrack (borderdist):", mBorderdist);
      return true;
    }
  }
  return false;
}


bool TDriver::statePit()
{
  if (mPit->getPitOffset(mFromStart)) {
    return true;
  }
  return false;
}


bool TDriver::stateLetpass()
{
  if (mOppLetPass == NULL) {
    return false;
  }
  // Check range
  if (mOppLetPass->mDist < -20.0 || mOppLetPass->mDist > 0.0) {
    return false;
  }
  // Check for other opponent between behind
  if (mOppBack != NULL) {
    if (mOppBack != mOppLetPass && mOppBack->mDist > mOppLetPass->mDist) {
      return false;
    }
  }
  // Check for other opponent aside
  if (mOppNear != NULL) {
    if (mOppNear != mOppLetPass) {
      if (fabs(mOppNear->mDist) < 2.0) {
        return false;
      }
    }
  }
  // Check for bad conditions
  if (mDrvState != DRV_LETPASS) {
    if (mDrivingFast || mSpeed > mOppLetPass->speed + 1.0) {
      return false;
    }
  }
  return true;
}


void TDriver::setDrvState(DriveState state)
{
  mStateChange = false;
  if (mDrvState != state) {
    driverMsg(2, "New state:", state);
    mDrvState = state;
    mStateChange = true;
  }
  if (mDrvState == DRV_LETPASS) {
    mOpp = mOppLetPass;
  }
}


double TDriver::laneOffs(DriveLane lane)
{
  double offs = 0.0;
  if (mDrvState == DRV_RACE) {
    if (lane == DRV_LR) {
      offs = (mPath[DRV_L].offset + mPath[DRV_R].offset) / 2.0;
    } else {
      offs = mPath[lane].offset;
    }
  }
  return offs;
}


void TDriver::setDrvLane(DriveLane lane)
{
  mLaneChange = false;
  if (mDrvLane != lane) {
    // Don't change when dangerous or speed on limits
    if (((mDangerSector || mDrivingFast) && !mOppAside) || mTwoOppsAside) {
      return;
    }
    // Choose first middle lane if lane offset too far
    if (fabs(laneOffs(lane)) > fabs(mPath[DRV_L].offset - mPath[DRV_R].offset) / 2.0 + 1.0) {
      lane = DRV_LR;
    }
    // Returning to track from excursion or pits
    if (mDrvState == DRV_OFFTRACK || mDrvState == DRV_PIT) {
      if (fabs(mPath[DRV_L].offset) < fabs(mPath[DRV_R].offset)) {
        lane = DRV_L;
      } else {
        lane = DRV_R;
      }
    }
    // Make the lane change
    if (mDrvLane != lane) {
      driverMsg(2, "New lane:", lane);
      mDrvLane = lane;
      mLaneChange = true;
    }
  }
}


void TDriver::calcDrvState()
{
  if (statePit()) {
    setDrvState(DRV_PIT);
  } else if (stateStuck()) {
    setDrvState(DRV_STUCK);
  } else if (stateOfftrack()) {
    setDrvState(DRV_OFFTRACK);
  } else if (stateLetpass()) {
    setDrvState(DRV_LETPASS);
  } else {
    setDrvState(DRV_RACE);
    DriveLane lane = DRV_O;
    if (mTestLine == 1) {
      lane = DRV_L;
    }
    if (mTestLine == 2) {
      lane = DRV_R;
    }
    if (mTestLine == 3) {
      lane = DRV_LR;
    }
    if (overtakeOpponent()) {
      lane = overtakeStrategy();
    }
    setDrvLane(lane);
  }
}


void TDriver::calcTargetToMiddle()
{
  switch (mDrvState) {
    case DRV_RACE: {
      if (mDrvLane == DRV_LR) {
        mTargetToMiddle = (mPath[DRV_L].target.toMiddle + mPath[DRV_R].target.toMiddle) / 2.0;
      } else {
        mTargetToMiddle = mPath[mDrvLane].target.toMiddle;
      }
      // Special cases
      if (mDrvLane == DRV_L || mDrvLane == DRV_R)  {
        if (mSpeed < 10.0)  {
          mTargetToMiddle = SIGN(mTargetToMiddle) * (mTrack->width / 2.0);
        }
      }
      if (fabs(mOppSidedist) < 4.0) {
        if (mBorderdist > 1.0) {
          mTargetToMiddle -= 5.0 * SIGN(mOppSidedist) * (4.0 - fabs(mOppSidedist));
        } else {
          mTargetToMiddle = SIGN(mTargetToMiddle) * ((mTrack->width / 2.0) - 1.0);
        }
      }
      if (oCurrSimTime < 3.0) {
        mTargetToMiddle = mToMiddle;
      }
      break;
    }
    case DRV_STUCK: {
      break;
    }
    case DRV_OFFTRACK: {
      mTargetToMiddle = SIGN(mToMiddle) * ((mTrack->width / 2.0) - 1.0);
      break;
    }
    case DRV_PIT: {
      mTargetToMiddle = mPit->getPitOffset(mTargetFromstart);
      break;
    }
    case DRV_LETPASS: {
      if (mTargetToMiddle > 0.0) {
        mTargetToMiddle = mPath[DRV_L].target.toMiddle;
      } else {
        mTargetToMiddle = mPath[DRV_R].target.toMiddle;
      }
      break;
    }
  }
}


bool TDriver::overtakeOpponent()
{
  bool ovt = false;
  if (mOpp != NULL) {
    double dist = mOpp->mDist;
    double catchtime = oppCatchTime(mOpp);
    if (dist < 30.0 && dist > 1.0 && mOpp->borderdist > -1.0 && !(mDrivingFastExtended || mDangerSector)) {
      if ((mOvtSector && !mOpp->teammate)
      || catchtime < 3.0
      || mSpeed < 15.0
      || mOpp->car->_fuel == 0.0
      || mOpp->letpassaccel) {
        ovt = true;
      }
    }
    // if aside always overtake
    if (dist > -2.0 && dist <= 1.0) {
      ovt = true;
    }
    // Special case: if in front and on raceline stay there
    if ((dist < 0.0 && mDrvLane == DRV_O)) {
      ovt = false;
    }
  }
  // Store status
  if (ovt != mOvertake) {
    mOvertake = ovt;
    driverMsg(2, "mOvertake:", mOvertake);
  }
  return mOvertake;
}


DriveLane TDriver::overtakeStrategy()
{
  DriveLane lane = mDrvLane;
  // Normal overtaking
  if ((mOpp->mDist > 1.0 && mOpp->mDist < 10.0) || mSpeed > mOpp->speed + 10.0) {
    if (mOppLeftOfMeHyst) {
      if (fabs(mPath[DRV_R].local.toMiddle - mOpp->toMiddle) > 2.5) {
        lane = DRV_R;
      } else {
        lane = DRV_L;
      }
    } else {
      if (fabs(mPath[DRV_L].local.toMiddle - mOpp->toMiddle) > 2.5) {
        lane = DRV_L;
      } else {
        lane = DRV_R;
      }
    }
  }
  // Special cases
  if ((mOpp->backmarker || mOpp->letpassaccel) && (mOpp->mDist > 1.0 && mOpp->mDist < 20.0)) {
    if (mOppLeft && mOpp->mAngleToLeft && fabs(mOpp->mAngleToTrack) >= 0.04) {
      lane = DRV_R;
    } else if (!mOppLeft && !mOpp->mAngleToLeft && fabs(mOpp->mAngleToTrack) >= 0.04) {
      lane = DRV_L;
    }
  }
  // Slow or stuck
  if (mSpeed < 10.0 && mOpp->mDist > 1.0) {
    if (mOppLeft) {
      lane = DRV_R;
    } else {
      lane = DRV_L;
    }
  }
  // Always stay on your side if opponent aside
  if (mOpp->mDist <= 1.0) {
    if (mOppLeftOfMe) {
      lane = DRV_R;
    } else {
      lane = DRV_L;
    }
  }
  return lane;
}


void TDriver::updateStuck()
{
  if (mTenthTimer) {
    if (mWait) {
      mStuckcount = 0;
    }
    if (mStuck) {
      if (fabs(mSpeed) < 7.0) {
        if (mStuckcount++ > 20) {
          mStuckcount = 0;
          mStuck = false;
        }
      } else {
        mStuckcount = 0;
        mStuck = false;
      }
    } else if (fabs(mSpeed) < 0.5) {
      if (mStuckcount++ > 10) {
        mStuckcount = 0;
        mStuck = true;
      }
    } else {
      mStuckcount = 0;
    }
  }
}


bool TDriver::onCollision()
{
  mWait = false;
  mColl = false;
  if (mPointingToWall && fabs(mAngleToTrack) > 0.7) {
    if (mWalldist < brakeDist1(mSpeed, 0.0) && !mStuck) {
      mColl = true;
    }
  }
  for (int i = 0; i < mOpponents->nopponents; i++) {
    Opponent* opp = &mOpponents->opponent[i];
    if (opp->mDist > -5.0 && opp->mDist < 150.0
    && oppInDrivingDirection(opp)
    && oppInCollisionZone(opp)) {
      double frontfactor = 1.0;
      if (mBackmarkerInFrontOfTeammate || (mDrivingFastExtended && !opp->teammate)) {
        frontfactor = 2.0;
      }
      if (mOvtSector && !opp->teammate) {
        frontfactor = 0.5;
      }
      if (mSpeed < 5.0) {
        frontfactor = 0.2;
      }
      double brakedist = brakeDistToOpp(opp);
      if (brakedist > (opp->mDist - frontfactor * mFRONTCOLL_MARGIN)
      || opp->mDist < frontfactor * mFRONTCOLL_MARGIN
      || (mSpeed < -0.1 && distFromCenter(opp) < 5.0)) {
        mColl = true;
        mCollOppSpeed = opp->speed;
        mCollOppDist = opp->mDist;
      }
    }
    // is track free to enter
    if (opp->mDist > -100.0 && opp->mDist < 0.0
    && oppComingFastBehind(opp)
    && mBorderdist < -2.0 && mBorderdist > -5.0
    && mSpeed < 15.0
    && !(mPointingToWall && oCar->_gear >= 0)) {
      mColl = true;
      mWait = true;
    }
  }
  return mColl;
}


double TDriver::distFromCenter(Opponent* opp)
{
  if (opp->mDist < -5.0 || opp->mDist > mOVT_FRONTMARGIN) {
    return 100.0;
  }
  v2d opppos;
  opppos.x = opp->car->_pos_X;
  opppos.y = opp->car->_pos_Y;
  Straight mystraight(oCar->_pos_X, oCar->_pos_Y, sin(oCar->_yaw), -cos(oCar->_yaw));
  return mystraight.dist(opppos);
}


double TDriver::distToStraight(Opponent* opp)
{
  if (opp->mDist < -5.0 || opp->mDist > 150.0) {
    return 100.0;
  }
  v2d opppos;
  opppos.x = opp->car->_pos_X;
  opppos.y = opp->car->_pos_Y;
  v2d mypos;
  mypos.x = oCar->_pos_X;
  mypos.y = oCar->_pos_Y;
  v2d mydir;
  mydir.x = cos(oCar->_yaw);
  mydir.y = sin(oCar->_yaw);
  mydir.rotate(v2d(0,0), mTargetAngle);
  Straight mystraight(mypos, mydir);
  return mystraight.dist(opppos);
}


bool TDriver::oppComingFastBehind(Opponent* opp)
{
  if (opp->mDist > -1.0 || opp->speed < 20.0) {
    return false;
  }
  double diffspeed = MAX(0.01, opp->speed - mSpeed);
  double catchtime = opp->mDist / diffspeed;
  if (catchtime > -1.0 && catchtime < -0.5) {
    return true;
  }
  return false;
}


double TDriver::oppCatchTime(Opponent* opp)
{
  if (opp->mDist <= 0.0) {
    return 0.0;
  }
  double diffspeed = MAX(0.01, mSpeed - opp->speed);
  return opp->mDist / diffspeed;
}


bool TDriver::oppIsBehind(Opponent* opp)
{
  v2d opppos;
  opppos.x = oCar->_pos_X - opp->car->_pos_X;
  opppos.y = oCar->_pos_Y - opp->car->_pos_Y;
  v2d center;
  center.x = 0.0;
  center.y = 0.0;
  double alpha = oCar->_yaw - opppos.alpha(center);
  NORM_PI_PI(alpha);
  if (fabs(alpha) > PI / 2.0) {
    return true;
  }
  return false;
}


bool TDriver::oppInCollisionZone(Opponent* opp)
{
  double disttostraight = distToStraight(opp);
  double diffspeedmargin = diffSpeedMargin(opp);
  if (disttostraight < diffspeedmargin) {
    return true;
  }
  return false;
}


bool TDriver::oppInDrivingDirection(Opponent* opp)
{
  if ((oppIsBehind(opp) && mSpeed < -0.1) || (!oppIsBehind(opp) && mSpeed > 0.1)) {
    return true;
  }
  return false;
}


bool TDriver::oppNoDanger(Opponent* opp)
{
  if ((opp->borderdist < -1.0 && fabs(opp->speed) < 0.5 && mBorderdist > 0.0 && fabs(opp->mDist) > 1.0)
  || opp->car->_state & RM_CAR_STATE_PIT) {
    return true;
  }
  return false;
}


double TDriver::diffSpeedMargin(Opponent* opp)
{
  double speeddiff = MAX(0.0, mSpeed - opp->speed);
  double oppangle = oppAngle(opp);
  double angle = 0.0;
  if ((oppangle < 0.0 && mOppLeftOfMe) || (oppangle > 0.0 && !mOppLeftOfMe)) {
    angle = MIN(0.3, fabs(oppangle));
  }
  double factor = MAX(0.1, 1.0 * angle);
  if (mDrivingFast) {
    factor *= 3.0;
  }
  double diffspeedmargin = MIN(15.0, MAX(3.0, 2.0 + 2 * sin(fabs(oppangle)) + factor * speeddiff));
  if (opp->teammate) {
    diffspeedmargin += 1.0;
  }
  if (mDrivingFastExtended) {
    diffspeedmargin += 1.0;
  }
  if (mDangerSector && mSpeed > 15.0) {
    diffspeedmargin += 3.0;
  }
  if (mSpeed < 5.0 || oppNoDanger(opp)) {
    diffspeedmargin = 2.0;
  }
  return diffspeedmargin;
}


double TDriver::oppAngle(Opponent* opp)
{
  double oppangle = opp->car->_yaw - oCar->_yaw;
  NORM_PI_PI(oppangle);
  if (fabs(oppangle) > PI / 2.0) {
    oppangle -= PI;
    NORM_PI_PI(oppangle);
    oppangle = -oppangle;
  }
  return oppangle;
}

// Compute aerodynamic downforce coefficient CA
void TDriver::initCa()
{
  char* WheelSect[4] = {(char*)SECT_FRNTRGTWHEEL, (char*)SECT_FRNTLFTWHEEL, (char*)SECT_REARRGTWHEEL, (char*)SECT_REARLFTWHEEL};
  double rearwingarea = GfParmGetNum(oCar->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*)NULL, 0.0);
  double rearwingangle = GfParmGetNum(oCar->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*)NULL, 0.0);
  double frontwingarea = GfParmGetNum(oCar->_carHandle, SECT_FRNTWING, PRM_WINGAREA, (char*)NULL, 0.0);
  double frontwingangle = GfParmGetNum(oCar->_carHandle, SECT_FRNTWING, PRM_WINGANGLE, (char*)NULL, 0.0);
  double frontclift = GfParmGetNum(oCar->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*)NULL, 0.0);
  double rearclift = GfParmGetNum(oCar->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0);
  double rearwingca = 1.23 * rearwingarea * sin(rearwingangle);
  double frntwingca = 1.23 * frontwingarea * sin(frontwingangle);
  double cl = frontclift + rearclift;
  double h = 0.0;
  for (int i = 0; i < 4; i++) {
    h += GfParmGetNum(oCar->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20f);
  }
  h*= 1.5; h = h * h; h = h * h; h = 2.0 * exp(-3.0 * h);
  mCA = h * cl + 4.0 * (frntwingca+rearwingca);
  double FCA = h * frontclift + 4.0 * frntwingca;
  double RCA = h * rearclift + 4.0 * rearwingca;
  fprintf(stderr,"%s: CA=%.3f FCA=%.3f RCA=%.3f\n", oCar->_name, mCA, FCA, RCA);
}


// Compute aerodynamic drag coefficient CW.
void TDriver::initCw()
{
  double cx = GfParmGetNum(oCar->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.0f);
  double frontarea = GfParmGetNum(oCar->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0f);
  mCW = 0.645f * cx * frontarea;
}


// Init car specifications
void TDriver::readSpecs()
{
  mWHEELBASE = GfParmGetNum(oCar->_carHandle, SECT_FRNTAXLE, PRM_XPOS, (char*)NULL, 0.0) - GfParmGetNum(oCar->_carHandle, SECT_REARAXLE, PRM_XPOS, (char*)NULL, 0.0);
  mCARMASS = GfParmGetNum(oCar->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
  mBRAKEPRESS = GfParmGetNum(oCar->_carHandle, SECT_BRKSYST, PRM_BRKPRESS, NULL, 20000000.0) / 1000.0;
}


void TDriver::readPrivateSection()
{
  mBRAKEDISTFACTOR = GfParmGetNum(oCar->_carHandle, "private", "brakedistfactor", (char*)NULL, 1.0);
  mBRAKEFORCEFACTOR = GfParmGetNum(oCar->_carHandle, "private", "brakeforcefactor", (char*)NULL, 1.0);
  mFUELWEIGHTFACTOR = GfParmGetNum(oCar->_carHandle, "private", "fuelweightfactor", (char*)NULL, 1.0);
  mPITENTRYMARGIN = GfParmGetNum(oCar->_carHandle, "private", "pitentrymargin", (char*)NULL, 0.0);
  mPITENTRYSPEED = GfParmGetNum(oCar->_carHandle, "private", "pitentryspeed", (char*)NULL, 25.0);
  mPITEXITSPEED = GfParmGetNum(oCar->_carHandle, "private", "pitexitspeed", (char*)NULL, 25.0);
  mPITDAMAGE = GfParmGetNum(oCar->_carHandle, "private", "pitdamage", (char*)NULL, 0);
  mEXTMARGIN = GfParmGetNum(oCar->_carHandle, "private", "extmargin", (char*)NULL, 1.5);
  mINTMARGIN = GfParmGetNum(oCar->_carHandle, "private", "intmargin", (char*)NULL, 1.5);

  int i = 0;
  char attname[20];

  // Overtake sectors
  i = 0;
  OvtInfo ovtinfo;
  sprintf(attname, "overtakestart%d", i);
  while ((ovtinfo.start = GfParmGetNum(oCar->_carHandle, "private", attname, (char*)NULL, 0.0)) > 0.0) {
    sprintf(attname, "overtakestop%d", i);
    ovtinfo.stop = GfParmGetNum(oCar->_carHandle, "private", attname, (char*)NULL, 0.0);
    mOvtInfo.push_back(ovtinfo);
    i++;
    sprintf(attname, "overtakestart%d", i);
  }

  // Danger sectors
  i = 0;
  DangerInfo dangerinfo;
  sprintf(attname, "dangerstart%d", i);
  while ((dangerinfo.start = GfParmGetNum(oCar->_carHandle, "private", attname, (char*)NULL, 0.0)) > 0.0) {
    sprintf(attname, "dangerstop%d", i);
    dangerinfo.stop = GfParmGetNum(oCar->_carHandle, "private", attname, (char*)NULL, 0.0);
    mDangerInfo.push_back(dangerinfo);
    i++;
    sprintf(attname, "dangerstart%d", i);
  }
}


// Antilocking filter for brakes
double TDriver::filterABS(double brake)
{
  double ABS_MINSPEED = 3.0;      // [m/s]
  double ABS_SLIP = 0.9;
  double slip = 0.0;

  if (mSpeed < ABS_MINSPEED) return brake;

  int i;
  for (i = 0; i < 4; i++) {
    slip += oCar->_wheelSpinVel(i) * oCar->_wheelRadius(i) / mSpeed;
  }
  slip = slip / 4.0;
  if (slip < ABS_SLIP) {
    if (mAbsFactor > 0.1) mAbsFactor -= 0.1;
    brake *= mAbsFactor;
  } else {
    if (mAbsFactor < 0.9) mAbsFactor += 0.1;
  }
  return brake;
}


// TCL filter for accelerator
double TDriver::filterTCL(double accel)
{
  if (mDrvState == DRV_RACE) {
    return accel;
  }
  const double TCL_SLIP = 2.0;
  double slip = filterTCL_RWD() - mSpeed;
  if (slip > TCL_SLIP) {
    if (mTclFactor > 0.1) mTclFactor -= 0.1;
    accel *= mTclFactor;
  } else {
    if (mTclFactor < 0.9) mTclFactor += 0.1;
  }
  return accel;
}


// TCL filter plugin for rear wheel driven cars.
double TDriver::filterTCL_RWD()
{
  return (oCar->_wheelSpinVel(REAR_RGT) + oCar->_wheelSpinVel(REAR_LFT)) *
      oCar->_wheelRadius(REAR_LFT) / 2.0f;
}


// Compute the needed distance to brake.
double TDriver::brakeDist(double radius, double speed, double allowedspeed)
{
  if (speed > allowedspeed) {
    double v1sqr = speed * speed;
    double v2sqr = allowedspeed * allowedspeed;
    double c = mMu * Gravity;
//    double c = sqrt(mMu * Gravity * mMu * Gravity - (v1sqr / radius) * (v1sqr / radius));
    double d = (mCA * mMu + mCW) / mMass;
    return (-log((c + v2sqr * d)/(c + v1sqr * d)) / (2.0 * d));
  }
  return 0.0;
}


double TDriver::brakeDist1(double speed, double allowedspeed)
{
  if (speed > allowedspeed) {
    double allowedspeedsqr = allowedspeed * allowedspeed;
    return (mMass * (speed * speed - allowedspeedsqr) / 2.0) / (mMu * Gravity * mMass + allowedspeedsqr * (mCA * mMu + mCW));
  }
  return 0.0;
}


double TDriver::brakeDistToOpp(Opponent* opp)
{
  double brakedist = brakeDist1(mSpeed, opp->speed);
  if (opp->mDist > 0.0 && opp->mDist < 15.0) {
    brakedist -= (opp->mDist / 15.0) * brakedist * opp->speed / mSpeed;
  } else if (opp->mDist >= 15.0) {
    brakedist -= brakedist * opp->speed / mSpeed;
  } else {
    brakedist = -DBL_MAX;
  }
  return brakedist;
}


double TDriver::fromStart(double fromstart)
{
  if (fromstart > mTrack->length) {
    return fromstart - mTrack->length;
  } else if (fromstart < 0.0) {
    return fromstart + mTrack->length;
  }
  return fromstart;
}


void TDriver::updateDanSectorId()
{
  for (int i = 0; i < (int)vSect.size(); i++) {
    if (mFromStart > vSect[i].fromstart
    && mFromStart < vSect[i].fromstart + 3.0) {
      mDanSectorId = i;
      driverMsg(1, "sector: ", i);
    }
  }
}


void TDriver::learnSpeedFactors()
{
  if (!mLearning || oSituation->_raceType != RM_TYPE_PRACTICE) {
    return;
  }

  int nrOfSectors = (int)vSect.size();

  if (oCar->_laps > 1 && mLearnLap) {
    // Get offtrack sector
    bool offtrack = mBorderdist < -1.5;
    int damagediff = oCar->_dammage - mLastDamage;
    mLastDamage = oCar->_dammage;
    if (offtrack) {
      mOfftracksector = mDanSectorId;
      std::cout << "offtrack: " << mBorderdist + 1.5 << std::endl;
    }
    if (damagediff > 3 && mBorderdist < 0.0) {
      mOfftracksector = mDanSectorId;
      std::cout << "damagediff: " << damagediff << std::endl;
    }
    // Get sector times
    for (int i = 0; i < nrOfSectors; i++) {
      if (mFromStart > vSect[i].fromstart
      && mFromStart < vSect[i].fromstart + 3.0) {
        if (mLearnSectTime) {
          mLearnSectTime = false;
          if (i > 0) {
            vSect[i-1].time = oCurrSimTime - mSectorTime;
            if (mOfftracksector == i-1) {
              vSect[i-1].time = 10000;
              std::cout << " offtracksector: " << mOfftracksector << std::endl;
              mOfftracksector = -1;
            }
          }
          if (i == 0 && oCar->_laps == 3) {
            vSect[nrOfSectors-1].time = oCurrSimTime - mSectorTime;
            if (mOfftracksector == nrOfSectors-1) {
              vSect[nrOfSectors-1].time = 10000;
              std::cout << " offtracksector: " << mOfftracksector << std::endl;
              mOfftracksector = -1;
            }
          }
          mSectorTime = oCurrSimTime;
          driverMsg(0, "sector: ", i);
        }
      } else if (mFromStart > vSect[i].fromstart + 3.0
      && mFromStart < vSect[i].fromstart + 6.0) {
        mLearnSectTime = true;
      }
    }
  }

  // Update speed factors
  if (mFromStart > 3.0 && mFromStart < 6.0 && oCar->_laps == 3 && mLearnLap) {
    mLearnLap = false;
    double bestlap = 0.0;
    double lastlap = 0.0;
    for (int i = 0; i < nrOfSectors; i++) {
      bestlap += vSect[i].besttime;
      lastlap += vSect[i].time;
    }
    for (int i = 0; i < nrOfSectors; i++) {
      std::cout << "S:" << i << " l:" << vSect[i].learned << " fs:" << vSect[i].fromstart << " t:" << vSect[i].time << " bt:" << vSect[i].besttime << " sf:" << vSect[i].speedfactor << " bsf:" << vSect[i].bestspeedfactor << std::endl;
      double delta = 0.04;
      if (vSect[i].speedfactor >= 1.4) {
        delta = 0.02;
      }
      int prev_i = (i > 0) ? i-1 : nrOfSectors-1;
      if (vSect[i].time > 0 && !vSect[i].learned) {
        if (vSect[i].besttime > vSect[i].time - 0.02 && vSect[i].bestspeedfactor < 2.0) {
          if (vSect[i].besttime > vSect[i].time - 0.001) {
            vSect[i].bestspeedfactor = vSect[i].speedfactor;
            vSect[i].besttime = vSect[i].time;
          }
        } else {
          if (vSect[i].time == 10000) {
            if (!vSect[prev_i].learned) {
              vSect[prev_i].speedfactor -= delta;
              vSect[prev_i].bestspeedfactor = vSect[prev_i].speedfactor;
            }
          }
          vSect[i].speedfactor = vSect[i].bestspeedfactor;
          vSect[i].learned = 1;
          std::cout << " learned: " << i << std::endl;
        }
      } else if (vSect[i].time == 10000) {
        if (!vSect[prev_i].learned) {
          vSect[prev_i].speedfactor -= 2 * delta;
          vSect[prev_i].bestspeedfactor = vSect[prev_i].speedfactor;
          vSect[prev_i].learned = 1;
          std::cout << " learned: " << prev_i << std::endl;
        }
      }
      if (vSect[i].learned == 0) {
        vSect[i].speedfactor += delta;
      }
    }
    std::cout << "lap: " << oCar->_laps -1 << " time total: " << lastlap << " best: " << bestlap << std::endl;
    saveFile();
  }
}


void TDriver::getSpeedFactors()
{
  for (int i = 0; i < (int)vSect.size(); i++) {
    if (mFromStart > vSect[i].fromstart && mFromStart < vSect[i].fromstart + 3.0) {
      mSpeedfactor = vSect[i].speedfactor;
    }
  }
}


void TDriver::getOvtSectors()
{
  for (int i = 0; i < (int)mOvtInfo.size(); i++) {
    if (mFromStart > mOvtInfo[i].start  && mFromStart < mOvtInfo[i].start + 3.0) {
      mOvtSector = true;
      driverMsg(2, "mOvtSector", mOvtSector);
    }
    if (mFromStart > mOvtInfo[i].stop  && mFromStart < mOvtInfo[i].stop + 3.0) {
      mOvtSector = false;
      driverMsg(2, "mOvtSector", mOvtSector);
    }
  }
}


void TDriver::getDangerSectors()
{
  for (int i = 0; i < (int)mDangerInfo.size(); i++) {
    if (mFromStart > mDangerInfo[i].start  && mFromStart < mDangerInfo[i].start + 3.0) {
      mDangerSector = true;
      driverMsg(2, "mDangerSector", mDangerSector);
    }
    if (mFromStart > mDangerInfo[i].stop  && mFromStart < mDangerInfo[i].stop + 3.0) {
      mDangerSector = false;
      driverMsg(2, "mDangerSector", mDangerSector);
    }
  }
}


void TDriver::updatePathCar(int path)
{
  if (!mDanPath->getDanPos(path, mFromStart, mPath[path].danpoint)) {
    #ifdef DRIVER_DEBUG
    exit(1);
    #endif
  }
  if (!mDanPath->getLocalPos(oCar->_trkPos.seg, mPath[path].danpoint, &mPath[path].local)) {
    driverMsg(0, "TDriver::updatePathCar", 1152);
    #ifdef DRIVER_DEBUG
    exit(1);
    #endif
  }
}


void TDriver::updatePathTarget(int path)
{
  mTargetFromstart = fromStart(mFromStart + mLOOKAHEAD_CONST + 0.3 * mSpeed);
  DanPoint target;
  if (!mDanPath->getDanPos(path, mTargetFromstart, target)) {
#ifdef DRIVER_DEBUG
    exit(1);
#endif
  }
  if (!mDanPath->getLocalPos(oCar->_trkPos.seg, target, &mPath[path].target)) {
    driverMsg(0, "TDriver::updatePathTarget", 1172);
#ifdef DRIVER_DEBUG
    exit(1);
#endif
  }
}


void TDriver::updatePathOffset(int path)
{
  mPath[path].offset = mPath[path].local.toMiddle - mToMiddle;
}


void TDriver::updatePathSpeed(int path)
{
  mPath[path].maxspeed = mSpeedfactor * getMaxSpeed(mPath[path].danpoint);
}


void TDriver::updateDrivingFast()
{
  bool oldstate = mDrivingFast;
  double maxspeed = MIN(mPath[DRV_L].maxspeed, mPath[DRV_R].maxspeed);
  mDrivingFast = mSpeed > 0.7 * maxspeed;
  // Extended mDrivingFast for some meters
  mDrivingFastExtended = mSpeed > 0.6 * maxspeed;
  if (mDrivingFast < oldstate) {
    mEndOfDrivingFast = mFromStart;
  }
  if (mFromStart >= mEndOfDrivingFast && mFromStart < fromStart(mEndOfDrivingFast + 50.0)) {
    mDrivingFastExtended = true;
  }
  // Message
  if (mDrivingFastExtended) {
    driverMsg(3, "mDrivingFastExtended", mDrivingFastExtended);
  }
}


void TDriver::updateCatchedRaceLine()
{
  if (mDrvState == DRV_RACE && mDrvLane < DRV_LR && !mLaneChange) {
    if (fabs(mPath[mDrvLane].offset) < 1.0) {
      if (mCatchedRaceLineTime > 3.0) {
        mCatchedRaceLine = true;
      } else if (mTenthTimer) {
        mCatchedRaceLineTime += 0.1;
        driverMsg(3, "mCatchedRaceLineTime", mCatchedRaceLineTime);
      }
    } else if (!mCatchedRaceLine) {
      mCatchedRaceLineTime = 0.0;
    }
  } else {
    mCatchedRaceLine = false;
    mCatchedRaceLineTime = 0.0;
  }
}


void TDriver::calcMaxspeed()
{
  // Calc slowspeed
  double slowspeed;
  if (mToMiddle > 0.0) {
    slowspeed = 0.8 * mPath[DRV_L].maxspeed;
  } else {
    slowspeed = 0.8 * mPath[DRV_R].maxspeed;
  }
  // Set speed
  switch (mDrvState) {
    case DRV_RACE: {
      double maxspeed;
      if (mDrvLane == DRV_LR) {
        maxspeed = MIN(mPath[DRV_L].maxspeed, mPath[DRV_R].maxspeed);
      } else {
        maxspeed = mPath[mDrvLane].maxspeed;
      }
      mMaxspeed = MAX((1.0 - fabs(laneOffs(mDrvLane)) / 5.0)  * maxspeed, slowspeed);
      if (mDrvLane == DRV_O && mCatchedRaceLine) {
        mMaxspeed = maxspeed;
      }
      // Special cases
      if (mOppDist >= 0.0 && mOppDist < 20.0) {
        if (mDangerSector) {
          mMaxspeed *= 0.96;
        } else {
          mMaxspeed *= 0.98;
        }
      }
      if (fabs(mAngleToTrack) > 1.0 && !mStuck) {
        mMaxspeed = 10.0;
      }
      break;
    }
    case DRV_STUCK: {
      mMaxspeed = 7.0;
      break;
    }
    case DRV_OFFTRACK: {
      mMaxspeed = 7.0;
      break;
    }
    case DRV_PIT: {
      mMaxspeed = getPitSpeed();
      break;
    }
    case DRV_LETPASS: {
      mMaxspeed = slowspeed;
      break;
    }
    default: {
      break;
    }
  }
}


void TDriver::limitSteerAngle(double& targetangle)
{
  double v2 = mSpeed * mSpeed;
  double rmax = v2 / (mMu * (Gravity + v2 * mCA / mMass));
  double maxangle = atan(mWHEELBASE / rmax);
  if (mDrvState == DRV_OFFTRACK) {
    maxangle *= 1.0;
  } else if (mCatchedRaceLine) {
    maxangle *= 9.0;
  } else {
    maxangle *= 3.0;
  }
  mMaxSteerAngle = false;
  if (fabs(targetangle) > maxangle) {
    targetangle = SIGN(targetangle) * maxangle;
    NORM_PI_PI(targetangle);
    mMaxSteerAngle = true;
    driverMsg(3, "maxangle", maxangle);
  }
}


void TDriver::calcGlobalTarget()
{
  mPath[DRV_O].target.toMiddle = mTargetToMiddle;
  tdble x, y;
  RtTrackLocal2Global(&mPath[DRV_O].target, &x, &y, TR_TOMIDDLE);
  mGlobalTarget.x = x;
  mGlobalTarget.y = y;
}


void TDriver::calcTargetAngle()
{
  mTargetAngle = mGlobalCarPos.alpha(mGlobalTarget) - oCar->_yaw;
  NORM_PI_PI(mTargetAngle);
}


void TDriver::controlSpeed(double& accelerator, double maxspeed)
{
  // Set parameters
  mSpeedController.m_p = 0.02;
  mSpeedController.m_d = 0.0;
  // Run controller
  double speeddiff =  maxspeed - mSpeed;
  accelerator += mSpeedController.sample(speeddiff);
  if (accelerator > 1.0) {
    accelerator = 1.0;
  }
  // Debug output
  if (fabs(speeddiff) < 2.0) {
    driverMsg(3, "speed contr diff: ", speeddiff);
  }
}


void TDriver::updateAttackAngle()
{
  double velAng = atan2(oCar->_speed_Y, oCar->_speed_X);
  mAttackAngle = velAng - oCar->_yaw;
  NORM_PI_PI(mAttackAngle);
  if (mSpeed < 1.0) {
    mAttackAngle = 0.0;
  }
}


bool TDriver::controlAttackAngle(double& targetangle)
{
  bool controling;
  if (fabs(mAttackAngle) > 0.15
  || mDrvState == DRV_OFFTRACK) {
    driverMsg(3, "attack angle", mAttackAngle);
    mAttackAngleController.m_d = 9.0;
    mAttackAngleController.m_p = 0.9;
    if (fabs(mAttackAngle) > 0.3) {
      mAttackAngleController.m_p = 1.9;
    } else if (fabs(mAttackAngle) > 0.25) {
      mAttackAngleController.m_p = 1.4;
    }
    targetangle += mAttackAngleController.sample(mAttackAngle);
    NORM_PI_PI(targetangle);
    controling = true;
  } else {
    mAttackAngleController.sample(mAttackAngle);
    controling = false;
  }
  return controling;
}


void TDriver::controlOffset(double& targetangle)
{
  // Set parameters
  if (mCatchedRaceLine && mDrvLane == DRV_O) {
    mOffsetController.m_p = 0.06;
    mOffsetController.m_d = 1.0;
  } else {
    mOffsetController.m_p = 0.01;
    mOffsetController.m_d = 0.6;
  }

  // Run controller
  double offs = laneOffs(mDrvLane);
  if (mCatchedRaceLine) {
    targetangle += mOffsetController.sample(offs);
    NORM_PI_PI(targetangle);
  } else {
    mOffsetController.sample(offs, 0.0);
  }
  // Debug output
  if (fabs(offs) > 1.0) {
    driverMsg(3, "offset", offs);
  }
}


void TDriver::controlYawRate(double& targetangle)
{
  if (mDrvState == DRV_RACE && mDrvLane < DRV_LR) {
    double AvgK = 1 / mPath[mDrvLane].danpoint.radius;
    // Control rotational velocity.
    double Omega = mSpeed * AvgK;
    double yawratediff = Omega - oCar->_yaw_rate;
    if (fabs(yawratediff) > 0.2) {
      driverMsg(3, "yaw rate diff", yawratediff);
      targetangle += 0.09 * (Omega - oCar->_yaw_rate);
      NORM_PI_PI(targetangle);
    }
  }
}


bool TDriver::hysteresis(bool lastout, double in, double hyst)
{
  if (lastout == false) {
    if (in > hyst) {
      return true;
    } else {
      return false;
    }
  } else {
    if (in < -hyst) {
      return false;
    } else {
      return true;
    }
  }
}


double TDriver::getFuel(double dist)
{
  double fueltoend = dist * mFuelPerMeter;
  int pitstops = int(floor(fueltoend / mTankvol));
  double stintfuel = fueltoend / (pitstops + 1) + 5.0;
  if (pitstops && (stintfuel / mTankvol > 0.8)) {
    stintfuel = mTankvol;
  }
  double fuel = MAX(MIN(stintfuel, mTankvol), 0.0);
  return fuel;
}


void TDriver::saveFile()
{
  char dirname[256];
  sprintf(dirname, "%s/drivers/%s/tracks/",GetLocalDir() ,RobotName);
#ifdef TARGET_TORCS
  if (GfCreateDir(strdup(dirname)) == GF_DIR_CREATED) {
#else
  if (GfDirCreate(strdup(dirname)) == GF_DIR_CREATED) {
#endif
    saveSectorSpeeds();
  } else {
    std::cout << "Error saveFile: unable to create user dir" << std::endl;
  }
}


void TDriver::saveSectorSpeeds()
{
  char filename[256];
  sprintf(filename, "%sdrivers/%s/tracks/%s.csv", GetLocalDir(), RobotName, mTrack->internalname);
  std::ofstream myfile;
  myfile.open (filename);
  for (int i = 0; i < (int)vSect.size(); i++) {
    if (i > 0) {
      myfile << std::endl;
    }
    myfile << vSect[i].sector << std::endl;
    myfile << vSect[i].fromstart << std::endl;
    myfile << vSect[i].speedfactor << std::endl;
    myfile << vSect[i].time << std::endl;
    myfile << vSect[i].bestspeedfactor << std::endl;
    myfile << vSect[i].besttime << std::endl;
    myfile << vSect[i].learned;
    myfile << std::endl;
  }
  myfile.close();
}


bool TDriver::readSectorSpeeds()
{
  char filename[256];
  if (mLearning) {
    sprintf(filename, "%sdrivers/%s/tracks/%s.csv", GetLocalDir(), RobotName, mTrack->internalname);
  } else {
    sprintf(filename, "%sdrivers/%s/tracks/%s.csv", GetDataDir(), RobotName, mTrack->internalname);
  }

  int sector = 0;
  DanSector sect;
  std::string line;
  std::ifstream myfile(filename);
  if (myfile.is_open()) {
    while (myfile.good()) {
      getline (myfile, line);
      sect.sector = atof(line.c_str());
      getline (myfile, line);
      sect.fromstart = atof(line.c_str());
      getline (myfile, line);
      sect.speedfactor = atof(line.c_str());
      getline (myfile, line);
      sect.time = atof(line.c_str());
      getline (myfile, line);
      sect.bestspeedfactor = atof(line.c_str());
      getline (myfile, line);
      sect.besttime = atof(line.c_str());
      getline (myfile, line);
      sect.learned = atof(line.c_str());
      getline (myfile, line);
      if (mLearning) {
        std::cout << "S:" << sector << " l:" << sect.learned << " fs:" << sect.fromstart << " t:" << sect.time << " bt:" << sect.besttime << " sf:" << sect.speedfactor << " bsf:" << sect.bestspeedfactor << std::endl;
      }
      vSect.push_back(sect);
      sector++;
    }
    myfile.close();
    return true;
  } else {
    std::cout << "readSectorSpeeds(): no csv file found -> enable learning mode first" << std::endl;
    return false;
  }
}


void TDriver::driverMsg(int priority, std::string desc, double value)
{
  if (priority <= mDriverMsgLevel) {
    std::cout << (int)mFromStart << "m " << oCar->_name << " s:" << mDrvState << " l:" << mDrvLane << " " << desc << " " << value << std::endl;
  }
}
