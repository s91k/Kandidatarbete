/***************************************************************************

    file                 : driver.h
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

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "torcs_or_sd.h"

#include "globaldefinitions.h"

#ifdef TARGET_TORCS
#include "robotname.h"
#endif

#ifdef TARGET_SPEEDDREAMS
#include "basedriver.h"
#endif

#include <fstream>
#include "d_linalg.h"
#include "opponent.h"
#include "pit.h"
#include "danpath.h"
#include "pidcontroller.h"


class PathInfo {
  public:
  DanPoint danpoint;
  tTrkLocPos local;
  tTrkLocPos target;
  double maxspeed;
  double offset;
};


class OvtInfo {
  public:
  double start;
  double stop;
};


class DangerInfo {
  public:
  double start;
  double stop;
};


enum DriveState {DRV_RACE, DRV_STUCK, DRV_OFFTRACK, DRV_PIT, DRV_LETPASS};
enum DriveLane {DRV_O, DRV_L, DRV_R, DRV_LR};


//==========================================================================*
// Declaration of class TDriver
//--------------------------------------------------------------------------*
#ifdef TARGET_TORCS
class TDriver {
  public:
  TDriver(int index);
  ~TDriver();
#endif

#ifdef TARGET_SPEEDDREAMS
class TDriver : public TBaseDriver {
  public:
  TDriver();
  ~TDriver();
  bool IsStuck();
  void Unstuck();
  void Update(PtCarElt Car, PSituation Situation);
#endif

  void InitTrack(PTrack Track, PCarHandle CarHandle, PCarSettings *CarParmHandle, PSituation Situation);
  void NewRace(PtCarElt Car, PSituation Situation);
  void Drive();
  int PitCmd();
  void EndRace();
  void Shutdown();

  private:
  // Utility functions
#ifdef TARGET_TORCS
  void clearControls();
  void updateTime();
#endif
  void updateTimer();
  void updateBasics();
  void updateOpponents();
  void updatePath();
  void setControls();
  double getMaxSpeed(DanPoint danpoint);
  double getPitSpeed();
  double curveSpeed(double radius);
  double brakeSpeed1(double nextdist, double nextspeed);
  double brakeSpeed(double speed, double radius, double nextdist, double nextradius);
  double getBrake(double maxspeed);
  double getAccel(double maxspeed);
  double getSteer();
  int getGear();
  double getClutch();
  bool stateStuck();
  bool stateOfftrack();
  bool statePit();
  bool stateLetpass();
  void setDrvState(DriveState state);
  double laneOffs(DriveLane lane);
  void setDrvLane(DriveLane lane);
  void calcDrvState();
  void calcTargetToMiddle();
  bool overtakeOpponent();
  DriveLane overtakeStrategy();
  bool changeOvtSide();
  void updateStuck();
  bool onCollision();
  double distToStraight(Opponent* opp);
  double distFromCenter(Opponent* opp);
  bool oppComingFastBehind(Opponent* opp);
  double oppCatchTime(Opponent* opp);
  bool oppIsBehind(Opponent* opp);
  bool oppInCollisionZone(Opponent* opp);
  bool oppInDrivingDirection(Opponent* opp);
  bool oppNoDanger(Opponent* opp);
  double diffSpeedMargin(Opponent* opp);
  double oppAngle(Opponent* opp);
  double brakeDist(double radius, double speed, double allowedspeed);
  double brakeDist1(double speed, double allowedspeed);
  double brakeDistToOpp(Opponent* opp);
  double fromStart(double fromstart);
  void updateDanSectorId();
  void learnSpeedFactors();
  void getSpeedFactors();
  void getOvtSectors();
  void getDangerSectors();
  void updatePathCar(int line);
  void updatePathTarget(int line);
  void updatePathOffset(int line);
  void updatePathSpeed(int line);
  void updateDrivingFast();
  void updateCatchedRaceLine();
  void calcMaxspeed();
  void calcTarget();
  void limitSteerAngle(double& targetangle);
  void calcGlobalTarget();
  void calcTargetAngle();
  double filterABS(double brake);
  double filterTCL(double accel);
  double filterTCL_RWD();
  void initCa();
  void initCw();
  void readSpecs();
  void readPrivateSection();
  void controlSpeed(double& accelerator, double speed);
  void updateAttackAngle();
  bool controlAttackAngle(double& targetangle);
  void controlOffset(double& targetangle);
  void controlYawRate(double& targetangle);
  bool hysteresis(bool lastout, double in, double hyst);
  double getFuel(double dist);
  void saveFile();
  void saveSectorSpeeds();
  bool readSectorSpeeds();
  void driverMsg(int priority, std::string desc, double value);
  // Per robot global data
#ifdef TARGET_TORCS
  char* RobotName;
  tSituation* oSituation;
  tCarElt* oCar;        // pointer to tCarElt struct
  double oCurrSimTime;
#endif
  PTrack mTrack;
  DanPath* mDanPath;
  Opponents* mOpponents;  // the container for opponents
  Opponent* mOpp;         // relevant opponent for calculations
  Opponent* mOppNear;
  Opponent* mOppNear2;
  Opponent* mOppBack;
  Opponent* mOppLetPass;
  double mOppDist;
  double mOppSidedist;
  bool mOppAside;
  bool mOppLeft;
  bool mOppLeftHyst;
  bool mOppLeftOfMe;
  bool mOppLeftOfMeHyst;
  bool mOppInFrontspace;
  bool mBackmarkerInFrontOfTeammate;
  bool mTwoOppsAside;
  bool mLearning;
  int mTestLine;
  int mDriverMsgLevel;
  Pit* mPit;
  double mTankvol;
  double mFuelPerMeter;
  double mMu;    // friction coefficient
  double mMass;  // mass of car + fuel
  double mSpeed;
  double mClutchtime;
  int mPrevgear;
  double mAttackAngle;
  double mSectorTime;
  double mOldTimer;
  bool mTenthTimer;
  bool mStuck;
  int mStuckcount;
  DriveState mDrvState;
  DriveLane mDrvLane;
  bool mStateChange;
  bool mLaneChange;
  bool mOvertake;
  double mFriction;
  double mCentifugal;
  double mBrakeFriction;
  double mBrakeforce;
  double mBorderdist;
  int mOfftracksector;
  bool mOnLeftSide;
  double mAngleToTrack;
  bool mAngleToLeft;
  bool mPointingToWall;
  double mWalldist;
  int mLastDamage;
  double mAccel;
  double mMaxspeed;
  std::vector <DanSector> vSect;
  int mDanSectorId;
  double mSpeedfactor;
  std::vector <OvtInfo> mOvtInfo;
  bool mOvtSector;
  std::vector <DangerInfo> mDangerInfo;
  bool mDangerSector;
  PathInfo mPath[3];
  bool mDrivingFast;
  double mEndOfDrivingFast;
  bool mDrivingFastExtended;
  bool mLearnSectTime;
  bool mLearnLap;
  double mFromStart;
  double mToMiddle;
  double mTargetFromstart;
  double mTargetToMiddle;
  double mOldTargetToMiddle;
  double mPrevTargetdiff;
  double mTargetAngle;
  bool mMaxSteerAngle;
  v2d mGlobalCarPos;
  v2d mGlobalTarget;
  bool mCatchedRaceLine;
  double mCatchedRaceLineTime;
  double mAbsFactor;
  double mTclFactor;
  bool mColl;
  double mCollOppSpeed;
  double mCollOppDist;
  bool mWait;
  double mSlipSideFront;
  double mSlipSideRear;
  PidController mSpeedController;
  PidController mAttackAngleController;
  PidController mOffsetController;
  // Data that should stay constant after first initialization
  double mWHEELBASE;  // wheelbase of the car
  double mCARMASS;    // mass of the car only
  double mCA;         // aerodynamic downforce coefficient
  double mCW;         // aerodynamic drag coefficient
  double mBRAKEPRESS;
  double mBRAKEDISTFACTOR;
  double mBRAKEFORCEFACTOR;
  double mFUELWEIGHTFACTOR;
  double mPITENTRYMARGIN;
  double mPITENTRYSPEED;
  double mPITEXITSPEED;
  double mPITDAMAGE;
  double mEXTMARGIN;
  double mINTMARGIN;
  // Class constants
  double mLOOKAHEAD_CONST;
  double mFRONTCOLL_MARGIN;
  double mOVT_FRONTSPACE;
  double mOVT_FRONTMARGIN;
};

#endif // _DRIVER_H_
