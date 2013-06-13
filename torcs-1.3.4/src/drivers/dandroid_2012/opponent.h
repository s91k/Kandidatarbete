/***************************************************************************

    file                 : opponent.h
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

#ifndef _OPPONENT_H_
#define _OPPONENT_H_

#include "globaldefinitions.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "d_linalg.h"


/* Opponent maintains the data for one opponent */
class Opponent {
  public:
    Opponent();
    void init(PTrack t, PSituation s, PCarElt c, PCarElt myc);
    void update(PSituation s);
    void initState();
    void calcBasics();
    void calcSpeed();
    double getSpeed(double ltrackangle);
    void calcDist();
    double cornerDist();
    // Data
    PCarElt car;    // pointer to the opponents car
    PCarElt mycar;  // pointer to the my car
    PTrack track;
    int state;      // state bitmask of the opponent
    double speed;    // speed in track direction
    double mDist;      // approximation of the real distance
    double mAngleToTrack;
    bool mAngleToLeft;
    bool mAside;
    double sidedist;    // side distance of center of gravity of the cars
    double toMiddle;
    double borderdist;
    bool teammate;
    bool backmarker;
    bool letpass;
    bool letpassaccel;
  private:
    // constants
    static const double FRONTRANGE;
    static const double BACKRANGE;
    static const int MAX_DAMAGE_DIFF;
};


/* The Opponents class holds an array of all Opponents */
class Opponents {
  public:
    Opponents(PTrack t, PSituation s, PCarElt car);
    ~Opponents();
    void update(PSituation s, PCarElt mycar);
    Opponent* oppNear() { return oppnear; }
    Opponent* oppNear2() { return oppnear2; }
    Opponent* oppLetPass() { return oppletpass; }
    Opponent* oppBack() { return oppback; }
    int nopponents;
    Opponent* opponent;
  private:
    Opponent* oppnear;
    Opponent* oppnear2;
    Opponent* oppletpass;
    Opponent* oppback;
};


#endif // _OPPONENT_H_
