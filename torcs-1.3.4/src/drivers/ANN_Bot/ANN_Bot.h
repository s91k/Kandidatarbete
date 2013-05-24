#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robot.h>
#include <robottools.h>
#include <math.h>
#include "spline.h"
#include "trackdesc.h"
#include "mycar.h"
#include "pathfinder.h"
#include "sensors.h"

#define TRACKFILENAME		"/home/berni/track.dat"
#define TRACKDESCFILENAME   "/home/berni/track.txt"
#define RMAX				10000.0
#define BOTS 10
#define BUFSIZE 256

#define ANN_BOT_SECT_PRIV	"ann_bot private"
#define ANN_BOT_ATT_PITENTRY	"pitentry"
#define ANN_BOT_ATT_PITEXIT	"pitexit"
#define ANN_BOT_ATT_AMAGIC	"caero"
#define ANN_BOT_ATT_FMAGIC	"cfriction"
#define ANN_BOT_ATT_FUELPERLAP "fuelperlap"

static const double g = 9.81;


/* compute the angle between the track and the car */
inline double queryAngleToTrack(tCarElt * car)
{
	double angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	NORM_PI_PI(angle);
	return angle;
}


inline double sign(double number)
{
	return (number >= 0.0) ? 1.0 : -1.0;
}


inline double sqr(double x)
{
	return x*x;
}


/* compute the radius given three points */
inline double radius(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double dx1 = x2 - x1;
	double dy1 = y2 - y1;
	double dx2 = x3 - x2;
	double dy2 = y3 - y2;

	//double z = (dy1*dx2 - dx1*dy2);
	double z = dx1*dy2 - dy1*dx2;

	if (z != 0.0) {
		double k = (dx2*(x3-x1) - dy2*(y1-y3))/z;
		return sign(z)*sqrt((1.0+k*k)*(dx1*dx1+dy1*dy1))/2.0;
	} else {
		return FLT_MAX;
	}
}