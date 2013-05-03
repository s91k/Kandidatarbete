#pragma once

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

class DistanceSensor
{
private:
	v2t<float> m_direction;
public:
	DistanceSensor();
	DistanceSensor(float x, float y);

	~DistanceSensor();

	float calcDistance(tTrack *track, tCarElt* car);
};

