#include "DistanceSensor.h"

DistanceSensor::DistanceSensor()
{
	this->m_direction = v2t<float>(0.0f, 0.0f);
}

DistanceSensor::DistanceSensor(float x, float y)
{
	this->m_direction = v2t<float>(x, y);
}

DistanceSensor::~DistanceSensor()
{

}

float DistanceSensor::calcDistance(tTrack *track, tCarElt* car)
{
	float distance = 200.0f;

	v2t<float> rotatedDirection = v2t<float>(this->m_direction.x * cos(car->pub.DynGCg.pos.az) - this->m_direction.y * sin(car->pub.DynGCg.pos.az), this->m_direction.x * sin(car->pub.DynGCg.pos.az) + this->m_direction.y * cos(car->pub.DynGCg.pos.az));
	v2t<float> collisionPoint;
	v2t<float> sensorStart = v2t<float>(car->pub.DynGCg.pos.x, car->pub.DynGCg.pos.y);
	v2t<float> sensorEnd = v2t<float>(car->pub.DynGCg.pos.x, car->pub.DynGCg.pos.y) + rotatedDirection;
	v2t<float> borderStart;
	v2t<float> borderEnd;
	float denominator;
	car->pub.posMat;

	printf_s("Rotated direction: %f %f\n", rotatedDirection.x, rotatedDirection.y);

	for(int i = 0; i < track->nseg; i++)
	{
		//Left side
		borderStart = v2t<float>(track->seg[i].vertex[TR_SL].x, track->seg[i].vertex[TR_SL].y);
		borderEnd = v2t<float>(track->seg[i].vertex[TR_EL].x, track->seg[i].vertex[TR_EL].y);

		denominator = (sensorStart.x - sensorEnd.x) * (borderStart.y - borderEnd.y) - (sensorStart.y - sensorEnd.y) * (borderStart.x - borderEnd.x);

		if(denominator != 0)
		{
			collisionPoint.x = ((sensorStart.x * sensorEnd.y - sensorStart.y * sensorEnd.x) * (borderStart.x - borderEnd.x) - (borderStart.x * borderEnd.y - borderStart.y * borderEnd.x) * (sensorStart.x - sensorEnd.x)) / denominator;
			collisionPoint.y = ((sensorStart.x * sensorEnd.y - sensorStart.y * sensorEnd.x) * (borderStart.y - borderEnd.y) - (borderStart.x * borderEnd.y - borderStart.y * borderEnd.x) * (sensorStart.y - sensorEnd.y)) / denominator;

			//Check if the collision point is negative to the direction
			if(collisionPoint.x / this->m_direction.x > 0.0f)
			{
				//Check if the collision point is within the vector
				if(collisionPoint.x - borderStart.x > 0.0f && collisionPoint.x - borderStart.x < borderEnd.x && collisionPoint.y - borderStart.y > 0.0f && collisionPoint.y - borderStart.y < borderEnd.y)
				{
					//Update the shortest distance
					distance = min(v3d(collisionPoint.x - borderStart.x, collisionPoint.y - borderStart.y, 0.0f).len(), distance);
				}		
			}
		}

		//Right side
		borderStart = v2t<float>(track->seg[i].vertex[TR_SR].x, track->seg[i].vertex[TR_SR].y);
		borderEnd = v2t<float>(track->seg[i].vertex[TR_ER].x, track->seg[i].vertex[TR_ER].y);

		denominator = (sensorStart.x - sensorEnd.x) * (borderStart.y - borderEnd.y) - (sensorStart.y - sensorEnd.y) * (borderStart.x - borderEnd.x);

		if(denominator != 0)
		{
			collisionPoint.x = ((sensorStart.x * sensorEnd.y - sensorStart.y * sensorEnd.x) * (borderStart.x - borderEnd.x) - (borderStart.x * borderEnd.y - borderStart.y * borderEnd.x) * (sensorStart.x - sensorEnd.x)) / denominator;
			collisionPoint.y = ((sensorStart.x * sensorEnd.y - sensorStart.y * sensorEnd.x) * (borderStart.y - borderEnd.y) - (borderStart.x * borderEnd.y - borderStart.y * borderEnd.x) * (sensorStart.y - sensorEnd.y)) / denominator;

			//Check if the collision point is negative to the direction
			if(collisionPoint.x / this->m_direction.x > 0.0f)
			{
				//Check if the collision point is within the vector
				if(collisionPoint.x > borderStart.x && collisionPoint.x < borderEnd.x && collisionPoint.y > borderStart.y && collisionPoint.y < borderEnd.y)
				{
					//Update the shortest distance
					distance = min(v3d(collisionPoint.x - borderStart.x, collisionPoint.y - borderStart.y, 0.0f).len(), distance);
				}		
			}
		}
	}

	return distance;
}