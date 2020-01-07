#ifndef ROTOIDAL_H_
#define ROTOIDAL_H_

#include <iostream>

#include "joint.h"

#define RADIUS_INNER_CIRCLE 5
#define RADIUS_OUTER_CIRCLE 10
#define THICKNESS 1

class Rotoidal : public Joint 
{
	public:
		Rotoidal();
		Rotoidal(float a, float alpha, float d, float teta);
		
		void draw(cv::Mat image);
};

#endif
