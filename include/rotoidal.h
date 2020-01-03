#ifndef ROTOIDAL_H_
#define ROTOIDAL_H_

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "joint.h"

#define RADIUS_INNER_CIRCLE 5
#define RADIUS_OUTER_CIRCLE 15
#define THICKNESS 1

class Rotoidal : public Joint 
{
	public:
		Rotoidal();
		Rotoidal(float a, float alpha, float d, float teta);
		
		void draw(cv::Mat image);
};

#endif
