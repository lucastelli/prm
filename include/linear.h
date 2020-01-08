#ifndef LINEAR_H_
#define LINEAR_H_

#include <iostream>

#include "joint.h"
#include "robot.h"

#define LINEAR_THICKNESS 1

class Linear: public Joint
{
	public:
		Linear();
		Linear(float a, float alpha, float d, float teta);
	
		void draw(cv::Mat image);
		cv::Point rotate(cv::Point vect);
};

#endif
