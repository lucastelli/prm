#ifndef ENDEFFECTOR_H_
#define ENDEFFECTOR_H_

#include <iostream>

#include "joint.h"

#define FINGER_LENGHT 10
#define THICKNESS 1

class EndEffector : public Joint 
{
	public:
		EndEffector();
		EndEffector(float a, float alpha, float d, float teta);
		
		void draw(cv::Mat image);
};

#endif
