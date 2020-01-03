#ifndef ROBOT_H_
#define ROBOT_H_

#include "joint.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class Robot
{
	private:
		std::vector<Joint*> joints;
		
	public:
		void addJoint(Joint* j);
		Joint * getJoint(int index);
		void computePose();
		float ** computeDHMatrix(Joint* joint);
		/*void draw();*/
};

#endif
