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
		Robot(Joint *zeroJoint, cv::Point referencePos);
		void addJoint(Joint *j);
		Joint * getJoint(int index);
		void computePose();
		float * computeDHMatrix(Joint *joint);
		float * multDHMatrix(float *mat1, float *mat2);
		/*void draw();*/
};

#endif
