#ifndef ROBOT_H_
#define ROBOT_H_

#include "joint.h"
#include "endeffector.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define ARM_THICKNESS 2

class Robot
{
	private:
		std::vector<Joint*> joints;
		std::vector<float*> robot_config;
		
	public:
		Robot(Joint *zeroJoint, cv::Point referencePos);
		void addJoint(Joint *j);
		Joint * getJoint(int index);
		void setConfiguration(float **config, int num_var);
		void getConfiguration();
		void computePose();
		float * computeDHMatrix(Joint *joint);
		float * multDHMatrix(float *mat1, float *mat2);
		void draw(cv::Mat image);
		void drawArm(Joint *j1, Joint *j2, cv::Mat image);
};

#endif
