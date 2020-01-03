#ifndef JOINT_H_
#define JOINT_H_

#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class Rotoidal;
class Linear;

class Joint
{
	protected:
		float arm;						// distance from z(i-1) to z(i) axes on x(i) axis
		float alpha;					// angle around x(i) between z(i-1) and z(i) axes (positive counter-clockwise)
		float distance;				// distance from x(i-1) to x(i) axes on z(i-1) axis
		float teta;						// angle around z(i-1) between x(i-1) and x(i) axes (positive counter-clockwise)
	
		cv::Point position;			// position of the joint in the workspace
	
	public:
		Joint(float arm_lenght, float alpha_angle, float distance_lenght, float teta_angle);	
		float getArm();	
		float getAlpha();
		float getDistance();	
		float getTeta();
		
		void setPosition(cv::Point p);
		virtual void draw(cv::Mat image) = 0;
};

#endif
