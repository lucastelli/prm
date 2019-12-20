#ifndef JOINT_H_
#define JOINT_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class Rotoidal;
class Linear;

class Joint
{
	protected:
	float arm, alpha, distance, teta;
	
	public:
		Joint(float arm_lenght, float alpha_angle, float distance_lenght, float teta_angle);	
		float getArm();	
		float getAlpha();
		float getDistance();	
		float getTeta();
		virtual void draw(cv::Mat image) = 0;
};

#endif
