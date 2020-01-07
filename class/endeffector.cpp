#include "endeffector.h"

EndEffector::EndEffector(float a, float alpha, float d, float teta) : Joint(a, alpha, d, teta){}

void EndEffector::draw(cv::Mat image)
{
	std::cout << "drawing end effector at position " << position << std::endl;
}
