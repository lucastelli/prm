#include "joint.h"

Joint::Joint(float arm_lenght, float alpha_angle, float distance_lenght, float teta_angle) : 
		arm(arm_lenght), alpha(alpha_angle), distance(distance_lenght), teta(teta_angle){}
		
float Joint::getArm()
{
	return arm;
}
		
float Joint::getAlpha()
{
	return alpha;
}
	
float Joint::getDistance()
{
	return distance;
}

float Joint::getTeta()
{
	return teta;
}

float * Joint::getTrMatrix()
{
	/*trMatrix = {cos(teta), -sin(teta)*cos(alpha), sin(teta)*sin(alpha), arm*cos(teta), 
						sin(teta), cos(teta)*cos(alpha), -cos(teta)*sin(alpha), arm*sin(teta),
						0, sin(alpha), cos(alpha), distance,
						0, 0, 0, 1};*/
						
	trMatrix[0][0] = cos(teta);
	trMatrix[0][1] = -sin(teta)*cos(alpha);
	trMatrix[0][2] = sin(teta)*sin(alpha);
	trMatrix[0][3] = arm*cos(teta);
	
	trMatrix[1][0] = sin(teta);
	trMatrix[1][1] = cos(teta)*cos(alpha);
	trMatrix[1][2] = -cos(teta)*sin(alpha);
	trMatrix[1][3] = arm*sin(teta);
	
	trMatrix[2][0] = 0;
	trMatrix[2][1] = sin(alpha);
	trMatrix[2][2] = cos(alpha);
	trMatrix[2][3] = distance;
	
	trMatrix[3][0] = 0;
	trMatrix[3][1] = 0;
	trMatrix[3][2] = 0;
	trMatrix[3][3] = 1;
	
	return &trMatrix[0][0];
}

void Joint::setPosition(cv::Point p)
{
	position = p;
}
