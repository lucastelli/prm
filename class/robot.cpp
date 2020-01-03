#include "robot.h"

void Robot::addJoint(Joint* j)
{
	joints.push_back(j);
}

Joint * Robot::getJoint(int index)
{
	return joints.at(index);
}

void Robot::computePose()
{
	std::cout << "test" << std::endl;

	/*float **A_01; 
	A_01 = computeDHMatrix(joints.at(0));
	
	/*std::cout << A0_1[0][3] << ", " << A0_1[1][3] << ";" << std::endl;*/
	/*joints.at(0)->setPosition(cv::Point(A0_1[0][3], A0_1[1][3]));
	for(int i=1; i<joints.size(); i++)
	{
	
	}*/
}

float ** Robot::computeDHMatrix(Joint* joint)
{
	/*float matrix[4][4] = {cos(teta), -sin(teta)*cos(alpha), sin(teta)*sin(alpha), arm*cos(teta), 
								sin(teta), cos(teta)*cos(alpha), -cos(teta)*sin(alpha), arm*sin(teta),
								0, sin(alpha), cos(alpha), distance,
								0, 0, 0, 1};*/
								
	float arm = joint->getArm();
	float distance = joint->getDistance();
	float alpha = joint->getAlpha();
	float teta = joint->getTeta();
	
	float **matrix;
	matrix = (float**)malloc(4*4*sizeof(float));
	
	matrix[0][0] = cos(teta);
	matrix[0][1] = -sin(teta)*cos(alpha);
	matrix[0][2] = sin(teta)*sin(alpha);
	matrix[0][3] = arm*cos(teta);
	
	matrix[1][0] = sin(teta);
	matrix[1][1] = cos(teta)*cos(alpha);
	matrix[1][2] = -cos(teta)*sin(alpha);
	matrix[1][3] = arm*sin(teta);
	
	matrix[2][0] = 0;
	matrix[2][1] = sin(alpha);
	matrix[2][2] = cos(alpha);
	matrix[2][3] = distance;
	
	matrix[3][0] = 0;
	matrix[3][1] = 0;
	matrix[3][2] = 0;
	matrix[3][3] = 1;
	
	return matrix;
}
