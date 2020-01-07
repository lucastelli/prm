#include "robot.h"

Robot::Robot(Joint *zeroJoint, cv::Point referencePos)
{
	addJoint(zeroJoint);
	joints.at(0)->setPosition(referencePos);
}

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
	float *m, *p; 
	float offset_x = joints.at(0)->getPosition().x;
	float offset_y = joints.at(0)->getPosition().y;
	m = computeDHMatrix(joints.at(0));
	joints.at(1)->setPosition(cv::Point(m[4*0+3] + offset_x, m[4*1+3] + offset_y));
	
	/*p = computeDHMatrix(joints.at(1));
	m = multDHMatrix(m, p);
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			std::cout << m[4*i+j] << " ";
		}
		std::cout << std::endl;
	}*/
	
	for(int i=1; i<joints.size()-1; i++)
	{
		p = computeDHMatrix(joints.at(i));
		m = multDHMatrix(m, p);
		joints.at(i+1)->setPosition(cv::Point(m[4*0+3] + offset_x, m[4*1+3] + offset_y));
	}
}

void Robot::draw(cv::Mat image)
{
	for(int i=0; i<joints.size(); i++)
	{
		joints.at(i)->draw(image);
	}
}

float * Robot::computeDHMatrix(Joint* joint)
{
	/*float matrix[4][4] = {cos(teta), -sin(teta)*cos(alpha), sin(teta)*sin(alpha), arm*cos(teta), 
								sin(teta), cos(teta)*cos(alpha), -cos(teta)*sin(alpha), arm*sin(teta),
								0, sin(alpha), cos(alpha), distance,
								0, 0, 0, 1};*/
								
	float arm = joint->getArm();
	float distance = joint->getDistance();
	float alpha = joint->getAlpha();
	float teta = joint->getTeta();
	
	float *matrix;
	matrix = (float*)malloc(4*4*sizeof(float));
	
	matrix[0] = cos(teta);
	matrix[1] = -sin(teta)*cos(alpha);
	matrix[2] = sin(teta)*sin(alpha);
	matrix[3] = arm*cos(teta);
	
	matrix[4] = sin(teta);
	matrix[5] = cos(teta)*cos(alpha);
	matrix[6] = -cos(teta)*sin(alpha);
	matrix[7] = arm*sin(teta);
	
	matrix[8] = 0;
	matrix[9] = sin(alpha);
	matrix[10] = cos(alpha);
	matrix[11] = distance;
	
	matrix[12] = 0;
	matrix[13] = 0;
	matrix[14] = 0;
	matrix[15] = 1;
	
	return matrix;
}

float * Robot::multDHMatrix(float *mat1, float *mat2)
{
	float *result;
	result = (float*)malloc(4*4*sizeof(float));
	
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			result[4*i+j] = 0;
		}
	}
	
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			for(int k=0; k<4; k++)
			{
				result[4*i+j] = result[4*i+j] + mat1[4*i+k]*mat2[4*k+j];
			}
		}
	}
	
	return result;
}


