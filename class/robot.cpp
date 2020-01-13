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

void Robot::setConfiguration(float **config, int num_var)
{
	for(int i=0; i<num_var; i++)
	{
		robot_config.push_back(config[i]);
	}
}

void Robot::getConfiguration()
{
	std::cout << "[";
	for(int i=0; i<robot_config.size()-1; i++)
	{
		std::cout << *robot_config.at(i) << ", ";
	}
	std::cout << *robot_config.at(robot_config.size()-1) << "]" << std::endl;
}

void Robot::editConfiguration(float *new_config, int num)
{
	for(int i=0; i<num; i++)
	{
		*robot_config.at(i) = new_config[i];
	}
	computePose(); //update robot pose with new config
}

void Robot::computePose()
{
	float *m, *p; 
	float offset_x = joints.at(0)->getPosition().x;
	float offset_y = joints.at(0)->getPosition().y;
	m = computeDHMatrix(joints.at(0));
	joints.at(1)->setPosition(cv::Point(m[4*0+3] + offset_x, m[4*1+3] + offset_y));
	joints.at(1)->setRotation(m);
	
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
		joints.at(i+1)->setRotation(m);
	}
}

void Robot::draw(cv::Mat image)
{
	// Draw arms
	for(int i=0; i<joints.size()-1; i++)
	{
		drawArm(joints.at(i), joints.at(i+1), image);
	}
	
	// Draw joints
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
	
	float cos_teta, cos_alpha;
	cos_teta = (teta == (float)M_PI/2 || teta == (float)-M_PI/2) ? 0 : cos(teta);
	cos_alpha = (alpha == (float)M_PI/2 || alpha == (float)-M_PI/2) ? 0 : cos(alpha);
	
	matrix[0] = cos_teta;
	matrix[1] = -sin(teta)*cos_alpha;
	matrix[2] = sin(teta)*sin(alpha);
	matrix[3] = arm*cos_teta;
	
	matrix[4] = sin(teta);
	matrix[5] = cos_teta*cos_alpha;
	matrix[6] = -cos_teta*sin(alpha);
	matrix[7] = arm*sin(teta);
	
	matrix[8] = 0;
	matrix[9] = sin(alpha);
	matrix[10] = cos_alpha;
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

void Robot::drawArm(Joint *j1, Joint *j2, cv::Mat image)
{
	cv::Scalar color = cv::Scalar(0, 0, 0);	
	
	line( image,
      j1->getPosition(),
      j2->getPosition(),
      color,
      ARM_THICKNESS,
      cv::LINE_AA );
}
