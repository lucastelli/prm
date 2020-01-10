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

cv::Point Joint::getPosition()
{
	return position;
}

void Joint::setPosition(cv::Point p)
{
	position = p;
}

float * Joint::getRotation()
{
	return &rotationMatrix[0][0];
}

void Joint::setRotation(float *matrix)
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			rotationMatrix[i][j] = matrix[4*i+j];
		}
	}
}

float * Joint::getPointerArm()
{
	return &arm;
}

float * Joint::getPointerAlpha()
{
	return &alpha;
}

float * Joint::getPointerDistance()
{
	return &distance;
}

float * Joint::getPointerTeta()
{
	return &teta;
}

// Only 2D function
cv::Point Joint::rotate(cv::Point vect)
{
	cv::Point result;
	result.x = rotationMatrix[0][0]*vect.x + rotationMatrix[0][1]*vect.y;
	result.y = rotationMatrix[1][0]*vect.x + rotationMatrix[1][1]*vect.y;
	
	return result;
}
