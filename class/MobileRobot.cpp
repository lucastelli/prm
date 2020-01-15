#include "mobilerobot.h"

MobileRobot::MobileRobot(struct point_t pos) : position(pos)
{
	rotateZ(0);
}

float * MobileRobot::rotateZ(float angle)
{
	float cos_angle;
	cos_angle = (angle == (float)M_PI/2 || angle == (float)-M_PI/2) ? 0 : cos(angle);
	
	rotation[0][0] = cos_angle;
	rotation[0][1] = -sin(angle);
	//orientation[0][2] = 0;
	rotation[1][0] = sin(angle);
	rotation[1][1] = cos_angle;
	//orientation[1][2] = 0;
	//orientation[2][0] = 0;
	//orientation[2][1] = 0;
	rotation[2][2] = 1;
	
	return (float*)rotation;
}
		
void MobileRobot::drawHitBox(cv::Mat image)
{
	cv::Scalar color = cv::Scalar(0, 0, 255); //red
	cv::Point pos = cv::Point(position.x, position.y);
	
	circle(
		image,
		pos,
		HITBOX_RADIUS,
		color,
		HITBOX_THICKNESS,
		cv::LINE_AA
	);
	
	circle(
		image,
		pos,
		0,
		cv::Scalar(0, 0, 0),
		HITBOX_THICKNESS,
		cv::LINE_AA
	);
}

void MobileRobot::drawRobot(cv::Mat image)
{
	cv::Scalar color = cv::Scalar(10, 240, 0); //green
	cv::Point pos = cv::Point(position.x, position.y);
	
	cv::Point pt_a = cv::Point(0, ROBOT_SIZE);
	cv::Point pt_b = cv::Point(ROBOT_SIZE * cos(M_PI/4), - ROBOT_SIZE * 0.5);
	cv::Point pt_c = cv::Point(- ROBOT_SIZE * cos(M_PI/4), - ROBOT_SIZE * 0.5);
	
	pt_a = rotate(pt_a) + pos;
	pt_b = rotate(pt_b) + pos;
	pt_c = rotate(pt_c) + pos;
	
	cv::Point robot[] = {pt_a, pt_b, pt_c};
	
	fillConvexPoly(
		image,
		robot,
		3,
		color,
		cv::LINE_AA
	);
}

cv::Point MobileRobot::rotate(cv::Point vect)
{
	cv::Point result;
	result.x = rotation[0][0]*vect.x + rotation[0][1]*vect.y;
	result.y = rotation[1][0]*vect.x + rotation[1][1]*vect.y;
	
	return result;
}

void MobileRobot::draw(cv::Mat image)
{
	drawRobot(image);
	drawHitBox(image);
}

void MobileRobot::getRotation()
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			std::cout << rotation[i][j] << " ";
		}
		std::cout << std::endl;
	}
}

void MobileRobot::setPosition(struct point_t pos)
{
	position = pos;
}

struct point_t MobileRobot::getPosition()
{
	return position;
}

struct point_t MobileRobot::support(struct vec2 direction)
{
	float a, b, mod;
	struct point_t result;
	a = direction.end.x - direction.start.x;
	b = direction.end.y - direction.start.y;
	mod = sqrt(pow(a,2)+pow(b,2));
	result.x = position.x + HITBOX_RADIUS * a / mod;
	result.y = position.y + HITBOX_RADIUS * b / mod;
	return result;
}
