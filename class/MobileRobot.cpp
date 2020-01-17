#include "mobilerobot.h"

MobileRobot::MobileRobot(struct vec2_t pos) : position(pos)
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
		
void MobileRobot::drawHitBox(cv::Mat image, struct vec2_t reference)
{
	cv::Scalar color = cv::Scalar(0, 0, 255); //red
	struct vec2_t hc = position + reference;
	cv::Point pos = cv::Point(hc.x, hc.y);
	
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

void MobileRobot::drawRobot(cv::Mat image, struct vec2_t reference)
{
	cv::Scalar color = cv::Scalar(10, 240, 0); //green
	struct vec2_t rc = position + reference;
	cv::Point pos = cv::Point(rc.x, rc.y);
	
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

void MobileRobot::draw(cv::Mat image, struct vec2_t reference)
{
	drawRobot(image, reference);
	drawHitBox(image, reference);
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

void MobileRobot::setPosition(struct vec2_t pos)
{
	position = pos;
}

struct vec2_t MobileRobot::getPosition()
{
	return position;
}

struct vec2_t MobileRobot::support(struct vec2_t direction)
{
	float mod;
	struct vec2_t result;
	mod = sqrt(pow(direction.x,2)+pow(direction.y,2));
	result.x = position.x + HITBOX_RADIUS * direction.x / mod;
	result.y = position.y + HITBOX_RADIUS * direction.y / mod;
	return result;
}
