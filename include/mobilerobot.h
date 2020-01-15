#ifndef MOBILEROBOT_H_
#define MOBILEROBOT_H_

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "structures.h"

#define HITBOX_RADIUS 15
#define HITBOX_THICKNESS 1
#define ROBOT_SIZE 10

class MobileRobot
{
	public:
		MobileRobot();
		MobileRobot(struct point_t pos);
	
		float * rotateZ(float angle);
		cv::Point rotate(cv::Point vect);
		
		void getRotation();
		void setPosition(struct point_t pos);
		struct point_t getPosition();
		struct point_t support(struct vec2 direction);
		
		void drawHitBox(cv::Mat image);
		void drawRobot(cv::Mat image);
		void draw(cv::Mat image);
		
	private:
		struct point_t position;
		float rotation[3][3] = {0};
};

#endif
