#include <cmath>
#include <iostream>
#include <string>
#include <regex>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "joint.h"
#include "rotoidal.h"
#include "linear.h"
#include "endeffector.h"
#include "robot.h"
#include "prmConfig.h"
#include "obstacle.h"
#include "mobilerobot.h"
#include "structures.h"

#include <time.h>

// Window Parameters //
#define WINDOW 400

// Frame Reference Parameters //
#define FRAME_OFFSET 5
#define LABEL_OFFSET 2
#define AXIS_LENGTH 50
#define ARROW_LENGTH 6

// Define functions //
void updateMap(cv::Mat &map_x, cv::Mat &map_y);
cv::Point drawVector(cv::Mat img, struct vec2_t origin, double width, double angle, cv::Scalar color);
void drawLabel(cv::Mat img, std::string label, cv::Point origin);
struct vec2_t getSupport(struct vec2_t direction, MobileRobot mobile, Obstacle obj);
struct vec3_t vectorProduct(struct vec3_t a, struct vec3_t b);
float dotProduct(struct vec2_t v, struct vec2_t dir);

// Define namespace //
using namespace cv;


// Main program //

int main(int argc, char** argv)
{
	// report version
   std::cout << argv[0] << " Version " << prm_VERSION_MAJOR << "."
              << prm_VERSION_MINOR << std::endl;
	
	clock_t time_start, time_end;
	time_start = clock();
	/*----------------------------------------------
		Define 3 arm planar robot manipulator
			with DH convention
	----------------------------------------------*/
	
	/*float l1 = 50;
	float l2 = 50;
	float l3 = 50;
	
	float angle_1 = M_PI/6;
	float angle_2 = M_PI/6;
	float angle_3 = M_PI/6;
	
	Rotoidal j1_r(l1, 0, 0, angle_1);
	Linear j2_l(l2, 0, 0, angle_2);
	Rotoidal j3_r(l3, 0, 0, angle_3);
	EndEffector end;*/
	
	/*float d1 = 140;
	float l2 = 150;
	float l3 = 150;
	
	float angle_1 = M_PI/12;
	float angle_2 = M_PI/6;
	
	Rotoidal j1_r(d1, -M_PI/2, 0, angle_1);
	Linear j2_l(0, M_PI/2, l2, 0);
	Rotoidal j3_r(l3, 0, 0, angle_2);
	EndEffector end;*/
	
	/*float l1 = 150;
	float l2 = 150;
	float l3 = 150;
	
	float angle_1 = M_PI/12;
	
	Rotoidal j1_r(l1, -M_PI/2, 0, angle_1);
	Linear j2_l(0, M_PI/2, l2, 0);
	Linear j3_l(l3, 0, 0, 0);
	EndEffector endEffector;
	
	// Create robot object
	cv::Point origin(FRAME_OFFSET, FRAME_OFFSET);
	Robot manipulator(&j1_r, origin);
  	manipulator.addJoint(&j2_l);
  	manipulator.addJoint(&j3_l);
  	manipulator.addJoint(&endEffector);
  	
  	float *vet[] = {j1_r.getPointerTeta(), j1_r.getPointerArm(), j2_l.getPointerDistance()};
  	manipulator.setConfiguration(vet, 3);
  	//manipulator.getConfiguration();*/
  	
	/*----------------------------------------------
		Display the environment: reference frame, robot, obstacles, 
			free robot configurations, optimal path
	----------------------------------------------*/
	
	char env_window[] = "PRM Environment";
	//float input_config[3] = {0};
	
	cv::Mat env_image( WINDOW, WINDOW, CV_8UC3, cv::Scalar(255,255,255));
  	cv::Mat flip_env_image( WINDOW, WINDOW, CV_8UC3, cv::Scalar(255,255,255));
  	cv::Mat map_x(env_image.size(), CV_32FC1);
  	cv::Mat map_y(env_image.size(), CV_32FC1);
  	
  	// Define and display reference frame	
  	cv::Point xAxis, yAxis;
  	struct vec2_t origin = {20, 20};
  	
  	xAxis = drawVector(env_image, origin, AXIS_LENGTH, 0, Scalar(255, 0, 0));
  	yAxis = drawVector(env_image, origin, AXIS_LENGTH, M_PI/2, Scalar(0, 0, 255));
  	
  	// Display Robot
  	//manipulator.computePose();
  	//manipulator.draw(env_image);
  	
  	MobileRobot mobile({0, 0});
  	//mobile.getRotation();
  	mobile.draw(env_image, origin);
  	
  	// Display obstacles
  	struct vec2_t ob_pts1[] = {	
  											{50,50}, 
  											{50, 100},
  											{85, 135},
  											{135, 135},
  											{170, 100},
  											{170, 50},
  											{135, 15},
  											{85, 15}
  										};
  										
  	struct vec2_t ob_pts2[] = {
  											{0, 400},
  											{200, 200},
  											{250,250},
  											{50,400}
  										};
  										
  	struct vec2_t ob_pts3[] = {
  											{250, 0},
  											{400, 0},
  											{400, 400}
  										};
  										
  	struct vec2_t ob_pts4[] = {
  											{80, 250},
  											{150, 200},
  											{0, 200}
  										};
  	
  	Obstacle ob1 = Obstacle(ob_pts1, 8);
  	Obstacle ob2 = Obstacle(ob_pts2, 4);
  	Obstacle ob3 = Obstacle(ob_pts3, 3);
  	Obstacle ob4 = Obstacle(ob_pts4, 3);
  	ob1.draw(env_image, cv::Scalar(0,0,0), origin);
  	ob2.draw(env_image, cv::Scalar(0,0,0), origin);
  	ob3.draw(env_image, cv::Scalar(0,0,0), origin);
  	ob4.draw(env_image, cv::Scalar(0,0,0), origin);
  	
  	/*----------------------------------------------
		PRM Algorithm
	----------------------------------------------*/
	
	//nel caso di un mobile robot
	
	
	// Sampling Strategy : Uniform Distribution
	struct vec2_t *setOfConfig; 
	setOfConfig = (vec2_t*)malloc(50 * 50 * sizeof(*setOfConfig));
	
	for(int i=0; i < 2500; i++)
	{
		setOfConfig[i].x = rand() % WINDOW;
		setOfConfig[i].y = rand() % WINDOW;
		
		circle(
			env_image,
			cv::Point(setOfConfig[i].x, setOfConfig[i].y),
			3,
			cv::Scalar(204,204,204),
			1
		);
	}
  	
  	struct vec2_t centre = ob1.getCentre();
  	std::cout << "centre mob = [" << mobile.getPosition().x << ", " << mobile.getPosition().y << "]" << std::endl;
  	std::cout << "centre ob1 = [" << centre.x << ", " << centre.y << "]" << std::endl;
  	
  	
	// Flip vertical entire image
	updateMap(map_x, map_y);
	remap( env_image, flip_env_image, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0) );
	
	// Display the vector labels of all frames reference
	drawLabel(flip_env_image, "x", xAxis);
	drawLabel(flip_env_image, "y", yAxis);
	
	// Display the flip image
	imshow( env_window, flip_env_image );
	
	time_end = clock();
	
	std::cout << "Execution time: " << double(time_end - time_start) / double(CLOCKS_PER_SEC) << std::endl;
	
	waitKey( 0 );
	
	free(setOfConfig);
	
	/*while(true)
	{ 	
		std::cout << "Inserisci configurazione [j1_teta j1_arm j2_distance]" << std::endl;
		scanf("%f %f %f", &input_config[0], &input_config[1], &input_config[2]);
		
		env_image.setTo(Scalar(255,255,255));
		flip_env_image.setTo(Scalar(255,255,255));
		
		manipulator.editConfiguration(input_config, 3);
	  	manipulator.draw(env_image);
	  	
	  	xAxis = drawVector(env_image, origin, AXIS_LENGTH, 0, Scalar(255, 0, 0));
  		yAxis = drawVector(env_image, origin, AXIS_LENGTH, M_PI/2, Scalar(0, 0, 255));
  	
	  	updateMap(map_x, map_y);
		remap( env_image, flip_env_image, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0) );
		
		drawLabel(flip_env_image, "x", xAxis);
		drawLabel(flip_env_image, "y", yAxis);
		
		imshow( env_window, flip_env_image );
		waitKey( 100 );
	}*/
	
	return(0);
}

void updateMap(cv::Mat &map_x, cv::Mat &map_y)
{
	for(int i = 0; i < map_x.rows; i++)
	{
		for(int j = 0; j < map_x.cols; j++)
		{
			map_x.at<float>(i, j) = (float)j;
      	map_y.at<float>(i, j) = (float)(map_x.rows - i);
		}
	}
}

cv::Point drawVector(cv::Mat img, struct vec2_t origin, double width, double angle, cv::Scalar color)
{
	int thickness = 1;
  	int lineType = cv::LINE_8;
  	
	cv::Point vectorAxis(origin.x + width * cos(angle), origin.y + width * sin(angle));
	
	cv::Point arrowAxis_l(vectorAxis.x - ARROW_LENGTH * cos(M_PI/6 - angle), vectorAxis.y + ARROW_LENGTH * sin(M_PI/6 - angle));
	cv::Point arrowAxis_r(vectorAxis.x - ARROW_LENGTH * cos(M_PI/6 + angle), vectorAxis.y - ARROW_LENGTH * sin(M_PI/6 + angle));
  	
  	line( img,
    cv::Point(origin.x, origin.y),
    vectorAxis,
    color,
    thickness,
    lineType );
    
    line( img,
    vectorAxis,
    arrowAxis_l,
    color,
    thickness,
    lineType );
    
    line( img,
    vectorAxis,
    arrowAxis_r,
    color,
    thickness,
    lineType );
    
    return vectorAxis;
}

void drawLabel(cv::Mat img, std::string label, cv::Point origin)
{	
	cv::Point tmp(origin.x + LABEL_OFFSET, WINDOW - origin.y - LABEL_OFFSET);
	putText(img, label, tmp, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 1, LINE_AA);
}

struct vec2_t getSupport(struct vec2_t direction, MobileRobot mobile, Obstacle obj)
{
	return mobile.support(direction) - obj.support(direction*(-1));;
}

struct vec3_t vectorProduct(struct vec3_t a, struct vec3_t b)
{
	struct vec3_t result;
	result.x = a.y * b.z - a.z * b.y;
	result.y = a.x * b.z - a.z * b.x;
	result.z = a.x * b.y - a.y * b.x;
	return result;
}

float dotProduct(struct vec2_t v, struct vec2_t dir)
{
	return (v.x * dir.x + v.y * dir.y);
}
