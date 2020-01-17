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

// PRM Parameters //
#define N_CONFIG 2100 // total configuration

// Define drawing functions //
void updateMap(cv::Mat &map_x, cv::Mat &map_y);
cv::Point drawVector(cv::Mat img, struct vec2_t origin, double width, double angle, cv::Scalar color);
void drawLabel(cv::Mat img, std::string label, cv::Point origin);

//Define function gjk v2.0 //
enum gjkState{
  		UPDATE_SIMPLEX,
  		NO_COLLISION,
  		COLLISION_FOUND
  	};
struct vec2_t getSupport(struct vec2_t direction, MobileRobot mobile, Obstacle obj);
struct vec2_t getSupport(struct vec2_t direction, Obstacle ob1, Obstacle ob2);
bool search(struct vec2_t *vett, struct vec2_t pt, int num);
struct vec3_t vectorProduct(struct vec3_t a, struct vec3_t b);
struct vec2_t normalize(struct vec2_t vett);
struct vec3_t normalize(struct vec3_t vett);
float dotProduct(struct vec2_t v, struct vec2_t dir);
enum gjkState updateSimplex(struct vec2_t &direction, std::vector<struct vec2_t> *simplex_vertices, MobileRobot r, Obstacle ob);
enum gjkState addSupport(struct vec2_t direction, std::vector<struct vec2_t> *simplex_vertices, MobileRobot r, Obstacle ob);

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
  	
  	std::cout << "--- Display Robot & Obstacle ---" << std::endl;
  	std::cout << "\tcentre mob = [" << mobile.getPosition().x << ", " << mobile.getPosition().y << "]" << std::endl;
  	std::cout << "\tcentre ob1 = [" << ob1.getCentre().x << ", " << ob1.getCentre().y << "]" << std::endl;
  	std::cout << "\tcentre ob2 = [" << ob2.getCentre().x << ", " << ob2.getCentre().y << "]" << std::endl;
  	std::cout << "\tcentre ob3 = [" << ob3.getCentre().x << ", " << ob3.getCentre().y << "]" << std::endl;
  	std::cout << "\tcentre ob4 = [" << ob4.getCentre().x << ", " << ob4.getCentre().y << "]" << std::endl;
  	
  	/*----------------------------------------------
		PRM Algorithm (Mobile Robot Case)
	----------------------------------------------*/
	
	std::cout << "--- PRM Algorithm (Mobile Robot Case) ---" << std::endl;
	
	// Sampling Strategy : Uniform Distribution
	std::cout << "\tSampling Strategy : Uniform Distribution" << std::endl;
	
	struct vec2_t *setOfConfig; 
	setOfConfig = (vec2_t*)malloc(50 * 50 * sizeof(*setOfConfig));
	
	for(int i=0; i < N_CONFIG; i++)
	{
		setOfConfig[i].x = (rand() % WINDOW) - origin.x;
		setOfConfig[i].y = (rand() % WINDOW) - origin.y;
		
		circle(
			env_image,
			cv::Point(setOfConfig[i].x + origin.x, setOfConfig[i].y + origin.y),
			3,
			cv::Scalar(204,204,204),
			1
		);
	}
	std::cout << "\tdone" << std::endl;
	
	// GJK Collision Detection
  	std::cout << "\tGJK Collision Detection" << std::endl;
  	
  	std::vector<struct vec2_t> simplex_vertices;
	struct vec2_t direction;
  	enum gjkState result;
  	int checkOb = 0;
  	bool checkAllObstacle = false;
  	Obstacle *obx;
  	obx = (Obstacle *)malloc(sizeof(obx));
  	
  	std::cout << "&ob1 = " << &ob1 << std::endl;
  	std::cout << "&ob2 = " << &ob2 << std::endl;
  	std::cout << "&ob3 = " << &ob3 << std::endl;
  	std::cout << "&ob4 = " << &ob4 << std::endl;
  	
  	for(int i=0; i < N_CONFIG; i++)
  	{
  		simplex_vertices.clear();
	  	mobile.setPosition(setOfConfig[i]);
	  	std::cout << "cicle = " << i << std::endl;
	  	std::cout << "mobile position =[" << setOfConfig[i].x << ", " << setOfConfig[i].y << "]" << std::endl;
	  	result = UPDATE_SIMPLEX;
	  	checkOb = 0;
	  	obx = &ob1;
	  	checkAllObstacle = false;
	  	
	  	while(result == UPDATE_SIMPLEX)
	  	{
	  		result = updateSimplex(direction, &simplex_vertices, mobile, *obx);
	  		if(result == NO_COLLISION && checkAllObstacle == false)
	  		{
	  			switch(checkOb)
	  			{
	  				case 0:	obx = &ob2; 
	  							checkOb++; 
	  							result = UPDATE_SIMPLEX;
	  							simplex_vertices.clear(); 
	  							break;
	  							
	  				case 1: 	obx = &ob3; 
	  							checkOb++; 
	  							result = UPDATE_SIMPLEX;
	  							simplex_vertices.clear();
	  							break;
	  							
	  				case 2: 	obx = &ob4; 
	  							checkOb++; 
	  							result = UPDATE_SIMPLEX;
	  							simplex_vertices.clear(); 
	  							break;
	  							
	  				case 3: 	obx = &ob1; 
	  							checkOb = 0; 
	  							result = NO_COLLISION; 
	  							checkAllObstacle = true; 
	  							break;
	  			}
	  		}
	  		std::cout << "obx = " << obx << std::endl;
	  	}
	  	
	  	std::cout << "result " << result << std::endl;
	  	
	  	if(result == COLLISION_FOUND)
	  	{
	  		std::cout << "Collision!" << std::endl;
	  		circle(
				env_image,
				cv::Point(setOfConfig[i].x + origin.x, setOfConfig[i].y + origin.y),
				3,
				cv::Scalar(0,0,255),
				1
			);
	  	} 
	  	else 
	  	{
	  		std::cout << "NO Collision!" << std::endl;
	  	}
	  	std::cout << std::endl;
  	}
  	
  	
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

struct vec2_t getSupport(struct vec2_t direction, MobileRobot mobile, Obstacle obj)
{
	return mobile.support(direction) - obj.support(direction*(-1));;
}

struct vec2_t getSupport(struct vec2_t direction, Obstacle ob1, Obstacle ob2)
{
	return ob1.support(direction) - ob2.support(direction*(-1));;
}

struct vec3_t vectorProduct(struct vec3_t a, struct vec3_t b)
{
	struct vec3_t result;
	result.x = a.y * b.z - a.z * b.y;
	result.y = - a.x * b.z + a.z * b.x;
	result.z = a.x * b.y - a.y * b.x;
	return result;
}

struct vec2_t normalize(struct vec2_t vett)
{
	float mod = sqrt(pow(vett.x, 2) + pow(vett.y, 2));
	return vec2_t(vett.x/mod, vett.y/mod);
}

struct vec3_t normalize(struct vec3_t vett)
{
	float mod = sqrt(pow(vett.x, 2) + pow(vett.y, 2) + pow(vett.z, 2));
	return vec3_t(vett.x/mod, vett.y/mod, vett.z/mod);
}

float dotProduct(struct vec2_t v, struct vec2_t dir)
{
	return (v.x * dir.x + v.y * dir.y);
}

enum gjkState updateSimplex(struct vec2_t &direction, std::vector<struct vec2_t> *simplex_vertices, MobileRobot r, Obstacle ob)
{
	struct vec2_t a, v2_o, v2_v1, v2_v0, v2_v1_orth_2d, v2_v0_orth_2d;
	struct vec3_t tmp, v2_v1_orth, v2_v0_orth;
	float dot1, dot2;
	
	
	std::cout << "simplex_vertices.size() = " << simplex_vertices[0].size() << std::endl;
	switch(simplex_vertices[0].size())
	{
		case 0:	direction = r.getPosition() - ob.getCentre();
					std::cout << "case 0" << std::endl;
					std::cout << "direction = ["<< direction.x << ", " << direction.y << "]" << std::endl;
					break;
					
		case 1:	direction = direction*(-1);
					std::cout << "case 1" << std::endl;
					std::cout << "direction = ["<< direction.x << ", " << direction.y << "]" << std::endl;
					break;
					
		case 2:	a = simplex_vertices[0].at(0) - simplex_vertices[0].at(1);
  					tmp = vectorProduct(vec3_t(a.x, a.y, 0), vec3_t(-simplex_vertices[0].at(0).x, -simplex_vertices[0].at(0).y, 0));
  					tmp = vectorProduct(tmp, vec3_t(a.x, a.y, 0));
  					tmp = normalize(tmp);
  					direction.x = tmp.x;
  					direction.y = tmp.y;
  					std::cout << "case 2" << std::endl;
					std::cout << "direction = ["<< direction.x << ", " << direction.y << "]" << std::endl;
  					break;
  		
  		case 3:	v2_o = simplex_vertices[0].at(2)*(-1);								//v2 to the origin
					v2_v1 = simplex_vertices[0].at(1) - simplex_vertices[0].at(2);	//v2 to v1
					v2_v0 = simplex_vertices[0].at(0) - simplex_vertices[0].at(2); //v2 to v0
					
					v2_v1_orth = vectorProduct(vec3_t(v2_v0.x, v2_v0.y, 0), vec3_t(v2_v1.x, v2_v1.y, 0));
					v2_v1_orth = normalize(v2_v1_orth);
					v2_v1_orth = vectorProduct(v2_v1_orth, vec3_t(v2_v1.x, v2_v1.y, 0));
					v2_v1_orth = normalize(v2_v1_orth);
					
					v2_v0_orth = vectorProduct(vec3_t(v2_v1.x, v2_v1.y, 0), vec3_t(v2_v0.x, v2_v0.y, 0));
					v2_v0_orth = normalize(v2_v0_orth);
					v2_v0_orth = vectorProduct(v2_v0_orth, vec3_t(v2_v0.x, v2_v0.y, 0));
					v2_v0_orth = normalize(v2_v0_orth);
					
					v2_o = normalize(v2_o);
					v2_v1_orth_2d = vec2_t(v2_v1_orth.x, v2_v1_orth.y);
					v2_v0_orth_2d = vec2_t(v2_v0_orth.x, v2_v0_orth.y);
					dot1 = dotProduct(v2_o, v2_v1_orth_2d);
					dot2 = dotProduct(v2_o, v2_v0_orth_2d);
					if(dot1 > 0)
					{
						// remove v0 -> reallocation (v1 become v0 & v2 become v1)
						simplex_vertices[0].erase(simplex_vertices[0].begin());
						direction = v2_v1_orth_2d;
					}
					else if(dot2 > 0)
					{
						// remove v1 -> reallocation (v2 become v1 & v0 remain v0)
						simplex_vertices[0].erase(simplex_vertices[0].begin()+1);
						direction = v2_v0_orth_2d;
					}
					else
					{
						return COLLISION_FOUND;
					}
					std::cout << "case 3" << std::endl;
					std::cout << "dot1 = " << dot1 << std::endl;
					std::cout << "dot2 = " << dot1 << std::endl;
					std::cout << "direction = ["<< direction.x << ", " << direction.y << "]" << std::endl;
					break;
		
		default:	std::cout << "Error: case dafault" << std::endl;
					break;
	}
	
	return addSupport(direction, simplex_vertices, r, ob);
}

enum gjkState addSupport(struct vec2_t direction, std::vector<struct vec2_t> *simplex_vertices, MobileRobot r, Obstacle ob)
{
	enum gjkState result = UPDATE_SIMPLEX;
	struct vec2_t newVertex;
	float dot;
	newVertex = getSupport(direction, r, ob);
	simplex_vertices[0].push_back(newVertex);
	dot = dotProduct(direction, normalize(newVertex));
	if(dot < 0)
	{
		result = NO_COLLISION;
	}
	std::cout << "Add Support" << std::endl;
	std::cout << "newVertex = ["<< newVertex.x << ", " << newVertex.y << "]" << std::endl;
	std::cout << "simplex_vertices = { " << std::endl;
	for(int i=0; i < simplex_vertices[0].size(); i++)
	{
		std::cout << "\t v"<< i << " = [" << simplex_vertices[0].at(i).x << ", " << simplex_vertices[0].at(i).y << "]" << std::endl;
	}
	std::cout << "}" << std::endl;
	std::cout << "simplex_vertices.size() = " << simplex_vertices[0].size() << std::endl;
	std::cout << "dotProduct(direction, newVertex) = " << dot << std::endl;
	return result;
}

void drawLabel(cv::Mat img, std::string label, cv::Point origin)
{	
	cv::Point tmp(origin.x + LABEL_OFFSET, WINDOW - origin.y - LABEL_OFFSET);
	putText(img, label, tmp, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 1, LINE_AA);
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
