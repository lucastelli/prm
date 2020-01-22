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
#include "gjkdetector.h"

#include <time.h>

// Window Parameters //
#define WINDOW 400

// Frame Reference Parameters //
#define FRAME_OFFSET 5
#define LABEL_OFFSET 2
#define AXIS_LENGTH 	50
#define ARROW_LENGTH 6

// PRM Parameters //
#define N_CONFIG 		50 	// total configuration
#define N_NEIGHBORS 	3

// Define drawing functions //
void updateMap(cv::Mat &map_x, cv::Mat &map_y);
cv::Point drawVector(cv::Mat img, struct vec2_t origin, double width, double angle, cv::Scalar color);
void drawLabel(cv::Mat img, std::string label, cv::Point origin);

//Define function gjk v2.0 //
/*enum gjkState{
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
enum gjkState addSupport(struct vec2_t direction, std::vector<struct vec2_t> *simplex_vertices, MobileRobot r, Obstacle ob);*/

float medianXAxis(struct vec2_t *array, int left, int right);
//void selectXAxis(struct vec2_t *array, int l, int r, int k);
void quicksortXaxis(struct vec2_t *array, int l, int r);
int partitionXAxis(struct vec2_t *array, int l, int r);
float medianYAxis(struct vec2_t *array, int left, int right);
//void selectYAxis(struct vec2_t *array, int l, int r, int k);
void quicksortYaxis(struct vec2_t *array, int l, int r);
int partitionYAxis(struct vec2_t *array, int l, int r);
void swap(struct vec2_t *array, int i, int j);

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
	
	GJKDetector gjk;					// GJK detector object
	struct vec2_t config_sample;	// Configuration sampled by PRM sampling strategy
	Obstacle *obs;						// Array of obstacles in the work-space
	struct vec2_t *conf_array;		// Array of free-collision configurations
	int num_conf = 0;					// number of free-collision configurations
																
	conf_array = (struct vec2_t *)malloc(N_CONFIG * sizeof(*conf_array));
	
	// Initialization array of obstacles
	obs = (Obstacle *)malloc(4*sizeof(*obs));
	obs[0] = ob1;
	obs[1] = ob2;
	obs[2] = ob3;
	obs[3] = ob4;
	
	std::cout << "Initialization of PRM graph" << std::endl;
	// Initialization of PRM graph
	for(int i=0; i < N_CONFIG; i++)
	{
		// Sampling Strategy : Uniform Distribution
		config_sample.x = (rand() % WINDOW) - origin.x;
		config_sample.y = (rand() % WINDOW) - origin.y;
		
		// GJK Collision Detection
		// config_sample is collision-free?
	  	if(gjk.checkAllCollision(config_sample, mobile, obs, 4) == NO_COLLISION)
	  	{
	  		// Configuration is collision-free -> add to graph
	  		conf_array[num_conf] = config_sample;
	  		//update number of nodes
	  		num_conf++;
	  		// config.draw(NO_COLLISION);
	  		circle(
				env_image,
				cv::Point(config_sample.x + origin.x, config_sample.y + origin.y),
				3,
				cv::Scalar(204,204,204),
				1
			);
	  	} 
	  	else 
	  	{
	  		// Configuration isn't collision-free -> NO add to graph
	  		// config.draw(COLLISION);
	  		circle(
	  			env_image,
	  			cv::Point(config_sample.x + origin.x, config_sample.y + origin.y),
	  			3,
	  			cv::Scalar(0,0,255),
	  			1
	  		);
	  	}
	}
  	
  	std::cout << "ARRAY OF FREE-COLLISION CONF :" << std::endl;
  	for(int i=0; i<num_conf; i++)
  	{
  		std::cout << "\t[" << conf_array[i].x <<", "<< conf_array[i].y << "]" << std::endl;
  	}
  	std::cout << "num_conf = " << num_conf << std::endl;
  	// Create PRM Roadmap
  	// Connection Strategy: kd-tree data structure
  	//	->	kd-tree construction
  	enum treeDirection dir = LEFT;
  	struct kdTreeNode tree;
  	struct kdTreeNode *treePointer;
  	tree.axis = X;
  	// search median
  	
  	int right = num_conf-1;
  	
  	
  	tree.value = medianXAxis(conf_array, 0, num_conf-1);
  	treePointer = &tree;
  	std::cout << "treePointer->value = " << treePointer->value << std::endl;
  	
  	line(
  		env_image,
  		cv::Point(origin.x + treePointer->value, 0),
  		cv::Point(origin.x + tree.value, WINDOW),
  		cv::Scalar(0,255,0),
  		1
  	);
  	
  	std::cout << "ARRAY OF FREE-COLLISION CONF :" << std::endl;
  	for(int i=0; i<num_conf; i++)
  	{
  		std::cout << "\t[" << conf_array[i].x <<", "<< conf_array[i].y << "]" << std::endl;
  	}
  	
  	int cicle = 1;
  	right = (num_conf-1)/2;
  	while(right >= 1) // more than one point
  	{
  		treePointer->left = (kdTreeNode *)malloc(sizeof(struct kdTreeNode));
	  	treePointer->left->parent = treePointer;
	  	treePointer = treePointer->left;
  		if(treePointer->parent->axis == X)
  		{
  			treePointer->axis = Y;
  			treePointer->value = medianYAxis(conf_array, 0, right);
  			std::cout << "treePointer->valueY = " << treePointer->value << std::endl;
  			
  			line(
		  		env_image,
		  		cv::Point(0, origin.y + treePointer->value),
		  		cv::Point(origin.x + treePointer->parent->value, origin.y + treePointer->value),
		  		cv::Scalar(0,255,0),
		  		1
  			);
  		} 
  		else
  		{
	  		treePointer->axis = X;
  			treePointer->value = medianXAxis(conf_array, 0, right);
  			std::cout << "treePointer->valueX = " << treePointer->value << std::endl;
  			
  			line(
		  		env_image,
		  		cv::Point(origin.x + treePointer->value, origin.y + treePointer->parent->value),
		  		cv::Point(origin.x + treePointer->value, 0),
		  		cv::Scalar(0,255,0),
		  		1
  			);
  		}
  		
  		std::cout << "ARRAY OF FREE-COLLISION CONF :" << std::endl;
		for(int i=0; i<num_conf; i++)
		{
			std::cout << "\t[" << conf_array[i].x <<", "<< conf_array[i].y << "]" << std::endl;
		}
  		cicle++;
  		right = (num_conf-1)/pow(2, cicle);
  	}
  	std::cout << "pointLeft = [" << conf_array[right].x << ", " << conf_array[right].y << "]" << std::endl;
  	
  	
  	/*std::cout << "treePointer(tree) = " << treePointer << std::endl;
  	treePointer->left = (kdTreeNode *)malloc(sizeof(struct kdTreeNode));
  	std::cout << "treePointer->left = " << treePointer->left << std::endl;
  	treePointer->left->parent = treePointer;
  	std::cout << "treePointer->left->parent = " << treePointer->parent << std::endl;
  	treePointer = treePointer->left;
  	std::cout << "treePointer = " << treePointer << std::endl;	
  	
  	treePointer->axis = Y;
  	treePointer->value = medianYAxis(conf_array, 0, (num_conf-1)/2, (num_conf-1)/2-1);
  	std::cout << "treePointer->value = " << treePointer->value << std::endl;
  	
  	line(
  		env_image,
  		cv::Point(0, origin.y + treePointer->value),
  		cv::Point(origin.x + treePointer->parent->value, origin.y + treePointer->value),
  		cv::Scalar(0,255,0),
  		1
  	);
  	
  	std::cout << "ARRAY OF FREE-COLLISION CONF :" << std::endl;
  	for(int i=0; i<num_conf; i++)
  	{
  		std::cout << "\t[" << conf_array[i].x <<", "<< conf_array[i].y << "]" << std::endl;
  	}
  	
  	treePointer->left = (kdTreeNode *)malloc(sizeof(struct kdTreeNode));
  	treePointer->left->parent = treePointer;
  	treePointer = treePointer->left;
  	
  	treePointer->axis = X;
  	treePointer->value = medianXAxis(conf_array, 0, (num_conf-1)/4, (num_conf-1)/2-2);
  	std::cout << "treePointer->value = " << treePointer->value << std::endl;
  	
  	line(
  		env_image,
  		cv::Point(origin.x + treePointer->value, origin.y + treePointer->parent->value),
  		cv::Point(origin.x + treePointer->value, 0),
  		cv::Scalar(0,255,0),
  		1
  	);*/
  	
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

float medianXAxis(struct vec2_t *array, int left, int right)
{
	float result;
	quicksortXaxis(array, left, right);
	int med_index = (right-left+1)/2;
	result = array[med_index].x;
	if((right-left) % 2 != 0)
	{
		// even number of configuration
		result = (array[med_index-1].x + array[med_index].x)/2;
	}
	return result;
}

void quicksortXaxis(struct vec2_t *array, int l, int r)
{
	if (r-l <= 0)
	{
		return;
	}
	else
	{
		int i = partitionXAxis(array, l, r);
		quicksortXaxis(array, l, i-1);
		quicksortXaxis(array, i+1, r);
	}
}

int partitionXAxis(struct vec2_t *array, int l, int r)
{
	int i, pivot;
	pivot = array[r].x;
	i = l - 1;
	for(int j = l; j <= r-1; j++)
	{
		if(array[j].x < pivot)
		{
			i++;
			swap(array, i, j);
		}
	}
	swap(array, i+1, r);
	return i+1;
}

float medianYAxis(struct vec2_t *array, int left, int right)
{
	float result;
	quicksortYaxis(array, left, right);
	int med_index = (right-left+1)/2;
	result = array[med_index].y;
	if((right-left) % 2 != 0)
	{
		// even number of configuration
		result = (array[med_index-1].y + array[med_index].y)/2;
	}
	return result;
}

void quicksortYaxis(struct vec2_t *array, int l, int r)
{
	if (r-l <= 0)
	{
		return;
	}
	else
	{
		int i = partitionYAxis(array, l, r);
		quicksortYaxis(array, l, i-1);
		quicksortYaxis(array, i+1, r);
	}
}

int partitionYAxis(struct vec2_t *array, int l, int r)
{
	int i, pivot;
	pivot = array[r].y;
	i = l - 1;
	for(int j = l; j <= r-1; j++)
	{
		if(array[j].y < pivot)
		{
			i++;
			swap(array, i, j);
		}
	}
	swap(array, i+1, r);
	return i+1;
}

void swap(struct vec2_t *array, int i, int j)
{
	struct vec2_t tmp;
	tmp = array[i];
	array[i] = array[j];
	array[j] = tmp;
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
