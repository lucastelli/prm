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
#define WINDOW 600
	
// Display Map Parameters
#define MAP_WIDTH		500	//	width of the map (pixels)
#define MAP_HEIGHT	500	// height of the map (pixels)
#define UNIT			10		// unit of grid (pixels per unit)
#define RESOLUTION	1		// resolution of the map (pixels per meter)
#define MARGIN_RIGHT	20		// margin right of map (pixels)
#define MARGIN_TOP	20		// margin top of map (pixels)

// Frame Reference Parameters //
#define FRAME_OFFSET 5
#define LABEL_OFFSET 2
#define AXIS_LENGTH 	50
#define ARROW_LENGTH 6

// PRM Parameters //
#define N_CONFIG 		200	// total configuration
#define N_NEIGHBORS 	5		// number of neighbors for each node

// k-d Tree parameters and functions//
#define MAX_DIM 2

int n_index = 0;		//global variable -> index of neighbors array
int n_max = 0;			//global variable -> index of farther neighbor in the neighbors array

// kd-tree function
void swap(struct kd_node_t *a, struct kd_node_t *b);
struct kd_node_t * partition(struct kd_node_t *array_s, struct kd_node_t *array_e, int axis);
struct kd_node_t * median(struct kd_node_t *array_s, struct kd_node_t *array_e, int axis);
struct kd_node_t * kdtree_make(struct kd_node_t *array_s, struct kd_node_t *array_e, int axis);
void kdtree_stamp(struct kd_node_t *root);
float dist(struct kd_node_t *node_a, struct kd_node_t *node_b);
void kdtree_nearest(struct kd_node_t *root, struct kd_node_t *query, struct kd_node_t **best_near, float *distance, int index);
void kdtree_neighbors(struct kd_node_t *root, struct kd_node_t *query, struct kd_node_t **best_near, float *distance, int index,
								struct kd_node_t **neighbors, float dist_neighbors, float *local_max_distance);

// subdivision collision checking
bool scc(struct kd_node_t *qs, struct kd_node_t *qe, float step_min, cv::Mat img, struct vec2_t origin, GJKDetector *gjk, MobileRobot *mobile, Obstacle **obx, int num_obs);

// A* algorithm
bool search_optimal_path(struct kd_node_t *start, struct kd_node_t *goal, struct kd_node_t *roadmap, int num_nodes);

// Define drawing functions //
void updateMap(cv::Mat &map_x, cv::Mat &map_y);
cv::Point drawVector(cv::Mat img, struct vec2_t origin, double width, double angle, cv::Scalar color);
void drawLabel(cv::Mat img, std::string label, cv::Point origin);

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
  	
  	// Define and display reference frame of the map	
  	cv::Point xAxis, yAxis;
  	struct vec2_t origin = {WINDOW - MAP_WIDTH - MARGIN_RIGHT, WINDOW - MAP_HEIGHT - MARGIN_TOP};
  	
  	// Draw map border
  	line(
  		env_image,
  		cv::Point(origin.x, origin.y),
  		cv::Point(origin.x + MAP_WIDTH, origin.y),
  		cv::Scalar(0, 0, 0),
  		1
  	);
  	
  	line(
  		env_image,
  		cv::Point(origin.x + MAP_WIDTH, origin.y),
  		cv::Point(origin.x + MAP_WIDTH, origin.y + MAP_HEIGHT),
  		cv::Scalar(0, 0, 0),
  		1
  	);
  	
  	line(
  		env_image,
  		cv::Point(origin.x + MAP_WIDTH, origin.y + MAP_HEIGHT),
  		cv::Point(origin.x, origin.y + MAP_HEIGHT),
  		cv::Scalar(0, 0, 0),
  		1
  	);
  	
  	line(
  		env_image,
  		cv::Point(origin.x, origin.y + MAP_HEIGHT),
  		cv::Point(origin.x, origin.y),
  		cv::Scalar(0, 0, 0),
  		1
  	);
  	
  	// Draw x axes
  	int val;
  	for(int i=0; i <= MAP_WIDTH; i = i + UNIT)
  	{
  		val = 5;
  		if(!(i % (UNIT * 5)))
  			val = 10;
  		line(
  			env_image,
  			cv::Point(origin.x + i, origin.y),
  			cv::Point(origin.x + i, origin.y - val),
  			cv::Scalar(0, 0, 0),
  			1
  		);
  	}
  	// Draw y axes
  	for(int i=0; i <= MAP_HEIGHT; i = i + UNIT)
  	{
  		val = 5;
  		if(!(i % (UNIT * 5)))
  			val = 10;
  		line(
  			env_image,
  			cv::Point(origin.x, origin.y + i),
  			cv::Point(origin.x - val, origin.y + i),
  			cv::Scalar(0, 0, 0),
  			1
  		);
  	}
  	// Draw grid
  	cv::Scalar color_grid(217, 217, 217);
  	for(int i = UNIT * 5; i < MAP_WIDTH; i = i + UNIT * 5)
  	{
  		line(
  			env_image,
  			cv::Point(origin.x + i, origin.y),
  			cv::Point(origin.x + i, origin.y + MAP_HEIGHT),
  			color_grid,
  			1
  		);
  	}
  	for(int i = UNIT * 5; i < MAP_HEIGHT; i = i + UNIT * 5)
  	{
  		line(
  			env_image,
  			cv::Point(origin.x, origin.y + i),
  			cv::Point(origin.x + MAP_WIDTH, origin.y + i),
  			color_grid,
  			1
  		);
  	}
  	
  	
  	// Draw vector axis
  	xAxis = drawVector(env_image, origin, AXIS_LENGTH, 0, Scalar(255, 0, 0));
  	yAxis = drawVector(env_image, origin, AXIS_LENGTH, M_PI/2, Scalar(0, 0, 255));
  	
  	// Display Robot
  	//manipulator.computePose();
  	//manipulator.draw(env_image);
  	
#if 1
  	MobileRobot mobile({0, 0});
  	//mobile.getRotation();
  	//mobile.draw(env_image, origin);
  	
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
	
	GJKDetector gjk;						// GJK detector object
	struct vec2_t config_sample;		// Single configuration sampled by PRM sampling strategy
	Obstacle **obs;						// Array of obstacles in the work-space
	struct vec2_t *conf_array;			// Array of random configurations sampled by PRM sampling strategy
	struct vec2_t *free_conf_array;	// Array of free-collision configurations after GJK Collision Detection
	int num_conf = 0;						// number of free-collision configurations
	
	// initialization of random configurations array													
	conf_array = (struct vec2_t *)calloc(N_CONFIG, sizeof(*conf_array));
	
	// initialization obstacles array
	obs = (Obstacle **)calloc(4, sizeof(*obs));
	obs[0] = &ob1;
	obs[1] = &ob2;
	obs[2] = &ob3;
	obs[3] = &ob4;
	
	std::cout << "Initialization of PRM graph" << std::endl;
	// Initialization of PRM graph
	while(num_conf < N_CONFIG)
	{
		// Sampling Strategy : Uniform Distribution
		config_sample.x = rand()/((RAND_MAX + 1u)/MAP_WIDTH);
		config_sample.y = rand()/((RAND_MAX + 1u)/MAP_HEIGHT);
		
		// GJK Collision Detection
		// config_sample is collision-free?
	  	if(gjk.checkAllCollision(&config_sample, &mobile, obs, 4) == NO_COLLISION)
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
  	
  	#if 1
  	// initialization array of free-collision configuration
  	free_conf_array = (struct vec2_t *)calloc(num_conf, sizeof(*free_conf_array));
  	
  	// copy values
  	for(int i=0; i<num_conf; i++)
  	{
  		free_conf_array[i] = conf_array[i];
  	}
  	
  	// eliminate conf_array and free memory
  	free(conf_array);
  	
  	/*std::cout << "ARRAY OF FREE-COLLISION CONFIGURATION :" << std::endl;
  	for(int i=0; i<num_conf; i++)
  	{
  		std::cout << i <<" :"<< "\t[" << free_conf_array[i].x <<", "<< free_conf_array[i].y << "]" << std::endl;
  	}
  	std::cout << "num_conf = " << num_conf << std::endl;*/
  	#endif
  	
  	// Create PRM Roadmap
  	// Connection Strategy: kd-tree data structure
  	//	->	kd-tree construction
  	
  	//struct kd_node_t nodes[num_conf];
  	struct kd_node_t roadmap[num_conf];
  	
  	// create node for all free-collision configurations
  	for(int i=0; i < num_conf; i++)
  	{
  		roadmap[i].x[0] = free_conf_array[i].x;
  		roadmap[i].x[1] = free_conf_array[i].y;
  		// initialization of values for A* algorithm
  		roadmap[i].heur = 0;
		roadmap[i].back_path_length = 0;
		roadmap[i].parent = nullptr;
  	}
  	
  	struct kd_node_t *root;
	printf("KD TREE CREATE\n");
	root = kdtree_make(roadmap, roadmap + num_conf, 0);
	kdtree_stamp(root);
	
  	struct kd_node_t **nearest;
	nearest = (struct kd_node_t **)calloc(1, sizeof(*nearest));
	
	// Construct roadmap's parameters
	float dist_neigh = 200;	// maximum distance for neighbors
	float step_size = 10;	// maximum step length for subdivision collision checking
	
	for(int i=0; i < num_conf; i++)
	{
		printf("%d : [%f, %f] kd_ptr = %p\n", i, roadmap[i].x[0], roadmap[i].x[1], &roadmap[i]);
	}
	
	// Creation of roadmap
	float dist;
	float local_distance;
	struct kd_node_t **neighbors;
	
	for(int i=0; i < num_conf; i++)
	{
		dist = INFINITY;
		n_index = 0;
		local_distance = 0;
		n_max = 0;
		neighbors = (struct kd_node_t **)calloc(N_NEIGHBORS, sizeof(*neighbors));
		
		printf("-------START TO CONNECT------------\n");
		printf(
			"point [%p]\n\t[%f, %f]\n\tleft -> %p\n\tright -> %p\n", 
			&roadmap[i],
			roadmap[i].x[0],
			roadmap[i].x[1],
			roadmap[i].left,
			roadmap[i].right
		);
		// search neighbors of node with kd-tree
		kdtree_neighbors(root, &roadmap[i], nearest, &dist, 0, neighbors, dist_neigh, &local_distance);
		for(int j=0; j < N_NEIGHBORS; j++)
		{
			if(neighbors[j])
			{
				printf(
					"neigh [%p]\n\t[%f, %f]\n\tleft -> %p\n\tright -> %p\n", 
					neighbors[j],
					(neighbors[j]) ? neighbors[j]->x[0] : 0,
					(neighbors[j]) ? neighbors[j]->x[1] : 0,
					neighbors[j]->left,
					neighbors[j]->right
				);
				
				if(scc(&roadmap[i], neighbors[j], step_size, env_image, origin, &gjk, &mobile, obs, 4))
				{
					// add only valid neighbor into roadmap graph
					roadmap[i].neighbors.push_back(neighbors[j]);
					neighbors[j]->neighbors.push_back(&roadmap[i]);
					
					line(
						env_image,
						cv::Point(roadmap[i].x[0]+origin.x, roadmap[i].x[1]+origin.y),
						cv::Point(neighbors[j]->x[0]+origin.x, neighbors[j]->x[1]+origin.y),
						cv::Scalar(128, 128, 128),
						1
					);
				}
				else
				{
					line(
						env_image,
						cv::Point(roadmap[i].x[0]+origin.x, roadmap[i].x[1]+origin.y),
						cv::Point(neighbors[j]->x[0]+origin.x, neighbors[j]->x[1]+origin.y),
						cv::Scalar(0,0,255),
						1
					);
				}
			}
		}
		free(neighbors);
	}
	
	// Solve query algorithm
	
	struct kd_node_t start = {{0, 0}};
	struct kd_node_t goal = {{450, 50}};
	start.parent = nullptr;
	goal.parent = nullptr;
	
	// Search neighbors for start
	struct kd_node_t **s_neigh;
	s_neigh = (struct kd_node_t **)calloc(N_NEIGHBORS, sizeof(*s_neigh));
	
	dist = INFINITY;
	n_index = 0;
	local_distance = 0;
	n_max = 0;
	kdtree_neighbors(root, &start, nearest, &dist, 0, s_neigh, dist_neigh, &local_distance);
	
	// Check connection from start to node which is a start's neighbor node in the roadmap
	for(int j=0; j < N_NEIGHBORS; j++)
	{
		if(s_neigh[j])
		{
			if(scc(&start, s_neigh[j], step_size, env_image, origin, &gjk, &mobile, obs, 4))
			{
				start.neighbors.push_back(s_neigh[j]);
				s_neigh[j]->neighbors.push_back(&start);
				
				line(
					env_image,
					cv::Point(start.x[0]+origin.x, start.x[1]+origin.y),
					cv::Point(s_neigh[j]->x[0]+origin.x, s_neigh[j]->x[1]+origin.y),
					cv::Scalar(255, 0, 255),
					1
				);
			}
			else
			{
				line(
					env_image,
					cv::Point(start.x[0]+origin.x, start.x[1]+origin.y),
					cv::Point(s_neigh[j]->x[0]+origin.x, s_neigh[j]->x[1]+origin.y),
					cv::Scalar(0,0,255),
					1
				);
			}
		}
	}
	
	// Search neighbors for goal
	struct kd_node_t **g_neigh;
	g_neigh = (struct kd_node_t **)calloc(N_NEIGHBORS, sizeof(*g_neigh));
	
	dist = INFINITY;
	n_index = 0;
	local_distance = 0;
	n_max = 0;
	kdtree_neighbors(root, &goal, nearest, &dist, 0, g_neigh, dist_neigh, &local_distance);
	
	// Check connection from goal to node which is a goal's neighbor node in the roadmap
	for(int j=0; j < N_NEIGHBORS; j++)
	{
		if(g_neigh[j])
		{
			if(scc(&goal, g_neigh[j], step_size, env_image, origin, &gjk, &mobile, obs, 4))
			{
				goal.neighbors.push_back(g_neigh[j]);
				g_neigh[j]->neighbors.push_back(&goal);
				
				line(
					env_image,
					cv::Point(goal.x[0]+origin.x, goal.x[1]+origin.y),
					cv::Point(g_neigh[j]->x[0]+origin.x, g_neigh[j]->x[1]+origin.y),
					cv::Scalar(255, 0, 255),
					1
				);
			}
			else
			{
				line(
					env_image,
					cv::Point(goal.x[0]+origin.x, goal.x[1]+origin.y),
					cv::Point(g_neigh[j]->x[0]+origin.x, g_neigh[j]->x[1]+origin.y),
					cv::Scalar(0,0,255),
					1
				);
			}
		}
	}
	
	// A* Algorithm
	#if 1
	bool check_shorter_path = false;
	if(search_optimal_path(&start, &goal, roadmap, num_conf))
	{
		check_shorter_path = true;
		puts("\n PATH FIND:");
		struct kd_node_t *ptr = &goal;
		while(ptr)
		{
			printf("\t%p [%f, %f]", ptr, ptr->x[0], ptr->x[1]);
			ptr = ptr->parent;
			if(ptr)
				printf(" <- ");
		}
		puts("");
		
		ptr = &goal;
		while(ptr)
		{
			if(ptr->parent)
			{
				line(
					env_image,
					cv::Point(ptr->x[0]+origin.x, ptr->x[1]+origin.y),
					cv::Point(ptr->parent->x[0]+origin.x, ptr->parent->x[1]+origin.y),
					cv::Scalar(0,255,0),
					2
				);
			}
			ptr = ptr->parent;
		}
	} 
	else
	{
		puts("PATH NOT FOUND");
	}
	#endif
	
	// start
	circle(
		env_image,
		cv::Point(start.x[0]+origin.x, start.x[1]+origin.y),
		5,
		cv::Scalar(0, 255, 0),
		-1
	);
	
	//end
	circle(
		env_image,
		cv::Point(goal.x[0]+origin.x, goal.x[1]+origin.y),
		5,
		cv::Scalar(0, 0, 255),
		-1
	);
	
	if(check_shorter_path)
	{
		// Postprocessing Queries
		struct kd_node_t *scan_path_ptr, *tmp_ptr, *node_index;
		node_index = &goal;
		scan_path_ptr = goal.parent;
		tmp_ptr = scan_path_ptr;
		while(scan_path_ptr)
		{
			if(scc(scan_path_ptr, node_index, step_size, env_image, origin, &gjk, &mobile, obs, 4))
			{
				tmp_ptr = scan_path_ptr;
				scan_path_ptr = scan_path_ptr->parent;
			}
			else
			{
				node_index->parent = tmp_ptr;
				node_index = tmp_ptr;
			}
		}
		node_index->parent = tmp_ptr;
		node_index = tmp_ptr;
		
		puts("\n SHORTER PATH FIND:");
		struct kd_node_t *ptr_stamp = &goal;
		while(ptr_stamp)
		{
			printf("\t%p [%f, %f]", ptr_stamp, ptr_stamp->x[0], ptr_stamp->x[1]);
			ptr_stamp = ptr_stamp->parent;
			if(ptr_stamp)
				printf(" <- ");
		}
		puts("");
		
		ptr_stamp = &goal;
		while(ptr_stamp)
		{
			if(ptr_stamp->parent)
			{
				line(
					env_image,
					cv::Point(ptr_stamp->x[0]+origin.x, ptr_stamp->x[1]+origin.y),
					cv::Point(ptr_stamp->parent->x[0]+origin.x, ptr_stamp->parent->x[1]+origin.y),
					cv::Scalar(255,0,0),
					2
				);
			}
			ptr_stamp = ptr_stamp->parent;
		}
	}
#endif
  	
	// Flip vertical entire image
	updateMap(map_x, map_y);
	remap( env_image, flip_env_image, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0) );
	
	// Display the vector labels of all frames reference
	drawLabel(flip_env_image, "x", xAxis);
	drawLabel(flip_env_image, "y", yAxis);
	
	// Display grid labels
	char s_number[5];
	cv::Size txt_size;
	int baseline = 0;
	for(int i=0; i <= MAP_WIDTH; i = i + UNIT * 5)
  	{
		sprintf(s_number, "%d", i/RESOLUTION);
		txt_size = getTextSize(s_number, FONT_HERSHEY_SIMPLEX, 0.35, 1, &baseline);
		putText(
			flip_env_image, 
			s_number, 
			cv::Point(origin.x + i - txt_size.width/2, MAP_HEIGHT + 20 + MARGIN_TOP),
			FONT_HERSHEY_SIMPLEX,
			0.35,  
			cv::Scalar(0,0,0), 
			1
		);
  	}
  	for(int i=0; i <= MAP_HEIGHT; i = i + UNIT * 5)
  	{
		int offset = 0;
		int num_char = sprintf(s_number, "%d", i/RESOLUTION);
		txt_size = getTextSize(s_number, FONT_HERSHEY_SIMPLEX, 0.35, 1, &baseline);
		if(num_char > 1)
			offset = txt_size.width/2;
		putText(
			flip_env_image, 
			s_number, 
			cv::Point(WINDOW - MAP_WIDTH - MARGIN_RIGHT - 20 - offset, MAP_HEIGHT - i + MARGIN_TOP + txt_size.height/2),
			FONT_HERSHEY_SIMPLEX,
			0.35,  
			cv::Scalar(0,0,0), 
			1
		);
  	}
	
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 1

/*float dist(struct kd_node_t *node_a, struct kd_node_t *node_b)
{
	struct vec2_t r;
	r = node_a->value - node_b->value;
	return sqrt(r.x*r.x + r.y*r.y);
}*/

bool contains(std::vector<struct kd_node_t*> *array, struct kd_node_t *element)
{
	for(int i = 0; i < array->size(); i++)
	{
		if(array->at(i) == element)
			return 1;
	}
	return 0;
}

struct kd_node_t * pick_n_best(std::vector<struct kd_node_t*> *queue)
{
	struct kd_node_t *node, *best_node = nullptr;
	double node_path_length, length = INFINITY;
	int index = 0;
	for(int i = 0; i < queue->size(); i++)
	{
		node = queue->at(i);
		node_path_length = node->back_path_length + node->heur;
		if(node_path_length < length)
		{
			length = node_path_length;
			best_node = node;
			index = i;
		}
	}
	queue->erase(queue->begin() + index);
	return best_node;
}

bool search_optimal_path(struct kd_node_t *start, struct kd_node_t *goal, struct kd_node_t *roadmap, int num_nodes)
{
	std::vector<struct kd_node_t*> priority_queue;
	std::vector<struct kd_node_t*> visited_nodes;
	struct kd_node_t *n_best = nullptr, *neigh = nullptr;

	for (int i = 0; i < num_nodes; i++)
	{
		roadmap[i].heur = dist(roadmap + i, goal);
	}

	priority_queue.push_back(start);
	
	while(priority_queue.size() > 0)
	{
		n_best = pick_n_best(&priority_queue);
		printf("\tn_best = [%p] [%f, %f]\n", n_best, n_best->x[0], n_best->x[1]);
		visited_nodes.push_back(n_best);

		printf("Q = { ");
		for (int i = 0; i < priority_queue.size(); i++)
		{
			printf("%p ", priority_queue.at(i));
		}
		printf("}\n");

		printf("V = { ");
		for (int i = 0; i < visited_nodes.size(); i++)
		{
			printf("%p ", visited_nodes.at(i));
		}
		printf("}\n");

		if (n_best == goal)
			return 1;

		for (int i = 0; i < n_best->neighbors.size(); i++)
		{
			neigh = n_best->neighbors.at(i);
			if (!contains(&visited_nodes, neigh))
			{
				if (!contains(&priority_queue, neigh))
				{
					neigh->parent = n_best;
					neigh->back_path_length = n_best->back_path_length + dist(neigh, n_best);
					priority_queue.push_back(neigh);
				}
				else if(n_best->back_path_length + dist(n_best, neigh) < neigh->back_path_length)
				{
					neigh->parent = n_best;
					neigh->back_path_length = n_best->back_path_length + dist(neigh, n_best);
				}
			}
		}

		printf("Q = { ");
		for (int i = 0; i < priority_queue.size(); i++)
		{
			printf("%p ", priority_queue.at(i));
		}
		printf("}\n");
	}

	return 0;
}
#endif

// Subdivision Collision Checking
bool scc(struct kd_node_t *qs, struct kd_node_t *qe, float step_min, cv::Mat img, struct vec2_t origin, GJKDetector *gjk, MobileRobot *mobile, Obstacle **obx, int num_obs)
{
	struct kd_node_t qi;
	float step_x, step_y, step_mod;
	int i=0;
	step_x = qe->x[0] - qs->x[0];
	step_y = qe->x[1] - qs->x[1];
	while(1)
	{
		step_x = step_x/2;
		step_y = step_y/2;
		step_mod = sqrt((step_x * step_x)+(step_y * step_y));
		//printf("step_x = %f\nstep_y = %f\nstep_mod =%f\n", step_x, step_y, step_mod);
		if(step_mod < step_min)
			return 1;
		//printf("qi = {\n");	
		for(int j=0; j<pow(2,i); j++)
		{
			qi.x[0] = (2*j+1)*step_x + qs->x[0];
			qi.x[1] = (2*j+1)*step_y + qs->x[1];
			
			/*circle(
				img,
				cv::Point(qi.x[0]+origin.x, qi.x[1]+origin.y),
				3,
				cv::Scalar(255, 0, 255),
				1
			);*/
			//printf("\t[%f, %f]\n", qi.x[0], qi.x[1]);
			// check configuration qi with GJK
			struct vec2_t qi_config;
			qi_config.x = qi.x[0];
			qi_config.y = qi.x[1];
			
			if(gjk->checkAllCollision(&qi_config, mobile, obx, num_obs) == COLLISION_FOUND)
				return 0;
		}
		//printf("}\n");
		i++;
	}
}

void swap(struct kd_node_t *a, struct kd_node_t *b)
{
	int tmp[MAX_DIM];
	memcpy(tmp, a->x, sizeof(tmp));
	memcpy(a->x, b->x, sizeof(tmp));
	memcpy(b->x, tmp, sizeof(tmp));
}

struct kd_node_t * partition(struct kd_node_t *array_s, struct kd_node_t *array_e, int axis)
{
	struct kd_node_t *p, *store;
	int pivot = (array_e - 1)->x[axis]; // pick the coordinate of array's last node as pivot
	p = array_s;
	store = array_s;
	if(array_s == array_e)	//array compose by single element (leaf case of kd-tree)
	{
		return array_s;
	}
	if(array_e < array_s)
	{
		return NULL;
	}
	while(p < (array_e-1))
	{
		if(p->x[axis] < pivot)
		{
			if(p != store)
			{
				swap(p, store);
			}
			store++;
		}
		p++;
	}
	if(store != (array_e - 1))
		swap(store, (array_e - 1));
	return store;
}

struct kd_node_t * median(struct kd_node_t *array_s, struct kd_node_t *array_e, int axis)
{
	struct kd_node_t *part, *md, *start, *end;
	part = NULL;									// result of partition function
	start = array_s;								// left limit of array (change during the cycle)
	end = array_e;									// right limit of array (change during the cycle)
	md = array_s + (array_e - array_s)/2;	// pointer of array's median
	
	if(array_e <= array_s)	//array compose by single element (leaf case of kd-tree)
	{
		return NULL;
	}
	while(1)
	{
		part = partition(start, end, axis);
		if(part < md)
		{
			// update right limit of array
			start = part + 1;
		}
		else if(part > md)
		{
			// update left limit of array
			end = part;
		}
		else
		{
			// part == md -> return median
			return part;
		}
	}
	return part;
}

struct kd_node_t * kdtree_make(struct kd_node_t *array_s, struct kd_node_t *array_e, int axis)
{
	struct kd_node_t *node;
	int axis_update;
	
	if((node = median(array_s, array_e, axis)))
	{
		axis_update = (axis + 1) % MAX_DIM;
		node->left = kdtree_make(array_s, node, axis_update);
		node->right = kdtree_make(node + 1, array_e, axis_update);
	}
		
	return node;
}

void kdtree_stamp(struct kd_node_t *root)
{
	if(root)
	{
		printf(
			"node [%p]\n\t[%f, %f]\n\tleft -> %p\n\tright -> %p\n", 
			root,
			(root) ? root->x[0] : 0,
			(root) ? root->x[1] : 0,
			root->left,
			root->right
		);
		if(root->left)
			kdtree_stamp(root->left);
		if(root->right)
			kdtree_stamp(root->right);
	}
}

float dist(struct kd_node_t *node_a, struct kd_node_t *node_b)
{
	float x, y;
	x = node_a->x[0] - node_b->x[0];
	y = node_a->x[1] - node_b->x[1];
	return sqrt(x*x + y*y);
}

void kdtree_nearest(struct kd_node_t *root, struct kd_node_t *query, struct kd_node_t **best_near, float *distance, int index)
{
	float d;
	
	if(!root)
	{
		return;
	}
	
	d = dist(root, query);
	
	if(d < *distance)
	{
		*distance = d; // the best short distance in distance pointer
		*best_near = root;	// the nearest node
	}
	
	if(!*distance)
	{
		// query == root
		return;
	}
	
	if(query->x[index] < root->x[index])
	{
		// search first left
		if(root->left)
		{
			kdtree_nearest(root->left, query, best_near, distance, (index+1) % MAX_DIM);
		}
	}
	else
	{
		// search first right
		if(root->right)
		{
			kdtree_nearest(root->right, query, best_near, distance, (index+1) % MAX_DIM);
		}
	}
	
	if(abs(root->x[index] - query->x[index]) >= *distance)
	{
		return;
	}
	
	if(query->x[index] < root->x[index])
	{
		// search also right
		if(root->right)
		{
			kdtree_nearest(root->right, query, best_near, distance, (index+1) % MAX_DIM);
		}
	}
	else
	{
		// search also left
		if(root->left)
		{
			kdtree_nearest(root->left, query, best_near, distance, (index+1) % MAX_DIM);
		}
	}
}

int get_max(struct kd_node_t *query, struct kd_node_t **neighbors, float *local_max_distance)
{
	int idx_max = 0;
	float max_dist = 0;
	float d;
	for(int i=0; i < N_NEIGHBORS; i++)
	{
		d = dist(query, neighbors[i]);
		if(d > max_dist)
		{
			max_dist = d;
			idx_max = i;
		}
	}
	*local_max_distance = max_dist;
	return idx_max;
}

void kdtree_neighbors(struct kd_node_t *root, struct kd_node_t *query, struct kd_node_t **best_near, float *distance, int index,
								struct kd_node_t **neighbors, float dist_neighbors, float *local_max_distance)
{
	float d;
	
	if(!root)
	{
		return;
	}
	
	d = dist(root, query);
	
	if((d <= dist_neighbors) && (d != 0))
	{
		if(n_index < N_NEIGHBORS)
		{
			neighbors[n_index] = root;
			if(d > *local_max_distance)
			{
				*local_max_distance = d;
				n_max = n_index;
			}
			n_index++;
		}
		else
		{
			if(d < *local_max_distance)
			{
				neighbors[n_max] = root;
				n_max = get_max(query, neighbors, local_max_distance);
			}
		}
		// this node probably isn't the nearest one but respect distance neighbors criteria
		// so I insert it in the array of pointers of neighbors
		/*if(d <= *local_max_distance)
		{
			neighbors[n_index] = root;
			*local_max_distance = d;
			n_index = (n_index + 1) % N_NEIGHBORS;
		}*/
		
	}
	
	if((d < *distance) && (d != 0))
	{
		*distance = d; // the best short distance in distance pointer
		*best_near = root;	// the nearest node
	}
	
	if(query->x[index] < root->x[index])
	{
		// search first left
		if(root->left)
		{
			kdtree_neighbors(root->left, query, best_near, distance, (index+1) % MAX_DIM, neighbors, dist_neighbors, local_max_distance);
		}
	}
	else
	{
		// search first right
		if(root->right)
		{
			kdtree_neighbors(root->right, query, best_near, distance, (index+1) % MAX_DIM, neighbors, dist_neighbors, local_max_distance);
		}
	}
	
	if(abs(root->x[index] - query->x[index]) >= dist_neighbors)
	{
		return;
	}
	
	if(query->x[index] < root->x[index])
	{
		// search also right
		if(root->right)
		{
			kdtree_neighbors(root->right, query, best_near, distance, (index+1) % MAX_DIM, neighbors, dist_neighbors, local_max_distance);
		}
	}
	else
	{
		// search also left
		if(root->left)
		{
			kdtree_neighbors(root->left, query, best_near, distance, (index+1) % MAX_DIM, neighbors, dist_neighbors, local_max_distance);
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
