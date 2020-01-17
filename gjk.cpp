/*-------------------------------------------------
		GJK Collision Detection Algorithm TEST
-------------------------------------------------*/

#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "obstacle.h"
#include "mobilerobot.h"
#include "structures.h"

// Window Parameters //
#define WINDOW 400

// Frame Reference Parameters //
#define FRAME_OFFSET 5
#define LABEL_OFFSET 2
#define AXIS_LENGTH 50
#define ARROW_LENGTH 6

using namespace cv;

cv::Point drawVector(cv::Mat img, struct vec2_t origin, double width, double angle, cv::Scalar color);
void updateMap(cv::Mat &map_x, cv::Mat &map_y);
void drawLabel(cv::Mat img, std::string label, cv::Point origin);
//function gjk v1.0
struct vec2_t getSupport(struct vec2_t direction, MobileRobot mobile, Obstacle obj);
struct vec2_t getSupport(struct vec2_t direction, Obstacle ob1, Obstacle ob2);
bool search(struct vec2_t *vett, struct vec2_t pt, int num);
struct vec3_t vectorProduct(struct vec3_t a, struct vec3_t b);
struct vec2_t normalize(struct vec2_t vett);
struct vec3_t normalize(struct vec3_t vett);
float dotProduct(struct vec2_t v, struct vec2_t dir);
//function gjk v2.0

enum gjkState{
  		UPDATE_SIMPLEX,
  		NO_COLLISION,
  		COLLISION_FOUND
  	};
  	
enum gjkState updateSimplex(struct vec2_t &direction, std::vector<struct vec2_t> *simplex_vertices, Obstacle r, Obstacle ob);
enum gjkState addSupport(struct vec2_t direction, std::vector<struct vec2_t> *simplex_vertices, Obstacle r, Obstacle ob);

int main()
{
	char mink_window[] = "Minkowski Difference";
	
	cv::Mat mink_image( WINDOW, WINDOW, CV_8UC3, cv::Scalar(255,255,255));
  	cv::Mat flip_mink_image( WINDOW, WINDOW, CV_8UC3, cv::Scalar(255,255,255));
  	cv::Mat map_x(mink_image.size(), CV_32FC1);
  	cv::Mat map_y(mink_image.size(), CV_32FC1);
  	
  	// Define and display reference frame	
  	cv::Point xAxis, yAxis;
  	struct vec2_t origin(200, 200);
  	
  	xAxis = drawVector(mink_image, origin, AXIS_LENGTH, 0, Scalar(255, 0, 0));
  	yAxis = drawVector(mink_image, origin, AXIS_LENGTH, M_PI/2, Scalar(0, 0, 255));
  	
  	struct vec2_t ob_pts1[] = {	
  											{50, 50}, 
  											{50, 100},
  											{85, 135},
  											{135, 135},
  											{170, 100},
  											{170, 50},
  											{135, 15},
  											{85, 15}
  										};
  	struct vec2_t robot[] = {
  											{0, 0},
  											{0, 20},
  											{20, 20},
  											{20, 0}
  										};
  	Obstacle ob = Obstacle(ob_pts1, 8);
  	Obstacle r = Obstacle(robot, 4);
  	
  	r.move(vec2_t(0, 0));
  	
  	ob.draw(mink_image, cv::Scalar(0,0,0), origin);
  	r.draw(mink_image, cv::Scalar(255,0,0), origin);
  	
  	// Minkowsky Difference point to point
  	#if 0
  	struct vec2_t *mink_diff1, *mink_diff2, *mink_diff3, *mink_diff4;
  	mink_diff1 = (vec2_t *)malloc(8*sizeof(*mink_diff1));
  	mink_diff2 = (vec2_t *)malloc(8*sizeof(*mink_diff2));
  	mink_diff3 = (vec2_t *)malloc(8*sizeof(*mink_diff3));
  	mink_diff4 = (vec2_t *)malloc(8*sizeof(*mink_diff4));
  	
  	for(int j=0; j<8; j++)
  	{
  		mink_diff1[j] = robot[0] - ob_pts1[j];
  	}
  	for(int j=0; j<8; j++)
  	{
  		mink_diff2[j] = robot[1] - ob_pts1[j];
  	}
  	for(int j=0; j<8; j++)
  	{
  		mink_diff3[j] = robot[2] - ob_pts1[j];
  	}
  	for(int j=0; j<8; j++)
  	{
  		mink_diff4[j] = robot[3] - ob_pts1[j];
  	}
  	
  	for(int i=0; i<8; i++)
  	{
  		mink_diff1[i] = mink_diff1[i] + origin;
  	}
  	for(int i=0; i<8; i++)
  	{
  		mink_diff2[i] = mink_diff2[i] + origin;
  	}
  	for(int i=0; i<8; i++)
  	{
  		mink_diff3[i] = mink_diff3[i] + origin;
  	}
  	for(int i=0; i<8; i++)
  	{
  		mink_diff4[i] = mink_diff4[i] + origin;
  	}
  	
  	Obstacle mink1 = Obstacle(mink_diff1, 8);
  	Obstacle mink2 = Obstacle(mink_diff2, 8);
  	Obstacle mink3 = Obstacle(mink_diff3, 8);
  	Obstacle mink4 = Obstacle(mink_diff4, 8);
  	mink1.draw(mink_image, cv::Scalar(0,0,0));
  	mink2.draw(mink_image, cv::Scalar(0,0,255));
  	mink3.draw(mink_image, cv::Scalar(0,255,0));
  	mink4.draw(mink_image, cv::Scalar(255,0,0));
  	# endif
  	
  	// Minkowsky Difference with support function for all directions
  	#if 1
  	int i = 0;
  	float angle = 2*M_PI;
  	struct vec2_t dir, vertex, *mink;
  	mink = (vec2_t *)malloc(10*sizeof(*mink));
  	
  	while(angle > 0)
  	{
  		dir.x = (angle == M_PI/2) ? 0 : cos(angle);
  		dir.y = sin(angle);
  		vertex = getSupport(dir, r, ob);
  		if(!search(mink, vertex, 10))
  		{
  			//mink[] doesn't cointain vertex
  			mink[i] = vertex;
  			i++;
  		}
  		angle -= 0.1;
  		
  		/*std::cout << "cycle = " << i << std::endl;
  		std::cout << "angle = " << angle << std::endl;
  		std::cout << "dir = [" << dir.x << ", " << dir.y << "]" << std::endl;
  		std::cout << "vertex = [" << vertex.x << ", " << vertex.y << "]" << std::endl << std::endl;
  		circle(
  			mink_image,
  			cv::Point(vertex.x+origin.x, vertex.y+origin.y),
  			3,
  			cv::Scalar(255,0,0),
  			1
  		);*/
  	}
  	
  	Obstacle mink_shape = Obstacle(mink, 8);
  	mink_shape.drawBoundaries(mink_image, cv::Scalar(0,0,0), 2, origin);
  	#endif
  	
  	//------------------------------
  	//	GJK version 1.0 found collision if obstacle colliding
  	//------------------------------
  	
  	#if 0
  	
  	//	GJK [create first simplex]
  	
  	std::cout << std::endl << "GJK [create first simplex]" << std::endl;
  	
  	struct vec2_t *simplex_vertices, a;
  	struct vec3_t tmp;
  	simplex_vertices = (struct vec2_t *)malloc(3*sizeof(*simplex_vertices));
  	// first simplex vertex
  	dir = r.getCentre() - ob.getCentre();
  	simplex_vertices[0] = getSupport(dir, r, ob);
  	std::cout << "dir = [" << dir.x << ", " << dir.y << "]" << std::endl;
  	std::cout << "v0 = [" << simplex_vertices[0].x << ", " << simplex_vertices[0].y << "]" << std::endl << std::endl;
  	circle(
  			mink_image,
  			cv::Point(simplex_vertices[0].x+origin.x, simplex_vertices[0].y+origin.y),
  			3,
  			cv::Scalar(255,0,0),
  			1
  	);
  	// second simplex vertex
  	dir = dir*(-1);
  	simplex_vertices[1] = getSupport(dir, r, ob);
  	std::cout << "dir = [" << dir.x << ", " << dir.y << "]" << std::endl;
  	std::cout << "v1 = [" << simplex_vertices[1].x << ", " << simplex_vertices[1].y << "]" << std::endl << std::endl;
  	circle(
  			mink_image,
  			cv::Point(simplex_vertices[1].x+origin.x, simplex_vertices[1].y+origin.y),
  			3,
  			cv::Scalar(255,0,0),
  			1
  	);
  	// third simplex vertex
  	a = simplex_vertices[0] - simplex_vertices[1];
  	tmp = vectorProduct(vec3_t(a.x, a.y, 0), vec3_t(-simplex_vertices[0].x, -simplex_vertices[0].y, 0));
  	tmp = vectorProduct(tmp, vec3_t(a.x, a.y, 0));
  	tmp = normalize(tmp);
  	simplex_vertices[2] = getSupport(vec2_t(tmp.x, tmp.y), r, ob);
  	std::cout << "tmp = [" << tmp.x << ", " << tmp.y << "]" << std::endl;
  	std::cout << "v2 = [" << simplex_vertices[2].x << ", " << simplex_vertices[2].y << "]" << std::endl << std::endl;
  	circle(
  			mink_image,
  			cv::Point(simplex_vertices[2].x+origin.x, simplex_vertices[2].y+origin.y),
  			3,
  			cv::Scalar(255,0,0),
  			1
  	);
  	#endif
  	
  	#if 0
  	
  	//	GJK [check if the origin is inside the simplex]
  	
  	std::cout << std::endl << "GJK [check if the origin is inside the simplex]" << std::endl;
  	
  	struct vec2_t v2_o, v2_v1, v2_v0;
  	struct vec3_t v2_v1_orth, v2_v0_orth;
  	bool collision = false;
  	
do{
	v2_o = simplex_vertices[2]*(-1);							//v2 to the origin
	v2_v1 = simplex_vertices[1] - simplex_vertices[2];	//v2 to v1
	v2_v0 = simplex_vertices[0] - simplex_vertices[2]; //v2 to v0
	  	
	std::cout << "v2_o = [" << v2_o.x << ", " << v2_o.y << "]" << std::endl;
	std::cout << "v2_v1 = [" << v2_v1.x << ", " << v2_v1.y << "]" << std::endl;
	std::cout << "v2_v0 = [" << v2_v0.x << ", " << v2_v0.y << "]" << std::endl;
		
	v2_v1_orth = vectorProduct(vec3_t(v2_v0.x, v2_v0.y, 0), vec3_t(v2_v1.x, v2_v1.y, 0));
	v2_v1_orth = normalize(v2_v1_orth);
	std::cout << "v2_v1_orth = [" << v2_v1_orth.x << ", " << v2_v1_orth.y << ", " << v2_v1_orth.z <<"]" << std::endl;
	v2_v1_orth = vectorProduct(v2_v1_orth, vec3_t(v2_v1.x, v2_v1.y, 0));
	v2_v1_orth = normalize(v2_v1_orth);
	std::cout << "v2_v1_orth = [" << v2_v1_orth.x << ", " << v2_v1_orth.y << ", " << v2_v1_orth.z <<"]" << std::endl;
		
	v2_v0_orth = vectorProduct(vec3_t(v2_v1.x, v2_v1.y, 0), vec3_t(v2_v0.x, v2_v0.y, 0));
	v2_v0_orth = normalize(v2_v0_orth);
	std::cout << "v2_v0_orth = [" << v2_v0_orth.x << ", " << v2_v0_orth.y << ", " << v2_v0_orth.z <<"]" << std::endl;
	v2_v0_orth = vectorProduct(v2_v0_orth, vec3_t(v2_v0.x, v2_v0.y, 0));
	v2_v0_orth = normalize(v2_v0_orth);
	std::cout << "v2_v0_orth = [" << v2_v0_orth.x << ", " << v2_v0_orth.y << ", " << v2_v0_orth.z <<"]" << std::endl;
		
	float dot1, dot2;
	v2_o = normalize(v2_o);
	std::cout << "v2_o_norm = [" << v2_o.x << ", " << v2_o.y << "]" << std::endl;
	dot1 = dotProduct(v2_o, vec2_t(v2_v1_orth.x, v2_v1_orth.y));
	dot2 = dotProduct(v2_o, vec2_t(v2_v0_orth.x, v2_v0_orth.y));
	std::cout << "dot1 = [" << dot1 << "]" << std::endl;
	std::cout << "dot2 = [" << dot2 << "]" << std::endl;
		
	if(dot1 > 0)
	{
		dir = vec2_t(v2_v1_orth.x, v2_v1_orth.y);
		simplex_vertices[0] = simplex_vertices[1];
		simplex_vertices[1] = simplex_vertices[2];
		simplex_vertices[2] = getSupport(dir, r, ob);
		circle(
  			mink_image,
  			cv::Point(simplex_vertices[2].x+origin.x, simplex_vertices[2].y+origin.y),
  			3,
  			cv::Scalar(0,0,255),
  			1
  		);
	} 
	else if(dot2 > 0)
	{
		dir = vec2_t(v2_v0_orth.x, v2_v0_orth.y);
		simplex_vertices[1] = simplex_vertices[2];
		simplex_vertices[2] = getSupport(dir, r, ob);
		circle(
  			mink_image,
  			cv::Point(simplex_vertices[2].x+origin.x, simplex_vertices[2].y+origin.y),
  			3,
  			cv::Scalar(0,255,0),
  			1
  		);
	}
	else
	{
		std::cout << "Collision!" << std::endl;
		collision = true;
	}
		
	std::cout << "dir = [" << dir.x << ", " << dir.y << "]" << std::endl;
	std::cout << "v2_o = [" << v2_o.x << ", " << v2_o.y << "]" << std::endl;
	std::cout << "dotProduct(dir, v2_o) = " << dotProduct(dir, v2_o) << std::endl;
	std::cout << std::endl;
}while(collision == false);
  	#endif
  	
  	//------------------------------
  	//	GJK version 2.0
  	//------------------------------
  	
  	std::vector<struct vec2_t> simplex_vertices;
  	struct vec2_t direction = {0, 0};
  	
  	enum gjkState result = UPDATE_SIMPLEX;
  	int cicle = 0;
  	
  	while(result == UPDATE_SIMPLEX)
  	{
  		result = updateSimplex(direction, &simplex_vertices, r, ob);
  		cicle++;
  		std::cout << "result " << result << std::endl << std::endl;
  	}
  	
  	if(result == COLLISION_FOUND)
  	{
  		std::cout << "Collision!" << std::endl;
  	} 
  	else 
  	{
  		std::cout << "NO Collision!" << std::endl;
  	}
  	
  	// Flip vertical entire image
	updateMap(map_x, map_y);
	remap( mink_image, flip_mink_image, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0) );
	
	// Display the vector labels of all frames reference
	drawLabel(flip_mink_image, "x", xAxis);
	drawLabel(flip_mink_image, "y", yAxis);
  	
  	imshow( mink_window, flip_mink_image );
  	waitKey(0);
}

bool search(struct vec2_t *vett, struct vec2_t pt, int num)
{
	bool result = false;
	for(int i=0; i<num; i++)
	{
		if(vett[i].x == pt.x && vett[i].y == pt.y)
			result = true;
	}
	return result;
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

//----------------------------------

enum gjkState updateSimplex(struct vec2_t &direction, std::vector<struct vec2_t> *simplex_vertices, Obstacle r, Obstacle ob)
{
	struct vec2_t a, v2_o, v2_v1, v2_v0, v2_v1_orth_2d, v2_v0_orth_2d;
	struct vec3_t tmp, v2_v1_orth, v2_v0_orth;
	float dot1, dot2;
	
	
	std::cout << "simplex_vertices.size() = " << simplex_vertices[0].size() << std::endl;
	switch(simplex_vertices[0].size())
	{
		case 0:	direction = r.getCentre() - ob.getCentre();
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

enum gjkState addSupport(struct vec2_t direction, std::vector<struct vec2_t> *simplex_vertices, Obstacle r, Obstacle ob)
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


