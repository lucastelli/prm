#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "joint.h"
#include "rotoidal.h"
#include "linear.h"
#include "prmConfig.h"

// Window Parameters //
#define WINDOW 400

// Frame Reference Parameters //
#define FRAME_OFFSET 10
#define LABEL_OFFSET 2
#define AXIS_LENGTH 50
#define ARROW_LENGTH 6

// Define functions //
void updateMap(cv::Mat &map_x, cv::Mat &map_y);

// Define namespace //
using namespace cv;


// Main program //

int main(int argc, char** argv)
{
	// report version
   std::cout << argv[0] << " Version " << prm_VERSION_MAJOR << "."
              << prm_VERSION_MINOR << std::endl;
	
	/*----------------------------------------------
		Define 3 arm planar robot manipulator
			with DH convention
	----------------------------------------------*/
	
	float l1 = 50;
	float l2 = 50;
	float l3 = 50;
	
	float angle_1 = M_PI/6;
	float angle_2 = 0;
	float angle_3 = 0;
	
	Rotoidal j1_r(l1, 0, 0, angle_1);
	Rotoidal j2_r(l2, 0, 0, angle_2);
	Rotoidal j3_r(l3, 0, 0, angle_3);
	
	
	/*----------------------------------------------
		Display the environment: Robot, obstacles, 
			free robot configurations, optimal path
	----------------------------------------------*/
	
	char env_window[] = "PRM Environment";
	
	Mat env_image( WINDOW, WINDOW, CV_8UC3, Scalar(255,255,255));
  	Mat flip_env_image( WINDOW, WINDOW, CV_8UC3, Scalar(255,255,255));
  	Mat map_x(env_image.size(), CV_32FC1);
  	Mat map_y(env_image.size(), CV_32FC1);
  	
	// Flip vertical entire image
	updateMap(map_x, map_y);
	remap( env_image, flip_env_image, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0) );
	
	// Display the flip image
	imshow( env_window, flip_env_image );
	waitKey( 0 );
	return(0);
}

void updateMap(Mat &map_x, Mat &map_y)
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

