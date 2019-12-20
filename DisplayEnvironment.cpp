#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <string>
#include <regex>

#define _USE_MATH_DEFINES

// Window Parameters
#define WINDOW 400

// Frame Reference Parameters
#define FRAME_OFFSET 10
#define LABEL_OFFSET 2
#define AXIS_LENGTH 50
#define ARROW_LENGTH 6

using namespace cv;
void MyEllipse( Mat img, double angle );
void MyFilledCircle( Mat img, Point center );
void MyPolygon( Mat img );
void MyLine( Mat img, Point start, Point end );

void drawLabel(Mat img, std::string label, Point origin);
void updateMap(Mat &map_x, Mat &map_y);
Point drawVector(Mat img, Point origin, double width, double angle, Scalar color);
void drawZaxis(Mat img, Point origin, Scalar color);

int main( void ){
  char env_window[] = "Environment";
  
  Mat env_image( WINDOW, WINDOW, CV_8UC3, Scalar(255,255,255));
  Mat flip_env_image( WINDOW, WINDOW, CV_8UC3, Scalar(255,255,255));
  Mat map_x(env_image.size(), CV_32FC1);
  Mat map_y(env_image.size(), CV_32FC1);
  
  // Display Zero Frame Reference
  Point origin(FRAME_OFFSET, FRAME_OFFSET);
  Point xAxis = drawVector(env_image, origin, AXIS_LENGTH, 0, Scalar(255, 0, 0));
  Point yAxis = drawVector(env_image, origin, AXIS_LENGTH, M_PI/2, Scalar(0, 0, 255));
  //drawZaxis(env_image, origin, Scalar(0, 255, 0));
  
  // Display another Frame Reference
  Point org_vect1(WINDOW/2, WINDOW/2);
  Point vect1 = drawVector(env_image, org_vect1, AXIS_LENGTH, M_PI/4, Scalar(255, 0, 0));
  
  // Flip vertical entire image
  updateMap(map_x, map_y);
  remap( env_image, flip_env_image, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0) );
  
  // Display the vector labels of all frames reference
  drawLabel(flip_env_image, "x", xAxis);
  drawLabel(flip_env_image, "y", yAxis);
  drawLabel(flip_env_image, "z", origin);
  
  drawLabel(flip_env_image, "v_1", vect1);
  
  
  // Display the flip image
  imshow( env_window, flip_env_image );
  
  waitKey( 0 );
  return(0);
}

Point drawVector(Mat img, Point origin, double width, double angle, Scalar color)
{
	int thickness = 1;
  	int lineType = LINE_8;
  	
	Point vectorAxis(origin.x + width * cos(angle), origin.y + width * sin(angle));
	
	Point arrowAxis_l(vectorAxis.x - ARROW_LENGTH * cos(M_PI/6 - angle), vectorAxis.y + ARROW_LENGTH * sin(M_PI/6 - angle));
	Point arrowAxis_r(vectorAxis.x - ARROW_LENGTH * cos(M_PI/6 + angle), vectorAxis.y - ARROW_LENGTH * sin(M_PI/6 + angle));
  	
  	line( img,
    origin,
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

void drawZaxis(Mat img, Point origin, Scalar color)
{
	int thickness = 1;
	
	circle( img,
      origin,
      ARROW_LENGTH,
      color,
      thickness,
      LINE_8 );
}

void drawLabel(Mat img, std::string label, Point origin)
{	
	Point tmp(origin.x + LABEL_OFFSET, WINDOW - origin.y - LABEL_OFFSET);
	putText(img, label, tmp, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 1, LINE_AA);
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
