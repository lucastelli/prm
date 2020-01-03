#include "rotoidal.h"

Rotoidal::Rotoidal(float a, float alpha, float d, float teta) : Joint(a, alpha, d, teta){}

void Rotoidal::draw(cv::Mat image)
{
	std::cout << "drawing rotoidal joint \n";
	
	cv::Scalar color = cv::Scalar(0, 0, 0);
	
	circle( image,
      position,
      0,
      color,
      THICKNESS,
      cv::LINE_8 );
	
	circle( image,
      position,
      RADIUS_INNER_CIRCLE,
      color,
      THICKNESS,
      cv::LINE_8 );
      
   circle( image,
      position,
      RADIUS_OUTER_CIRCLE,
      color,
      THICKNESS,
      cv::LINE_8 );
}
