#include "rotoidal.h"

Rotoidal::Rotoidal(float a, float alpha, float d, float teta) : Joint(a, alpha, d, teta){}

void Rotoidal::draw(cv::Mat image)
{
	std::cout << "drawing rotoidal joint at position " << position << std::endl;
	std::cout << "and orientation: " << std::endl;
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			std::cout << rotationMatrix[i][j] << "\t";
		}
		std::cout << std::endl;
	}
	
	cv::Scalar color = cv::Scalar(0, 0, 0);
	
	circle( image,
      position,
      RADIUS_OUTER_CIRCLE,
      cv::Scalar(255, 255, 255),
      -1, // Filled circle
      cv::LINE_AA );
      
   circle( image,
      position,
      RADIUS_OUTER_CIRCLE,
      color,
      ROTOIDAL_THICKNESS,
      cv::LINE_AA );
	
	circle( image,
      position,
      RADIUS_INNER_CIRCLE,
      color,
      ROTOIDAL_THICKNESS,
      cv::LINE_AA );
}
