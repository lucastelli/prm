#include "endeffector.h"

EndEffector::EndEffector(float a, float alpha, float d, float teta) : Joint(a, alpha, d, teta){}

void EndEffector::draw(cv::Mat image)
{
	std::cout << "drawing end effector at position " << position << std::endl;
	
	cv::Scalar color = cv::Scalar(0, 0, 0);
	
	cv::Point end_finger_right_bottom = cv::Point(position.x + FINGER_LENGHT, position.y);
	cv::Point end_finger_right_top = cv::Point(position.x + FINGER_LENGHT, position.y + FINGER_LENGHT);
	cv::Point end_finger_left_bottom = cv::Point(position.x - FINGER_LENGHT, position.y);
	cv::Point end_finger_left_top = cv::Point(position.x - FINGER_LENGHT, position.y + FINGER_LENGHT);
	
	line( image,
      position,
      end_finger_right_bottom,
      color,
      THICKNESS,
      cv::LINE_8 );
      
   line( image,
      end_finger_right_bottom,
      end_finger_right_top,
      color,
      THICKNESS,
      cv::LINE_8 );
      
   line( image,
      position,
      end_finger_left_bottom,
      color,
      THICKNESS,
      cv::LINE_8 );
      
   line( image,
      end_finger_left_bottom,
      end_finger_left_top,
      color,
      THICKNESS,
      cv::LINE_8 );
}
