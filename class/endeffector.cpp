#include "endeffector.h"

EndEffector::EndEffector() : Joint(0, 0, 0, 0){}

EndEffector::EndEffector(float a, float alpha, float d, float teta) : Joint(a, alpha, d, teta){}

void EndEffector::draw(cv::Mat image)
{
	std::cout << "drawing end effector at position " << position << std::endl;
	
	cv::Scalar color = cv::Scalar(0, 0, 0);
	
	cv::Point end_finger_right_bottom = cv::Point(FINGER_LENGHT, 0);
	cv::Point end_finger_right_top = cv::Point(FINGER_LENGHT, FINGER_LENGHT);
	cv::Point end_finger_left_bottom = cv::Point(- FINGER_LENGHT, 0);
	cv::Point end_finger_left_top = cv::Point(- FINGER_LENGHT, FINGER_LENGHT);
	
	end_finger_right_bottom = rotate(end_finger_right_bottom) + position;
	end_finger_right_top = rotate(end_finger_right_top) + position;
	end_finger_left_bottom = rotate(end_finger_left_bottom) + position;
	end_finger_left_top = rotate(end_finger_left_top) + position;
	
	line( image,
      position,
      end_finger_right_bottom,
      color,
      ENDEFFECTOR_THICKNESS,
      cv::LINE_AA );
      
   line( image,
      end_finger_right_bottom,
      end_finger_right_top,
      color,
      ENDEFFECTOR_THICKNESS,
      cv::LINE_AA );
      
   line( image,
      position,
      end_finger_left_bottom,
      color,
      ENDEFFECTOR_THICKNESS,
      cv::LINE_AA );
      
   line( image,
      end_finger_left_bottom,
      end_finger_left_top,
      color,
      ENDEFFECTOR_THICKNESS,
      cv::LINE_AA );
}

// Only 2D function
cv::Point EndEffector::rotate(cv::Point vect)
{
	cv::Point result;
	result.x = rotationMatrix[1][0]*vect.x + rotationMatrix[1][1]*vect.y;
	result.y = -rotationMatrix[0][0]*vect.x - rotationMatrix[0][1]*vect.y;
	
	return result;
}
