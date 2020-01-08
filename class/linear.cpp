#include "linear.h"

Linear::Linear(float a, float alpha, float d, float teta) : Joint(a, alpha, d, teta){}

void Linear::draw(cv::Mat image)
{
	std::cout << "drawing linear joint at position: " << position << std::endl;
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
	
	cv::Point rect_lt = cv::Point(-5, 10);
	cv::Point rect_lb = cv::Point(-5, -10);
	cv::Point rect_rt = cv::Point(5, 10);
	cv::Point rect_rb = cv::Point(5, -10);
	
	cv::Point slider_start = cv::Point(0, -10);
	cv::Point slider_end = cv::Point(0, 10);
	
	rect_lt = rotate(rect_lt) + position; //rotate + traslate
	rect_lb = rotate(rect_lb) + position;
	rect_rt = rotate(rect_rt) + position;
	rect_rb = rotate(rect_rb) + position;
	
	cv::Point vertices[] = {rect_lt, rect_rt, rect_rb, rect_lb};
	
	slider_start = rotate(slider_start) + position;
	slider_end = rotate(slider_end) + position;
	
	fillConvexPoly(
		image,
		vertices,
		4,
		cv::Scalar(0, 0, 255),
		cv::LINE_AA
	);
	
	line(
		image,
		rect_lb,
		rect_lt,
		color,
		LINEAR_THICKNESS,
		cv::LINE_AA
	);
	
	line(
		image,
		rect_rb,
		rect_rt,
		color,
		LINEAR_THICKNESS,
		cv::LINE_AA
	);
	
	line(
		image,
		slider_start,
		slider_end,
		color,
		ARM_THICKNESS,
		cv::LINE_AA
	);
}

// Only 2D function
cv::Point Linear::rotate(cv::Point vect)
{
	cv::Point result;
	if(rotationMatrix[2][2] == 1)
	{
		result.x = rotationMatrix[0][0]*vect.x + rotationMatrix[0][1]*vect.y;
		result.y = rotationMatrix[1][0]*vect.x + rotationMatrix[1][1]*vect.y;
	} else {
		result.x = - rotationMatrix[1][0]*vect.x - rotationMatrix[1][2]*vect.y;
		result.y = rotationMatrix[0][0]*vect.x + rotationMatrix[0][2]*vect.y;
	}
	
	return result;
}
