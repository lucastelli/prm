#include "linear.h"

Linear::Linear(float a, float alpha, float d, float teta) : Joint(a, alpha, d, teta){}

void Linear::draw(cv::Mat image)
{
	std::cout << "drawing linear joint at position " << position << std::endl;
	
	cv::Scalar color = cv::Scalar(0, 0, 0);
	
	cv::Point rect_lt = cv::Point(-10, 5);
	cv::Point rect_lb = cv::Point(-10, -5);
	cv::Point rect_rt = cv::Point(10, 5);
	cv::Point rect_rb = cv::Point(10, -5);
	
	cv::Point slider_start = cv::Point(-10, 0);
	cv::Point slider_end = cv::Point(10, 0);
	
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
		cv::Scalar(255, 255, 255),
		cv::LINE_AA
	);
	
	line(
		image,
		rect_lt,
		rect_rt,
		color,
		LINEAR_THICKNESS,
		cv::LINE_AA
	);
	
	line(
		image,
		rect_lb,
		rect_rb,
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
	result.x = rotationMatrix[0][0]*vect.x + rotationMatrix[0][2]*vect.y;
	result.y = rotationMatrix[1][0]*vect.x + rotationMatrix[1][2]*vect.y;
	
	return result;
}
