#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "structures.h"

class Obstacle
{
	public:
		Obstacle();
		Obstacle(struct vec2_t *pts, int num);
		
		void draw(cv::Mat image, cv::Scalar color, struct vec2_t offset);
		void drawBoundaries(cv::Mat image, cv::Scalar color, int thick, struct vec2_t offset);
		struct vec2_t getCentre();
		struct vec2_t support(struct vec2_t direction);
		void move(struct vec2_t displacement);
		
	private:
		int num_vertices;
		struct vec2_t *vertices;
		struct vec2_t centre;
		
		float dotProduct(struct vec2_t v, struct vec2_t dir);
};

#endif
