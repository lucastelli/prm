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
		
		void draw(cv::Mat image);
		struct vec2_t getCentre();
		struct vec2_t support(struct vec2_t direction);
		
	private:
		int num_vertices;
		struct vec2_t *vertices;
		struct vec2_t centre;
		
		float dotProduct(struct vec2_t v, struct vec2_t dir);
};

#endif
