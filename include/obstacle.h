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
		Obstacle(struct point_t *pts, int num);
		
		void draw(cv::Mat image);
		struct point_t getCentre();
		struct point_t support(struct vec2 direction);
		
	private:
		int num_vertices;
		struct point_t *vertices;
		struct point_t centre;
		
		float dotProduct(struct point_t v, struct vec2 dir);
};

#endif
