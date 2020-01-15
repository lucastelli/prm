#include "obstacle.h"

Obstacle::Obstacle(struct vec2_t *pts, int num)
{
	vertices = (struct vec2_t *)malloc(num * sizeof(struct vec2_t));
	for(int i = 0; i < num; i++)
	{
		vertices[i] = pts[i];
	}
	num_vertices = num;
	
	// compute centre of obstacle
	float sum_x = 0, sum_y = 0;
	for(int i = 0; i < num; i++)
	{
		sum_x = sum_x + vertices[i].x;
		sum_y = sum_y + vertices[i].y;
	}
	centre.x = sum_x/num;
	centre.y = sum_y/num;
}

void Obstacle::draw(cv::Mat image)
{
	cv::Scalar color = cv::Scalar(0,0,0);
	cv::Point *picture;
	picture = (cv::Point *)malloc(num_vertices * sizeof(cv::Point));
	for(int i=0; i<num_vertices; i++)
	{
		picture[i].x = vertices[i].x;
		picture[i].y = vertices[i].y;
	}
	
	fillConvexPoly(
		image,
		picture,
		num_vertices,
		color
	);
	
	free(picture);
}

struct vec2_t Obstacle::getCentre()
{
	return centre;
}

struct vec2_t Obstacle::support(struct vec2_t direction)
{
	float furthestDistance = - INFINITY;
	struct vec2_t *furthestVertex = NULL;
	
	for(int i=0; i < num_vertices; i++)
	{
		float distance = dotProduct(vertices[i], direction);
		if(distance > furthestDistance)
		{
			furthestDistance = distance;
			furthestVertex = &vertices[i];
		}
	}
	
	return *furthestVertex;
}

float Obstacle::dotProduct(struct vec2_t v, struct vec2_t dir)
{
	return (v.x * dir.x + v.y * dir.y);
}
