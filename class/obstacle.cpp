#include "obstacle.h"

Obstacle::Obstacle(struct point_t *pts, int num)
{
	vertices = (struct point_t *)malloc(num * sizeof(struct point_t));
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

struct point_t Obstacle::getCentre()
{
	return centre;
}

struct point_t Obstacle::support(struct vec2 direction)
{
	float furthestDistance = - INFINITY;
	struct point_t *furthestVertex = NULL;
	
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

float Obstacle::dotProduct(struct point_t v, struct vec2 dir)
{
	struct point_t tmp = {dir.end.x - dir.start.x, dir.end.y - dir.start.y};
	return (v.x * tmp.x + v.y * tmp.y);
}
