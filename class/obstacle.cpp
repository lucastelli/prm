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

void Obstacle::draw(cv::Mat image, cv::Scalar color, struct vec2_t offset)
{
	cv::Point *picture;
	picture = (cv::Point *)malloc(num_vertices * sizeof(cv::Point));
	for(int i=0; i<num_vertices; i++)
	{
		picture[i].x = vertices[i].x + offset.x;
		picture[i].y = vertices[i].y + offset.y;
	}
	
	fillConvexPoly(
		image,
		picture,
		num_vertices,
		color
	);
	
	free(picture);
}

void Obstacle::drawBoundaries(cv::Mat image, cv::Scalar color, int thick, struct vec2_t offset)
{
	for(int i=0; i<num_vertices-1; i++)
	{
		line(
			image,
			cv::Point((vertices[i] + offset).x, (vertices[i] + offset).y),
			cv::Point((vertices[i+1] + offset).x, (vertices[i+1] + offset).y),
			color,
			thick
		);
	}
	
	line(
			image,
			cv::Point((vertices[num_vertices-1] + offset).x, (vertices[num_vertices-1] + offset).y),
			cv::Point((vertices[0] + offset).x, (vertices[0] + offset).y),
			color,
			thick
		);
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
