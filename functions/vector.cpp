#include "vector.h"

#include <iostream>

struct vec3_t vectorProduct(struct vec3_t *a, struct vec3_t *b)
{
	struct vec3_t result;
	result.x = a->y * b->z - a->z * b->y;
	result.y = - a->x * b->z + a->z * b->x;
	result.z = a->x * b->y - a->y * b->x;
	return result;
}

struct vec2_t normalize(struct vec2_t vett)
{
	struct vec2_t result;
	float mod = sqrt(pow(vett.x, 2) + pow(vett.y, 2));
	result.x = vett.x/mod;
	result.y = vett.y/mod;
	return result;
}

struct vec3_t normalize(struct vec3_t vett)
{
	float mod = sqrt(pow(vett.x, 2) + pow(vett.y, 2) + pow(vett.z, 2));
	return vec3_t(vett.x/mod, vett.y/mod, vett.z/mod);
}

float dotProduct(struct vec2_t v, struct vec2_t dir)
{
	return (v.x * dir.x + v.y * dir.y);
}
