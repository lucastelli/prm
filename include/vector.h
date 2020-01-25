#ifndef VECTOR_H_
#define	VECTOR_H_

#include <vector>
#include <cmath>
#include "structures.h"

struct vec3_t vectorProduct(struct vec3_t a, struct vec3_t b);

struct vec2_t normalize(struct vec2_t vett);

struct vec3_t normalize(struct vec3_t vett);

float dotProduct(struct vec2_t v, struct vec2_t dir);

#endif
