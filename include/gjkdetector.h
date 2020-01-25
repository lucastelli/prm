#ifndef GJKDETECTOR_H_
#define GJKDETECTOR_H_

#include <iostream>

#include "vector.h"
#include "obstacle.h"
#include "mobilerobot.h"

enum gjkState{
  		UPDATE_SIMPLEX,
  		NO_COLLISION,
  		COLLISION_FOUND
  	};

class GJKDetector
{
	private:
		std::vector<struct vec2_t> simplex_vertices;
		struct vec2_t direction;
	  	enum gjkState result;
	  	
	  	enum gjkState updateSimplex(MobileRobot *r, Obstacle *ob);
	  	enum gjkState addSupport(MobileRobot *r, Obstacle *ob);
	  	struct vec2_t getSupport(struct vec2_t direction, MobileRobot *mobile, Obstacle *obj);
	  	
	public:
		GJKDetector();
		enum gjkState checkAllCollision(struct vec2_t *config, MobileRobot *mobile, Obstacle **obx, int num_obs);
		enum gjkState checkCollision(struct vec2_t *config, MobileRobot *mobile, Obstacle *obx);
};

#endif
