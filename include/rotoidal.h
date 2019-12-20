#ifndef ROTOIDAL_H_
#define ROTOIDAL_H_

#include "joint.h"

class Rotoidal : public Joint 
{
	public:
		Rotoidal();
		Rotoidal(float a, float alpha, float d, float teta);
		
		void draw();
};

#endif
