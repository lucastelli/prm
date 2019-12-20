#ifndef LINEAR_H_
#define LINEAR_H_

#include "joint.h"
#include <iostream>

class Linear: public Joint
{
	public:
		Linear();
		Linear(float a, float alpha, float d, float teta);
	
		void draw();
};

#endif
