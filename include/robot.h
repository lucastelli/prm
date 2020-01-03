#ifndef ROBOT_H_
#define ROBOT_H_

#include "joint.h"

class Robot
{
	private:
		std::vector<Joint*> joints;
		
	public:
		void addJoint(Joint* j);
		/*void computePose();
		void draw();*/
};

#endif
