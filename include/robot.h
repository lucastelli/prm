#ifndef ROBOT_H_
#define ROBOT_H_

#include "joint.h"

class Robot
{
	private:
		std::vector<Joint*> joints;
		
	public:
		void addJoint(Joint* j);
		Joint * getJoint(int index);
		void computePose();
		/*void draw();*/
};

#endif
