#include "robot.h"

void Robot::addJoint(Joint* j)
{
	joints.push_back(j);
}

Joint * Robot::getJoint(int index)
{
	return joints.at(index);
}

void Robot::computePose()
{
	
}
