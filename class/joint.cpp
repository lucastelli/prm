#include "joint.h"

Joint::Joint(float arm_lenght, float alpha_angle, float distance_lenght, float teta_angle) : 
		arm(arm_lenght), alpha(alpha_angle), distance(distance_lenght), teta(teta_angle) {}
		
float Joint::getArm()
{
	return arm;
}
		
float Joint::getAlpha()
{
	return alpha;
}
	
float Joint::getDistance()
{
	return distance;
}

float Joint::getTeta()
{
	return teta;
}

/*virtual void Joint::draw()
{

}*/
