#include "linear.h"

Linear::Linear(float a, float alpha, float d, float teta) : Joint(a, alpha, d, teta){}

void Linear::draw()
{
	std::cout << "drawing linear joint \n";
}
