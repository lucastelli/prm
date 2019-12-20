#include <cmath>
#include <iostream>
#include "joint.h"
#include "rotoidal.h"
#include "linear.h"
#include "prmConfig.h"

int main(int argc, char** argv)
{
    // report version
    std::cout << argv[0] << " Version " << prm_VERSION_MAJOR << "."
              << prm_VERSION_MINOR << std::endl;
	  
	Joint j1(10,0,0,M_PI/6);
	std::cout << j1.getArm() << "\n";
	std::cout << j1.getAlpha() << "\n";
	std::cout << j1.getDistance() << "\n";
	std::cout << j1.getTeta() << "\n";
	
	Rotoidal j2_r(50, 40, 23, 54);
	Linear j3_l(100, 3, 12, 77);
	
	std::cout << j2_r.getArm() << "\n";
	return 0;
}

