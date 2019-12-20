#ifndef JOINT_H_
#define JOINT_H_

class Rotoidal;
class Linear;

class Joint
{
	protected:
	float arm, alpha, distance, teta;
	
	public:
		Joint(float arm_lenght, float alpha_angle, float distance_lenght, float teta_angle);	
		float getArm();	
		float getAlpha();
		float getDistance();	
		float getTeta();
		void draw();
};

#endif
