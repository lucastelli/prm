#ifndef STRUCTURES_H_
#define STRUCTURES_H_

struct vec2_t
{
	float x;
	float y;
	
	//default + constructor
	vec2_t(float x=0, float y=0) : x(x), y(y) {}
	
	//assignment operator
	vec2_t& operator=(const vec2_t& a)
	{
		x = a.x;
		y = a.y;
		return *this;
	}
	
	//sum operator
	vec2_t operator+(const vec2_t& a) const
	{
		return vec2_t(a.x+x, a.y+y);
	}
	
	//subtract operator
	vec2_t operator-(const vec2_t& a) const
	{
		return vec2_t(x-a.x, y-a.y);
	}
	
	//multiply by a float number
	vec2_t operator*(const float value) const
	{
		return vec2_t(value * x, value * y);
	}
};

struct vec3_t
{
	float x;
	float y;
	float z;
	
	//default + constructor
	vec3_t(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
	
	//assignment operator
	vec3_t& operator=(const vec3_t& a)
	{
		x = a.x;
		y = a.y;
		z = a.z;
		return *this;
	}
	
	//sum operator
	vec3_t operator+(const vec3_t& a) const
	{
		return vec3_t(a.x+x, a.y+y, a.z+z);
	}
	
	//subtract operator
	vec3_t operator-(const vec3_t& a) const
	{
		return vec3_t(x-a.x, y-a.y, z-a.z);
	}
	
	//multiply by a float number
	vec3_t operator*(const float value) const
	{
		return vec3_t(value * x, value * y, value * z);
	}
};

struct node 
{
	struct vec2_t element;
	int num;
	struct node **neighbors;
};
#endif

