#ifndef STRUCTURES_H_
#define STRUCTURES_H_

struct point_t
{
	float x;
	float y;
	
	//default + constructor
	point_t(float x=0, float y=0) : x(x), y(y) {}
	
	//assignment operator
	point_t& operator=(const point_t& a)
	{
		x = a.x;
		y = a.y;
		return *this;
	}
	
	//sum operator
	point_t operator+(const point_t& a) const
	{
		return point_t(a.x+x, a.y+y);
	}
	
	//subtract operator
	point_t operator-(const point_t& a) const
	{
		return point_t(a.x-x, a.y-y);
	}
	
	//multiply by a float number
	point_t operator*(const float value) const
	{
		return point_t(value * x, value * y);
	}
	
};

struct vec2 
{
	struct point_t start;
	struct point_t end;
	
	//default + constructor
	vec2(struct point_t start = {0,0}, struct point_t end = {0,0}) : start(start), end(end) {}
	
	//assignment operator
	vec2& operator=(const vec2& a)
	{
		start = a.start;
		end = a.end;
		return *this;
	}
	
	//multiply by a float number
	vec2 operator*(const float value) const
	{
		return vec2(start * value, end * value);
	}
};

struct node 
{
	struct point_t element;
	int num;
	struct node **neighbors;
};
#endif

