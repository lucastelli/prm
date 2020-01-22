#ifndef STRUCTURES_H_
#define STRUCTURES_H_

/*------------------------//
		 	Vector 2D
//------------------------*/
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

/*------------------------//
		 	Vector 3D
//------------------------*/
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

/*------------------------//
		Node of a graph
//------------------------*/
struct node 
{
	struct vec2_t element;		// payload
	int num;							// num of neighbors
	struct node **neighbors; 	// array of reference
};

/*------------------------//
		k-d Tree node
//------------------------*/
enum treeAxis
{
	X, 
	Y
};

enum treeDirection
{
	LEFT, 
	RIGHT
};

struct kdTreeNode
{
	enum treeAxis axis;
	float value = 0;
	struct kdTreeNode *left = NULL;
	struct kdTreeNode *right = NULL;
	struct kdTreeNode *parent = NULL;
	struct vec2_t *point = NULL;
};

/*------------------------//
		k-d Tree node v 2.0
//------------------------*/

struct kd_node_t 
{
	float x[2];
	struct kd_node_t *left, *right;
};

#endif
