#include "gjkdetector.h"
#include "vector.cpp"

GJKDetector::GJKDetector()
{
	direction = {0, 0};
	result = UPDATE_SIMPLEX;
	std::vector<struct vec2_t> simplex_vertices;
}

enum gjkState GJKDetector::checkAllCollision(struct vec2_t *config, MobileRobot *mobile, Obstacle **obx, int num_obs)
{
	enum gjkState result;
	for(int i=0; i < num_obs; i++)
	{
		result = checkCollision(config, mobile, obx[i]);
		if(result == COLLISION_FOUND)
		{
			return COLLISION_FOUND;
		} 
	}
	return result;
}

enum gjkState GJKDetector::checkCollision(struct vec2_t *config, MobileRobot *mobile, Obstacle *obx)
{
	simplex_vertices.clear();
  	mobile->setPosition(*config);
  	result = UPDATE_SIMPLEX;
  	while(result == UPDATE_SIMPLEX)
  	{
  		result = updateSimplex(mobile, obx);
	}	
	return result;
}

enum gjkState GJKDetector::updateSimplex(MobileRobot *r, Obstacle *ob)
{
	struct vec2_t a, v2_o, v2_v1, v2_v0, v2_v1_orth_2d, v2_v0_orth_2d;
	struct vec3_t tmp, v2_v1_orth, v2_v0_orth, a_3d, v0_0_3d, v2_v0_3d, v2_v1_3d;
	double dot1, dot2;
	
	
	//std::cout << "simplex_vertices.size() = " << simplex_vertices.size() << std::endl;//
	switch(simplex_vertices.size())
	{
		case 0:	direction = r->getPosition() - ob->getCentre();
					//std::cout << "case 0" << std::endl;
					//std::cout << "direction = ["<< direction.x << ", " << direction.y << "]" << std::endl;
					break;
					
		case 1:	direction = direction*(-1);
					//std::cout << "case 1" << std::endl;
					//std::cout << "direction = ["<< direction.x << ", " << direction.y << "]" << std::endl;
					break;
					
		case 2:	a = simplex_vertices.at(0) - simplex_vertices.at(1);
					a_3d = vec3_t(a.x, a.y, 0);
					v0_0_3d = vec3_t(-simplex_vertices.at(0).x, -simplex_vertices.at(0).y, 0);
					//std::cout << "a = ["<< a.x << ", " << a.y << "]" << std::endl;
					//std::cout << "a_3d = ["<< a_3d.x << ", " << a_3d.y << ", " << a_3d.z << "]" << std::endl;
					//std::cout << "v0_0_3d = ["<< v0_0_3d.x << ", " << v0_0_3d.y << ", " << v0_0_3d.z << "]" << std::endl;
  					tmp = vectorProduct(&a_3d, &v0_0_3d);
  					//std::cout << "&tmp = ["<< &tmp << "]" << std::endl;
  					//std::cout << "tmp = ["<< tmp.x << ", " << tmp.y << ", " << tmp.z << "]" << std::endl;
  					tmp = vectorProduct(&tmp, &a_3d);
  					//tmp = normalize(tmp);
  					
  					direction.x = tmp.x;
  					direction.y = tmp.y;
  					//std::cout << "case 2" << std::endl;
					//std::cout << "direction = ["<< direction.x << ", " << direction.y << "]" << std::endl;
  					break;
  		
  		case 3:	v2_o = simplex_vertices.at(2)*(-1);								//v2 to the origin
					v2_v1 = simplex_vertices.at(1) - simplex_vertices.at(2);	//v2 to v1
					v2_v0 = simplex_vertices.at(0) - simplex_vertices.at(2); //v2 to v0
					
					v2_v0_3d = vec3_t(v2_v0.x, v2_v0.y, 0);
					v2_v1_3d = vec3_t(v2_v1.x, v2_v1.y, 0);
					
					v2_v1_orth = vectorProduct(&v2_v0_3d, &v2_v1_3d);
					v2_v1_orth = vectorProduct(&v2_v1_orth, &v2_v1_3d);
					
					v2_v0_orth = vectorProduct(&v2_v1_3d, &v2_v0_3d);
					v2_v0_orth = vectorProduct(&v2_v0_orth, &v2_v0_3d);
					
					v2_v1_orth_2d = vec2_t(v2_v1_orth.x, v2_v1_orth.y);
					v2_v0_orth_2d = vec2_t(v2_v0_orth.x, v2_v0_orth.y);
					dot1 = dotProduct(v2_o, v2_v1_orth_2d);
					dot2 = dotProduct(v2_o, v2_v0_orth_2d);
					if(dot1 > 0)
					{
						// remove v0 -> reallocation (v1 become v0 & v2 become v1)
						simplex_vertices.erase(simplex_vertices.begin());
						direction = v2_v1_orth_2d;
					}
					else if(dot2 > 0)
					{
						// remove v1 -> reallocation (v2 become v1 & v0 remain v0)
						simplex_vertices.erase(simplex_vertices.begin()+1);
						direction = v2_v0_orth_2d;
					}
					else
					{
						return COLLISION_FOUND;
					}
					//std::cout << "case 3" << std::endl;
					//std::cout << "dot1 = " << dot1 << std::endl;
					//std::cout << "dot2 = " << dot1 << std::endl;
					//std::cout << "direction = ["<< direction.x << ", " << direction.y << "]" << std::endl;
					break;
		
		default:	std::cout << "Error: case dafault" << std::endl;
					break;
	}
	
	return addSupport(r, ob);
}

enum gjkState GJKDetector::addSupport(MobileRobot *r, Obstacle *ob)
{
	enum gjkState result = UPDATE_SIMPLEX;
	struct vec2_t newVertex;
	double dot;
	newVertex = getSupport(direction, r, ob);
	simplex_vertices.push_back(newVertex);
	dot = dotProduct(direction, newVertex);
	if(dot < 0)
	{
		result = NO_COLLISION;
	}
	/*std::cout << "Add Support" << std::endl;
	std::cout << "newVertex = ["<< newVertex.x << ", " << newVertex.y << "]" << std::endl;
	std::cout << "simplex_vertices = { " << std::endl;
	for(int i=0; i < simplex_vertices.size(); i++)
	{
		std::cout << "\t v"<< i << " = [" << simplex_vertices.at(i).x << ", " << simplex_vertices.at(i).y << "]" << std::endl;
	}
	std::cout << "}" << std::endl;
	std::cout << "simplex_vertices.size() = " << simplex_vertices.size() << std::endl;
	std::cout << "dotProduct(direction, newVertex) = " << dot << std::endl;*/
	return result;
}

struct vec2_t GJKDetector::getSupport(struct vec2_t direction, MobileRobot *mobile, Obstacle *obj)
{
	return mobile->support(direction) - obj->support(direction*(-1));;
}	
  	
