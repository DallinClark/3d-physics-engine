#pragma once
#include "../../glm/glm.hpp"
#include "../../glm/gtc/matrix_transform.hpp"

/* Class for axis-alligned boudning boxes used for broad phase collision detection */
class AABB {
public:
	glm::vec3 min;
	glm::vec3 max;

	AABB() {};
	AABB(glm::vec3 min, glm::vec3 max);
	AABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);

	// returns true if they intersect and false if not
	static bool intersectAABBs(const AABB& boxOne, const AABB& boxTwo);
};