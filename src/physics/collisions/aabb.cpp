#include "../../../include/physics/collisions/aabb.h"

AABB::AABB(glm::vec3 min, glm::vec3 max) {
	this->min = min;
	this->max = max;
}

AABB::AABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
	this->min = glm::vec3(minX, minY, minZ);
	this->max = glm::vec3(maxX, maxY, maxZ);
}

bool AABB::intersectAABBs(const AABB& boxOne, const AABB& boxTwo) {
	if (boxOne.max.x > boxTwo.min.x && boxOne.max.y > boxTwo.min.y && boxOne.max.z > boxTwo.min.z
		&& boxOne.min.x < boxTwo.max.x && boxOne.min.y < boxTwo.max.y && boxOne.min.z < boxTwo.max.z) {
		return true;
	}

	return false;
}

