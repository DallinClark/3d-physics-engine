#include "../include/aabb.h"

AABB::AABB(glm::vec2 min, glm::vec2 max) {
	this->min = min;
	this->max = max;
}

AABB::AABB(float minX, float minY, float maxX, float maxY) {
	this->min = glm::vec2(minX, minY);
	this->max = glm::vec2(maxX, maxY);
}
