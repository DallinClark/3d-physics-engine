#pragma once
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

class AABB {
public:
	glm::vec2 min;
	glm::vec2 max;

	AABB() {};
	AABB(glm::vec2 min, glm::vec2 max);
	AABB(float minX, float minY, float maxX, float maxY);
};