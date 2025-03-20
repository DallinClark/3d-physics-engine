#pragma once

#include "glm/glm.hpp"

struct PointLight {
    glm::vec3 Position;
    glm::vec3 Color;
	PointLight(glm::vec3 position, glm::vec3 color) : Position(position), Color(color) {}
};