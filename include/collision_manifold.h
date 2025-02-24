#pragma once
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "rigid_body_2D.h"

#include <memory>


class CollisionManifold {
public:
    const std::shared_ptr<RigidBody2D> bodyA;
    const std::shared_ptr<RigidBody2D> bodyB;
    const float depth;
    const glm::vec2 normal;
    const glm::vec2 contactOne;
    const glm::vec2 contactTwo;
    const int contactCount;

    CollisionManifold(
        const std::shared_ptr<RigidBody2D>& bodyA,
        const std::shared_ptr<RigidBody2D>& bodyB,
        float depth,
        const glm::vec2 normal,
        const glm::vec2 contactOne,
        const glm::vec2 contactTwo,
        int contactCount
    )
        : bodyA(bodyA), 
        bodyB(bodyB),
        depth(depth),
        normal(normal),
        contactOne(contactOne),
        contactTwo(contactTwo),
        contactCount(contactCount)
    {
    }
};


