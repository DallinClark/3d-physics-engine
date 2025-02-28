#pragma once
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "rigid_body.h"

#include <memory>


class CollisionManifold {
public:
    const std::shared_ptr<RigidBody> bodyA;
    const std::shared_ptr<RigidBody> bodyB;
    const float depth;
    const glm::vec3 normal;
    const glm::vec3 contactOne;
    const glm::vec3 contactTwo;
    const int contactCount;

    CollisionManifold(
        const std::shared_ptr<RigidBody>& bodyA,
        const std::shared_ptr<RigidBody>& bodyB,
        const float depth,
        const glm::vec3 normal,
        const glm::vec3 contactOne,
        const glm::vec3 contactTwo,
        const int contactCount
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



