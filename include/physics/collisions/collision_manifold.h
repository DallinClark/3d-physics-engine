#pragma once
#include "../../glm/glm.hpp"
#include "../../glm/gtc/matrix_transform.hpp"
#include "../../glm/gtc/type_ptr.hpp"
#include "../rigid_body.h"

#include <memory>


class CollisionManifold {
public:
    std::shared_ptr<RigidBody> bodyA;
    std::shared_ptr<RigidBody> bodyB;
    float depth;
    glm::vec3 normal;
    std::vector<glm::vec3> contactPoints;
    int contactCount;

    CollisionManifold(
        const std::shared_ptr<RigidBody>& bodyA,
        const std::shared_ptr<RigidBody>& bodyB,
        float depth,
        const glm::vec3& normal,
        const std::vector<glm::vec3>& contactPoints,
        int contactCount
    )
        : bodyA(bodyA),
        bodyB(bodyB),
        depth(depth),
        normal(normal),
        contactPoints(contactPoints),
        contactCount(contactCount)
    {
    }
};



