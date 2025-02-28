#pragma once

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "mesh.h"
#include "aabb.h"

#include <string>
#include <random>
#include <memory>
#define _USE_MATH_DEFINES
#include <math.h>


class RigidBody {
private:
    glm::vec3 position;
    glm::vec3 linearVelocity;
    glm::quat rotation;
    float angularVelocity;
    bool transformUpdateRequired;
    bool aabbUpdateRequired;
    AABB aabb;

    glm::mat4 worldMatrix;

    glm::vec2 force;

    std::shared_ptr<Mesh> mesh;

    static glm::vec3 getRandomColor();

    float CalculateRotationalInertia();


public:
    RigidBody(glm::vec3 position, float density, float mass, float restitution, float area,
        bool isStatic, float radius, float width, float height, float depth, ShapeType shapeType, glm::vec3 color, std::shared_ptr<Mesh> mesh);

    const glm::vec3 color;

    const float density;
    const float mass;
    float invMass;
    const float restitution;
    const float area;
    float inertia;
    float invInertia;

    const bool isStatic;

    const float radius;
    const float width;
    const float height;
    const float depth;

    float staticFriction;
    float dynamicFriction;

    const ShapeType shapeType;

    static bool CreateCircleBody(float radius, glm::vec3 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh);
    static bool CreateSquareBody(float width, float height, float depth, glm::vec3 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh);

    void Move(glm::vec3 amount);
    void moveTo(glm::vec3 newPosition) { position = newPosition; }
    void rotate(float angleDegrees, const glm::vec3& axis);

    float getWidth() const { return width; }
    float getRadius() const { return radius; }
    glm::vec3 getLinearVelocity() const { return linearVelocity; }
    ShapeType getType() const { return shapeType;  }
    glm::vec3 getPosition() const { return position; }
    float getAngularVelocity() const { return angularVelocity; }
    std::shared_ptr<Mesh> getMesh() const { return mesh; }
    vector<glm::vec3> getTransformedVertices();
    glm::mat4 getTransformMatrix();
    


    void setLinearVelocity(glm::vec3 newVelocity) { linearVelocity = newVelocity;  }
    void setAngularVelocity(float newVelocity) { angularVelocity = newVelocity; }


    void Step(float time, glm::vec3 gravity, int iterations);

    void AddForce(glm::vec2 amount);

    AABB getAABB();
};