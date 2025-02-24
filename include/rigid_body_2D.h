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


class RigidBody2D {
private:
    glm::vec2 position;
    glm::vec2 linearVelocity;
    float angle;
    float angularVelocity;
    bool transformUpdateRequired;
    bool aabbUpdateRequired;
    AABB aabb;

    glm::mat4 transformMatrix;

    glm::vec2 force;

    std::shared_ptr<Mesh> mesh;

    static glm::vec3 getRandomColor();

    float CalculateRotationalInertia();


public:
    RigidBody2D(glm::vec2 position, float density, float mass, float restitution, float area,
        bool isStatic, float radius, float width, float height, ShapeType shapeType, glm::vec3 color, std::shared_ptr<Mesh> mesh);

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

    float staticFriction;
    float dynamicFriction;

    const ShapeType shapeType;

    static bool CreateCircleBody(float radius, glm::vec2 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody2D>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh);
    static bool CreateSquareBody(float width, float height, glm::vec2 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody2D>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh);

    void Move(glm::vec2 amount);
    void MoveTo(glm::vec2 newPosition) { position = newPosition; }
    void Rotate(float amount);

    float getWidth() const { return width; }
    float getRadius() const { return radius; }
    glm::vec2 getLinearVelocity() const { return linearVelocity; }
    ShapeType getType() const { return shapeType;  }
    float getAngle() const { return angle; }
    glm::vec2 getPosition() const { return position; }
    float getAngularVelocity() const { return angularVelocity; }
    vector<glm::vec4> getTransformedVertices();
    glm::mat4 getTransformMatrix();
    


    void setLinearVelocity(glm::vec2 newVelocity) { linearVelocity = newVelocity;  }
    void setAngularVelocity(float newVelocity) { angularVelocity = newVelocity; }


    void Step(float time, glm::vec2 gravity, int iterations);

    void AddForce(glm::vec2 amount);

    AABB getAABB();
};