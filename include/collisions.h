#pragma once
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

#include <memory>
#include <cmath>

#include "mesh.h"
#include "rigid_body_2D.h"

class Collisions {
public:
	static bool IntersectCircles(glm::vec2 centerA, float radiusA, glm::vec2 centerB, float radiusB,
	glm::vec2& normal, float& depth);
	static bool IntersectPolygons(vector<glm::vec4> verticesA, vector<glm::vec4> verticesB, glm::vec2 polyCenterA, glm::vec2 polyCenterB, glm::vec2& normal, float& depth);
	static bool IntersectCirclePolygon(glm::vec2 circleCenter, float circleRadius, vector<glm::vec4> vertices, glm::vec2 polyCenter, glm::vec2& normal, float& depth);

	static void FindContactPoints(std::shared_ptr<RigidBody2D> bodyA, std::shared_ptr<RigidBody2D> bodyB, glm::vec2& contactOne, glm::vec2& contactTwo, int& contactCount);
	static void FindContactPoint(glm::vec2 centerA, float radiusA, glm::vec2 centerB, glm::vec2& contactPoint);
	static void FindContactPoint(glm::vec2 circleCenter, float circleRadius, glm::vec2 polygonCenter, vector<glm::vec4> polygonVertices, glm::vec2& collisionPoint);
	static void FindContactPoint(vector<glm::vec4> verticesA, vector<glm::vec4> verticesB, glm::vec2& contact1, glm::vec2& contact2, int& contactCount);

	static void PointSegmentDistance(glm::vec2 p, glm::vec2 a, glm::vec2 b, float& distanceSquared, glm::vec2& contact);


	static bool Collide(std::shared_ptr<RigidBody2D> bodyA, std::shared_ptr<RigidBody2D> bodyB, glm::vec2& normal, float& depth);

	static bool IntersectAABBs(AABB a, AABB b);

private:

	static void ProjectVertices(vector<glm::vec4> vertices, glm::vec2 axis, float& min, float& max);
	static void ProjectCircle(glm::vec2 center, float radius, glm::vec2 axis, float& min, float& max);

	static int FindClosestPointOnPolygon(glm::vec2 circleCenter, vector<glm::vec4> vertices);
};
