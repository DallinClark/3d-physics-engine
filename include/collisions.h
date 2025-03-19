#pragma once
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/epsilon.hpp"

#include <memory>
#include <cmath>
#include <algorithm>

#include "mesh.h"
#include "collision_manifold.h"
#include "rigid_body.h"

class Collisions {
public:
	// Takes in two rigid bodies and returns if they collide and updates depth and normal to depth and direction of collision
	static bool collide(std::shared_ptr<RigidBody> bodyA, std::shared_ptr<RigidBody> bodyB, glm::vec3& normal, float& depth);

	// Functions for intersecting specific shape types, implement Seperating Axis Therom
	static bool intersectCircles(glm::vec3 centerA, float radiusA, glm::vec3 centerB, float radiusB, glm::vec3& normal, float& depth);
	static bool intersectPolygons(const vector<glm::vec3>& verticesA, const vector<glm::vec3>& verticesB, const glm::vec3& centerA, const glm::vec3& centerB, vector<Face> facesA, vector<Face> facesB, vector<int> edgesA, vector<int> edgesB, glm::vec3& normal, float& depth);
	static bool intersectCirclePolygon(const glm::vec3& circleCenter,const float& circleRadius, const vector<glm::vec3>& vertices, vector<Face> faces, glm::vec3 polyCenter, glm::vec3& normal, float& depth);

	// Helpers for SAT
	//static std::vector<glm::vec3> getFaceNormals(const std::vector<glm::vec3>& vertices, std::vector<int> faces);
	static void projectVerticesOntoAxis(const std::vector<glm::vec3>& vertices, const glm::vec3& axis, float& min, float& max);
	static void projectCircleOntoAxis(float radius, glm::vec3 center, glm::vec3 axis, float& min, float& max);

	static void findContactPoints(
		std::shared_ptr<RigidBody> bodyA,
		std::shared_ptr<RigidBody> bodyB,
		const glm::vec3& collisionNormal,  
		float penetrationDepth,
		std::vector<glm::vec3>& outContactPoints,
		int& outContactCount);

	static void findContactPointsPolygonToPolygon(
		std::shared_ptr<RigidBody> bodyA,
		std::shared_ptr<RigidBody> bodyB,
		const glm::vec3& collisionNormal,  // points from A to B
		float penetrationDepth,
		std::vector<glm::vec3>& outContactPoints,
		int& outContactCount);

	static void findContactPointsSphereToPolygon(
		std::shared_ptr<RigidBody> sphere,  
		std::shared_ptr<RigidBody> poly,      
		const glm::vec3& collisionNormal,        
		float penetrationDepth,
		std::vector<glm::vec3>& outContactPoints,
		int& outContactCount);

	static void findContactPointsSphereToSphere(
		std::shared_ptr<RigidBody> bodyA,
		std::shared_ptr<RigidBody> bodyB,
		const glm::vec3& collisionNormal,
		float penetrationDepth,
		std::vector<glm::vec3>& outContactPoints,
		int& outContactCount);

};
