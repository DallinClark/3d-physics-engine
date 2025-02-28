#pragma once
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/epsilon.hpp"

#include <memory>
#include <cmath>

#include "mesh.h"
#include "collision_manifold.h"
#include "rigid_body.h"

class Collisions {
public:
	// Takes in two rigid bodies and returns if they collide and updates depth and normal to depth and direction of collision
	static bool collide(std::shared_ptr<RigidBody> bodyA, std::shared_ptr<RigidBody> bodyB, glm::vec3& normal, float& depth);

	// Functions for intersecting specific shape types
	static bool intersectCircles(glm::vec3 centerA, float radiusA, glm::vec3 centerB, float radiusB, glm::vec3& normal, float& depth);
	static bool intersectPolygons(const vector<glm::vec3>& verticesA, const vector<glm::vec3>& verticesB, const glm::vec3& centerA, const glm::vec3& centerB, glm::vec3& normal, float& depth);
	static bool intersectCirclePolygon(const glm::vec3& circleCenter,const float& circleRadius, const vector<glm::vec3>& vertices, glm::vec3 polyCenter, glm::vec3& normal, float& depth);

	// Implementation of GJK collision algorithm and EPA for depth and normal detection
	static bool EPA(std::vector<glm::vec3> simplex, const std::vector<glm::vec3>& verticesA, const std::vector<glm::vec3>& verticesB, 
		            const glm::vec3& circleCenter, const float& radius, bool isCircle, float& depth, glm::vec3& normal);
	static glm::vec3 minkowskiDifference(const vector<glm::vec3>& verticesA, const vector<glm::vec3>& verticesB, const glm::vec3& direction);             // for polygon polygon
	static glm::vec3 minkowskiDifference(const vector<glm::vec3>& vertices, const glm::vec3& circleCenter, const float radius, glm::vec3& direction);    // for circle polygon

	// Helper functions for GJK and EPA algorithms
	static bool nextSimplex(std::vector<glm::vec3>& simplex, glm::vec3& direction);
	static void addIfUniqueEdge(std::vector<std::pair<int, int>>& edges, int a, int b);
	static bool tetrahedron(std::vector<glm::vec3>& simplex, glm::vec3& direction);
	static bool triangle(std::vector<glm::vec3>& simplex, glm::vec3& direction);
	static bool line(std::vector<glm::vec3>& simplex, glm::vec3& direction);

private:
	// Constants for GJK algorithm
	static constexpr int GJK_MAX_NUM_ITERATIONS = 50;
	static constexpr int EPA_MAX_NUM_ITERATIONS = 50;
	static constexpr float EPA_TOLERANCE = 0.001f;
};
