#pragma once
#include <vector>
#include <memory>
#include "../glm/glm.hpp"
#include "../glm/gtc/matrix_transform.hpp"
#include "rigid_body.h"
#include "collisions/aabb.h"
#include "collisions/collisions.h"
#include "../rendering/shader.h"
#include "collisions/collision_manifold.h"
#include "collisions/collisions.h"
#include "../rendering/camera.h"
#include "../rendering/point_light.h"
#include "../setup/mesh_creator.h"

#include <unordered_map>
#include <chrono>
#include <thread>
#include <mutex>

const float FIXED_TIMESTEP = 1.0f / 60.0f;
const int SUBSTEPS = 15;
class World {
public:

	struct ContactPair {
		int object1;
		int object2;

		ContactPair(int a, int b) : object1(a), object2(b) {}
	};

	static const float MIN_BODY_SIZE;
	static const float MAX_BODY_SIZE;

	static const float MIN_DENSITY;
	static const float MAX_DENSITY;

	static const glm::vec3 GRAVITY_CONSTANT;

	World();

	void AddBody(std::shared_ptr<RigidBody> body);
	void RemoveBody(int index);
	bool GetBody(int index, std::shared_ptr<RigidBody>& body);
	int GetBodyCount() { return bodyList.size(); }

	void Step(float time, int iterations);
	void StepBodies(float time, int iterations);

	void resolveCollisions(CollisionManifold contact);


	void draw(Shader& shader, glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos, PointLight light);

	std::shared_ptr<Mesh> getSphereMesh() { return meshes[ShapeType::Sphere]; }
	std::shared_ptr<Mesh> getSquareMesh() { return meshes[ShapeType::Cube]; }
	std::shared_ptr<Mesh> getTetrahedronMesh() { return meshes[ShapeType::Tetrahedron]; }
	std::shared_ptr<Mesh> getDiamondMesh() { return meshes[ShapeType::Diamond]; }

private:
	std::unordered_map<ShapeType, std::shared_ptr<Mesh>> meshes;
	void createMeshes();

	void BroadPhase();
	void NarrowPhase();
	void seperateBodies(std::shared_ptr<RigidBody> bodyA, std::shared_ptr<RigidBody> bodyB, glm::vec3 moveVector);

	void checkCollision(ContactPair currPair);


	std::vector<std::shared_ptr<RigidBody>> bodyList;
	std::vector<ContactPair> contactPairs;
};