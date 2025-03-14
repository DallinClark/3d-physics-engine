#pragma once
#include <vector>
#include <memory>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "rigid_body.h"
#include "collisions.h"
#include "shader.h"
#include "collision_manifold.h"
#include "camera.h"

#include <unordered_map>



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

	static const int SPHERE_STACK_COUNT;
	static const int SPHERE_SECTOR_COUNT;

	World();

	void AddBody(std::shared_ptr<RigidBody> body);
	void RemoveBody(int index);
	bool GetBody(int index, std::shared_ptr<RigidBody>& body);
	int GetBodyCount() { return bodyList.size(); }

	void Step(float time, int iterations);
	void StepBodies(float time, int iterations);

	void resolveCollisions(CollisionManifold contact);


	void draw(Shader& shader, glm::mat4 view, glm::mat4 projection);

	std::shared_ptr<Mesh> getSphereMesh() { return meshes[ShapeType::Sphere]; }
	std::shared_ptr<Mesh> getSquareMesh() { return meshes[ShapeType::Cube]; }

private:
	std::unordered_map<ShapeType, std::shared_ptr<Mesh>> meshes;
	void createMeshes();

	void BroadPhase();
	void NarrowPhase();
	void seperateBodies(std::shared_ptr<RigidBody> bodyA, std::shared_ptr<RigidBody> bodyB, glm::vec3 moveVector);

	std::vector<std::shared_ptr<RigidBody>> bodyList;
	std::vector<ContactPair> contactPairs;
};