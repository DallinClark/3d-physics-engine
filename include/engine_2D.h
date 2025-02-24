#pragma once
#include <vector>
#include <memory>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "rigid_body_2D.h"
#include "collisions.h"
#include "shader.h"
#include "collision_manifold.h"

#include <unordered_map>



class Engine2D {
public:

	struct ContactPair {
		int item1;
		int item2;

		ContactPair(int a, int b) : item1(a), item2(b) {}
	};

	static const float MIN_BODY_SIZE;
	static const float MAX_BODY_SIZE;

	static const float MIN_DENSITY;
	static const float MAX_DENSITY;

	Engine2D();

	void AddBody(std::shared_ptr<RigidBody2D> body);
	void RemoveBody(int index);
	bool GetBody(int index, std::shared_ptr<RigidBody2D>& body);
	int GetBodyCount() { return bodyList.size(); }

	void Step(float time, int iterations);

	void ResolveCollisionsBasic(CollisionManifold contact);
	void ResolveCollisionsWithRotation(CollisionManifold contact);
	void ResolveCollisionsWithRotationAndFriction(CollisionManifold contact);


	void Draw(Shader& shader, glm::mat4 trans);

	std::shared_ptr<Mesh> getCircleMesh() { return meshes[ShapeType::Circle]; }
	std::shared_ptr<Mesh> getSquareMesh() { return meshes[ShapeType::Square]; }

	void StepBodies(float time, int iterations);




	

private:
	std::unordered_map<ShapeType, std::shared_ptr<Mesh>> meshes;
	void createMeshes();

	void BroadPhase();
	void NarrowPhase();


	std::vector<std::shared_ptr<RigidBody2D>> bodyList;
	glm::vec2 gravity;
	std::vector<ContactPair> contactPairs;
	void SeperateBodies(std::shared_ptr<RigidBody2D> bodyA, std::shared_ptr<RigidBody2D> bodyB, glm::vec2 mtv);
};