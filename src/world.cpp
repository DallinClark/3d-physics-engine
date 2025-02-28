#include "../include/world.h"

const float World::MAX_BODY_SIZE = 640.0f * 640.0f;
const float World::MIN_BODY_SIZE = 0.01f;
const float World::MIN_DENSITY = 0.5f;    // g/cm^3
const float World::MAX_DENSITY = 21.4f;
const glm::vec3 World::GRAVITY_CONSTANT = glm::vec3(0.0f, -980.665f, 0.0f);

const int World::SPHERE_STACK_COUNT = 5;
const int World::SPHERE_SECTOR_COUNT = 5;

World::World() {
	createMeshes();
}

void World::createMeshes() {
	// Rectangle Mesh

	std::vector<Vertex> vertices = {
		// Front face
		{glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec2(0.0f, 0.0f)}, // 0
		{glm::vec3(0.5f, -0.5f,  0.5f), glm::vec2(1.0f, 0.0f)}, // 1
		{glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec2(0.0f, 1.0f)}, // 2
		{glm::vec3(0.5f,  0.5f,  0.5f), glm::vec2(1.0f, 1.0f)}, // 3

		// Back face
		{glm::vec3(0.5f, -0.5f, -0.5f), glm::vec2(0.0f, 0.0f)}, // 4
		{glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec2(1.0f, 0.0f)}, // 5
		{glm::vec3(0.5f,  0.5f, -0.5f), glm::vec2(0.0f, 1.0f)}, // 6
		{glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec2(1.0f, 1.0f)}, // 7

		// Left face
		{glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec2(0.0f, 0.0f)}, // 8
		{glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec2(1.0f, 0.0f)}, // 9
		{glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec2(0.0f, 1.0f)}, // 10
		{glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec2(1.0f, 1.0f)}, // 11

		// Right face
		{glm::vec3(0.5f, -0.5f,  0.5f), glm::vec2(0.0f, 0.0f)}, // 12
		{glm::vec3(0.5f, -0.5f, -0.5f), glm::vec2(1.0f, 0.0f)}, // 13
		{glm::vec3(0.5f,  0.5f,  0.5f), glm::vec2(0.0f, 1.0f)}, // 14
		{glm::vec3(0.5f,  0.5f, -0.5f), glm::vec2(1.0f, 1.0f)}, // 15

		// Bottom face
		{glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec2(0.0f, 0.0f)}, // 16
		{glm::vec3(0.5f, -0.5f, -0.5f), glm::vec2(1.0f, 0.0f)}, // 17
		{glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec2(0.0f, 1.0f)}, // 18
		{glm::vec3(0.5f, -0.5f,  0.5f), glm::vec2(1.0f, 1.0f)}, // 19

		// Top face
		{glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec2(0.0f, 0.0f)}, // 20
		{glm::vec3(0.5f,  0.5f,  0.5f), glm::vec2(1.0f, 0.0f)}, // 21
		{glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec2(0.0f, 1.0f)}, // 22
		{glm::vec3(0.5f,  0.5f, -0.5f), glm::vec2(1.0f, 1.0f)}  // 23
	};


	std::vector<unsigned int> indices = {
		// Front face
		0, 1, 2,  1, 2, 3,
		// Back face
		4, 5, 6,  5, 6, 7,
		// Left face
		8, 9, 10,  9, 10, 11,
		// Right face
		12, 13, 14,  13, 14, 15,
		// Bottom face
		16, 17, 18,  17, 18, 19,
		// Top face
		20, 21, 22,  21, 22, 23
	};

	std::vector<glm::vec4> verticesNoDuplicates = {
		// Front face
		{glm::vec4(-0.5f, -0.5f,  0.5f, 1.0f)}, // 0
		{glm::vec4(0.5f, -0.5f,  0.5f, 1.0f)}, // 1
		{glm::vec4(-0.5f,  0.5f,  0.5f, 1.0f)}, // 2
		{glm::vec4(0.5f,  0.5f,  0.5f, 1.0f)}, // 3

		// Back face
		{glm::vec4(0.5f, -0.5f, -0.5f, 1.0f)}, // 4
		{glm::vec4(-0.5f, -0.5f, -0.5f, 1.0f)}, // 5
		{glm::vec4(0.5f,  0.5f, -0.5f, 1.0f)}, // 6
		{glm::vec4(-0.5f,  0.5f, -0.5f, 1.0f)} // 7
	};


	std::shared_ptr<Mesh> newMesh = std::make_shared<Mesh>(vertices, indices, ShapeType::Cube);
	newMesh->setVerticesNoDuplicates(verticesNoDuplicates);
	meshes[ShapeType::Cube] = newMesh;

	vertices.clear();
	indices.clear();

	// Circle Mesh
	float radius = 1.0f;
	float x, y, z, xy;                              // vertex position
	float u, v;                                     // vertex texCoord

	float sectorStep = 2 * M_PI / SPHERE_SECTOR_COUNT;
	float stackStep = M_PI / SPHERE_STACK_COUNT;
	float sectorAngle, stackAngle;

	for (int i = 0; i <= SPHERE_STACK_COUNT; ++i) {
		stackAngle = (M_PI / 2) - (i * stackStep);        // starting from pi/2 to -pi/2
		xy = radius * cosf(stackAngle);             // r * cos(u)
		z = radius * sinf(stackAngle);              // r * sin(u)

		// add (sectorCount+1) vertices per stack
		// first and last vertices have same position, but different tex coords
		for (int j = 0; j <= SPHERE_SECTOR_COUNT; ++j)
		{
			sectorAngle = j * sectorStep;           // starting from 0 to 2pi

			// vertex position (x, y, z)
			x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
			y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)

			// vertex tex coord (u, v) range between [0, 1]
			u = (float)j / SPHERE_SECTOR_COUNT;
			v = (float)i / SPHERE_STACK_COUNT;

			vertices.push_back({ glm::vec3{x,y,z}, glm::vec2{u,v} });
			verticesNoDuplicates.push_back({ glm::vec4{x,y,z,1} });
		}
	}

	// generate index list of sphere triangles
	// k1--k1+1
	// |  / |
	// | /  |
	// k2--k2+1

	int k1, k2;
	for (int i = 0; i < SPHERE_STACK_COUNT; ++i) {
		k1 = i * (SPHERE_SECTOR_COUNT + 1);      // beginning of current stack
		k2 = k1 + SPHERE_SECTOR_COUNT + 1;         // beginning of next stack

		for (int j = 0; j < SPHERE_SECTOR_COUNT; ++j, ++k1, ++k2) {
			// First triangle of the quad (all stacks except the top pole)
			if (i != 0) {
				indices.push_back(k1);
				indices.push_back(k2);
				indices.push_back(k1 + 1);
			}
			// Second triangle of the quad (all stacks except the bottom pole)
			if (i != (SPHERE_STACK_COUNT - 1)) {
				indices.push_back(k1 + 1);
				indices.push_back(k2);
				indices.push_back(k2 + 1);
			}
		}
	}

	std::shared_ptr<Mesh> newSphereMesh = std::make_shared<Mesh>(vertices, indices, ShapeType::Sphere);
	newSphereMesh->setVerticesNoDuplicates(verticesNoDuplicates);
	meshes[ShapeType::Sphere] = newSphereMesh;
}


void World::draw(Shader& shader, glm::mat4 view, glm::mat4 projection) {
	for (int i = 0; i < bodyList.size(); ++i) {
		std::shared_ptr<RigidBody> currBody = bodyList[i];

		glm::mat4 world = currBody->getTransformMatrix();

		currBody->getMesh()->draw(shader, world, view, projection);
	}
}


void World::AddBody(std::shared_ptr<RigidBody> body) {
	bodyList.push_back(body);
}
void World::RemoveBody(int index) {
	this->bodyList.erase(bodyList.begin() + index);
	std::cout << "removed" << endl;
}


bool World::GetBody(int index, std::shared_ptr<RigidBody>& body) {
	body = nullptr;
	{}
	if (index < 0 || index >= bodyList.size()) {
		return false;
	}

	body = bodyList[index];
	return true;
}

//void World::ResolveCollisionsBasic(CollisionManifold contact) {
//
//	std::shared_ptr<RigidBody> bodyA = contact.bodyA;
//	std::shared_ptr<RigidBody> bodyB = contact.bodyB;
//	glm::vec2 normal = contact.normal;
//	float depth = contact.depth;
//
//	glm::vec2 linVelocityA = bodyA->getLinearVelocity();
//	glm::vec2 linVelocityB = bodyB->getLinearVelocity();
//
//	glm::vec2 relativeVelocity = linVelocityB - linVelocityA;
//
//	if (glm::dot(relativeVelocity, normal) > 0.0f) {
//		return;
//	}
//
//	float e = std::min(bodyA->restitution, bodyB->restitution);
//
//
//	float j = -(1.0f + e) * glm::dot(relativeVelocity, normal);
//	j /= bodyA->invMass + bodyB->invMass;
//
//	glm::vec2 impulse = j * normal;
//
//	bodyA->setLinearVelocity(linVelocityA - (impulse * bodyA->invMass));
//	bodyB->setLinearVelocity(linVelocityB + (impulse * bodyB->invMass));
//}
//
//void World::ResolveCollisionsWithRotation(CollisionManifold contact) {
//	std::shared_ptr<RigidBody> bodyA = contact.bodyA;
//	std::shared_ptr<RigidBody> bodyB = contact.bodyB;
//	glm::vec2 normal = contact.normal;
//	glm::vec2 contact1 = contact.contactOne;
//	glm::vec2 contact2 = contact.contactTwo;
//	int contactCount = contact.contactCount;
//
//	float e = std::min(bodyA->restitution, bodyB->restitution);
//
//	glm::vec2 contactList[] = { contact1, contact2 };
//	glm::vec2 impulseList[2];
//	glm::vec2 raList[2];
//	glm::vec2 rbList[2];
//
//	for (int i = 0; i < contactCount; ++i) {
//		glm::vec2 ra = contactList[i] - bodyA->getPosition();
//		glm::vec2 rb = contactList[i] - bodyB->getPosition();
//
//		raList[i] = ra;
//		rbList[i] = rb;
//
//		glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
//		glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);
//
//		glm::vec2 angularLinearVelocityA = raPerp * bodyA->getAngularVelocity();
//		glm::vec2 angularLinearVelocityB = rbPerp * bodyB->getAngularVelocity();
//
//		glm::vec2 relativeVelocity = (bodyB->getLinearVelocity() + angularLinearVelocityB)
//									- (bodyA->getLinearVelocity() + angularLinearVelocityA);
//
//		float contactVelocityMag = glm::dot(relativeVelocity, normal);
//
//		if (contactVelocityMag > 0.0f) {
//			continue;
//		}
//		
//		float raPerpDotN = glm::dot(raPerp, normal);
//		float rbPerpDotN = glm::dot(rbPerp, normal);
//
//		float denom = bodyA->invMass + bodyB->invMass + (raPerpDotN * raPerpDotN) * bodyA->invInertia +
//														(rbPerpDotN * rbPerpDotN) * bodyB->invInertia;
//
//		float j = -(1.0f + e) * contactVelocityMag;
//		j /= denom;
//		if (contactCount != 1) {
//			j /= 2.0f;
//		}
//
//		glm::vec2 impulse = j * normal;
//
//		impulseList[i] = impulse;
//	}
//
//	for (int i = 0; i < contactCount; ++i) {
//		glm::vec2 impulse = impulseList[i];
//		glm::vec2 ra = raList[i];
//		glm::vec2 rb = rbList[i];
//
//		bodyA->setLinearVelocity(bodyA->getLinearVelocity() - impulse * bodyA->invMass);
//		bodyB->setLinearVelocity(bodyB->getLinearVelocity() + impulse * bodyB->invMass);
//
//		float angularImpulseA = glm::cross(glm::vec3(ra, 0.0f), glm::vec3(impulse, 0.0f)).z;
//		float angularImpulseB = glm::cross(glm::vec3(rb, 0.0f), glm::vec3(impulse, 0.0f)).z;
//
//		bodyA->setAngularVelocity(bodyA->getAngularVelocity() - angularImpulseA * bodyA->invInertia);
//		bodyB->setAngularVelocity(bodyB->getAngularVelocity() + angularImpulseB * bodyB->invInertia);
//
//	}
//}
//
//void World::ResolveCollisionsWithRotationAndFriction(CollisionManifold contact) {
//	std::shared_ptr<RigidBody> bodyA = contact.bodyA;
//	std::shared_ptr<RigidBody> bodyB = contact.bodyB;
//	glm::vec2 normal = contact.normal;
//	glm::vec2 contact1 = contact.contactOne;
//	glm::vec2 contact2 = contact.contactTwo;
//	int contactCount = contact.contactCount;
//
//	float e = std::min(bodyA->restitution, bodyB->restitution);
//
//	float sf = (bodyA->staticFriction + bodyB->staticFriction) * 0.5f;
//	float df = (bodyA->dynamicFriction + bodyB->dynamicFriction) * 0.5f;
//
//
//	glm::vec2 contactList[] = { contact1, contact2 };
//	glm::vec2 impulseList[2];
//	glm::vec2 frictionImpulseList[2];
//	glm::vec2 raList[2];
//	glm::vec2 rbList[2];
//	float jList[2] = { 0.0f ,0.0f };
//
//	for (int i = 0; i < contactCount; ++i) {
//		glm::vec2 ra = contactList[i] - bodyA->getPosition();
//		glm::vec2 rb = contactList[i] - bodyB->getPosition();
//
//		raList[i] = ra;
//		rbList[i] = rb;
//
//		glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
//		glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);
//
//		glm::vec2 angularLinearVelocityA = raPerp * bodyA->getAngularVelocity();
//		glm::vec2 angularLinearVelocityB = rbPerp * bodyB->getAngularVelocity();
//
//		glm::vec2 relativeVelocity = (bodyB->getLinearVelocity() + angularLinearVelocityB)
//			- (bodyA->getLinearVelocity() + angularLinearVelocityA);
//
//		float contactVelocityMag = glm::dot(relativeVelocity, normal);
//
//		if (contactVelocityMag > 0.0f) {
//			continue;
//		}
//
//		float raPerpDotN = glm::dot(raPerp, normal);
//		float rbPerpDotN = glm::dot(rbPerp, normal);
//
//		float j = -(1.0f + e) * contactVelocityMag;
//		float denom = bodyA->invMass + bodyB->invMass + ((raPerpDotN * raPerpDotN) * bodyA->invInertia) +
//			((rbPerpDotN * rbPerpDotN) * bodyB->invInertia);
//
//		j /= denom;
//		if (contactCount != 1) {
//			j /= 2.0f;
//		}
//
//		glm::vec2 impulse = j * normal;
//
//		jList[i] = j;
//
//		impulseList[i] = impulse;
//	}
//
//	for (int i = 0; i < contactCount; ++i) {
//		glm::vec2 impulse = impulseList[i];
//		glm::vec2 ra = raList[i];
//		glm::vec2 rb = rbList[i];
//
//		bodyA->setLinearVelocity(bodyA->getLinearVelocity() - impulse * bodyA->invMass);
//		bodyB->setLinearVelocity(bodyB->getLinearVelocity() + impulse * bodyB->invMass);
//
//		float angularImpulseA = glm::cross(glm::vec3(ra, 0.0f), glm::vec3(impulse, 0.0f)).z;
//		float angularImpulseB = glm::cross(glm::vec3(rb, 0.0f), glm::vec3(impulse, 0.0f)).z;
//
//		bodyA->setAngularVelocity(bodyA->getAngularVelocity() - angularImpulseA * bodyA->invInertia);
//		bodyB->setAngularVelocity(bodyB->getAngularVelocity() + angularImpulseB * bodyB->invInertia);
//
//	}
//	for (int i = 0; i < contactCount; ++i) {
//		glm::vec2 ra = raList[i];
//		glm::vec2 rb = rbList[i];
//
//		glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
//		glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);
//
//		glm::vec2 angularLinearVelocityA = raPerp * bodyA->getAngularVelocity();
//		glm::vec2 angularLinearVelocityB = rbPerp * bodyB->getAngularVelocity();
//
//		glm::vec2 relativeVelocity = (bodyB->getLinearVelocity() + angularLinearVelocityB)
//			- (bodyA->getLinearVelocity() + angularLinearVelocityA);
//
//		glm::vec2 tangent = relativeVelocity - glm::dot(relativeVelocity, normal) * normal;
//
//		if (glm::length(tangent) < 0.01f) {
//			continue;
//		}
//		else {
//			tangent = glm::normalize(tangent);
//		}
//
//		float raPerpDotT = glm::dot(raPerp, tangent);
//		float rbPerpDotT = glm::dot(rbPerp, tangent);
//
//		float denom = bodyA->invMass + bodyB->invMass + (raPerpDotT * raPerpDotT) * bodyA->invInertia +
//			(rbPerpDotT * rbPerpDotT) * bodyB->invInertia;
//		if (denom < 1e-6f) {
//			denom = 1e-6f;
//		}
//
//		float jt = -glm::dot(relativeVelocity, tangent);
//		jt /= denom;
//
//		if (contactCount != 1) {
//			jt /= 2.0f;
//		}
//
//		glm::vec2 frictionImpulse;
//
//		float j = jList[i];
//
//		//frictionImpulse = jt * tangent;
//
//
//		if (std::abs(jt) <= (j * 0.6f)) {
//			frictionImpulse = jt * tangent;
//		}
//		else {
//			frictionImpulse = -j * 0.4f * tangent;
//		}
//
//		frictionImpulseList[i] = frictionImpulse;
//	}
//
//	for (int i = 0; i < contactCount; ++i) {
//		glm::vec2 impulse = frictionImpulseList[i];
//		glm::vec2 ra = raList[i];
//		glm::vec2 rb = rbList[i];
//
//		bodyA->setLinearVelocity(bodyA->getLinearVelocity() - impulse * bodyA->invMass);
//		bodyB->setLinearVelocity(bodyB->getLinearVelocity() + impulse * bodyB->invMass);
//
//		float angularImpulseA = ra.x * impulse.y - ra.y * impulse.x;
//		float angularImpulseB = rb.x * impulse.y - rb.y * impulse.x;
//
//		bodyA->setAngularVelocity(bodyA->getAngularVelocity() - angularImpulseA * bodyA->invInertia);
//		bodyB->setAngularVelocity(bodyB->getAngularVelocity() + angularImpulseB * bodyB->invInertia);
//
//	}
//}


void World::Step(float time, int iterations) {
	for (int i = 0; i < iterations; ++i) {
		contactPairs.clear();

		//StepBodies(time, iterations);
		//BroadPhase();
		NarrowPhase();
	}
}
//
//void World::BroadPhase() {
//	for (int i = 0; i < bodyList.size(); ++i) {
//		std::shared_ptr<RigidBody> bodyA = bodyList[i];
//		AABB bodyAAabb = bodyA->getAABB();
//
//		for (int j = i + 1; j < bodyList.size(); ++j) {
//			std::shared_ptr<RigidBody> bodyB = bodyList[j];
//			AABB bodyBAabb = bodyB->getAABB();
//
//			if (bodyA->isStatic && bodyB->isStatic) {
//				continue;
//			}
//
//			if (Collisions::IntersectAABBs(bodyAAabb, bodyBAabb)) {
//				continue;
//			}
//
//			contactPairs.push_back(ContactPair(i, j));
//		}
//	}
//}
void World::NarrowPhase() {
	glm::vec3 normal;
	float depth = 0.0f;
	for (int i = 0; i < bodyList.size() - 1; ++i) {
		std::shared_ptr<RigidBody> bodyA = bodyList[i];

		for (int j = i + 1; j < bodyList.size(); ++j) {
			std::shared_ptr<RigidBody> bodyB = bodyList[j];

			if (Collisions::collide(bodyA, bodyB, normal, depth)) {
				seperateBodies(bodyA, bodyB, (normal * depth));
			}
		}
	}
	/*for (int i = 0; i < contactPairs.size(); ++i) {
		glm::vec2 normal;
		float depth;
		std::shared_ptr<RigidBody> bodyA = bodyList[contactPairs[i].item1];
		std::shared_ptr<RigidBody> bodyB = bodyList[contactPairs[i].item2];


		if (Collisions::Collide(bodyA, bodyB, normal, depth)) {
			SeperateBodies(bodyA, bodyB, (normal * depth));

			glm::vec2 contactOne, contactTwo;
			int contactCount = 0;
			Collisions::FindContactPoints(bodyA, bodyB, contactOne, contactTwo, contactCount);
			CollisionManifold contact = CollisionManifold(bodyA, bodyB, depth, normal, contactOne, contactTwo, contactCount);
			this->ResolveCollisionsWithRotationAndFriction(contact);
		}
	}*/
}
//
//void World::StepBodies(float time, int iterations) {
//	for (int i = 0; i < bodyList.size(); ++i) {
//		if (!bodyList[i]->isStatic) {
//			bodyList[i]->Step(time, GRAVITY_CONSTANT, iterations);
//		}
//	}
//}
//
void World::seperateBodies(std::shared_ptr<RigidBody> bodyA, std::shared_ptr<RigidBody> bodyB, glm::vec3 moveVector) {
	if (bodyA->isStatic) {
		bodyB->Move(moveVector);
	}
	else if (bodyB->isStatic) {
		bodyA->Move(-moveVector);
	}
	else {
		bodyA->Move(-moveVector / 2.0f);
		bodyB->Move(moveVector / 2.0f);
	}
}

