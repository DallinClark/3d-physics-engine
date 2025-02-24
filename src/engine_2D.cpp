#include "../include/engine_2D.h"

const float Engine2D::MAX_BODY_SIZE = 640.0f * 64.0f;
const float Engine2D::MIN_BODY_SIZE = 0.01f;
const float Engine2D::MIN_DENSITY = 0.5f;    // g/cm^3
const float Engine2D::MAX_DENSITY = 21.4f;

Engine2D::Engine2D() {
	gravity = glm::vec2(0.0f, -980.665f);
	createMeshes();
}

void Engine2D::createMeshes() {
	// Rectangle Mesh

	std::vector<Vertex> vertices = {
		{glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f)}, // bottom left
		{glm::vec3(0.5f, -0.5f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f)},  // bottom right
		{glm::vec3(-0.5f,  0.5f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f)}, // top left
		{glm::vec3(0.5f,  0.5f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f)}   // top right
	};

	std::vector<unsigned int> indices = {
		0, 1, 2,   // first triangle
		1, 2, 3    // second triangle
	};

	std::shared_ptr<Mesh> newMesh = std::make_shared<Mesh>(vertices, indices, ShapeType::Square);
	meshes[ShapeType::Square] = newMesh;

	// Circle Mesh
	int segments = 30;
	vertices.clear();
	indices.clear();


	vertices.push_back({ glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f) });

	for (int i = 0; i < segments; i++) {
		double angle = (2.0 * M_PI * i) / segments; 
		float x = static_cast<float>(cos(angle));
		float y = static_cast<float>(sin(angle));
		if (i == 0 || i == 1 || i == 2) {
			vertices.push_back({ glm::vec3(x, y, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f) });
		}
		else {
			vertices.push_back({ glm::vec3(x, y, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f) });
		}
	}

	for (int i = 1; i < segments; i++) {
		indices.push_back(0);    
		indices.push_back(i);     
		indices.push_back(i + 1); 
	}

	indices.push_back(0);
	indices.push_back(segments - 1);
	indices.push_back(1);        

	std::shared_ptr<Mesh> newCirMesh = std::make_shared<Mesh>(vertices, indices, ShapeType::Circle);
	meshes[ShapeType::Circle] = newCirMesh;
}


void Engine2D::Draw(Shader& shader, glm::mat4 proj) {
	for (int i = 0; i < bodyList.size(); ++i) {
		std::shared_ptr<RigidBody2D> currBody = bodyList[i];

		glm::mat4 trans = currBody->getTransformMatrix();
		trans = proj * trans;

		if (currBody->shapeType == ShapeType::Square) {
			meshes[ShapeType::Square]->Draw(shader, trans, currBody->color);
		}
		else if (currBody->shapeType == ShapeType::Circle) {
			meshes[ShapeType::Circle]->Draw(shader, trans, currBody->color);
		}
	}
}


void Engine2D::AddBody(std::shared_ptr<RigidBody2D> body) {
	bodyList.push_back(body);
}
void Engine2D::RemoveBody(int index) {
	this->bodyList.erase(bodyList.begin() + index);
	std::cout << "removed" << endl;
}


bool Engine2D::GetBody(int index, std::shared_ptr<RigidBody2D>& body) {
	body = nullptr;
	{}
	if (index < 0 || index >= bodyList.size()) {
		return false;
	}

	body = bodyList[index];
	return true;
}

void Engine2D::ResolveCollisionsBasic(CollisionManifold contact) {

	std::shared_ptr<RigidBody2D> bodyA = contact.bodyA;
	std::shared_ptr<RigidBody2D> bodyB = contact.bodyB;
	glm::vec2 normal = contact.normal;
	float depth = contact.depth;

	glm::vec2 linVelocityA = bodyA->getLinearVelocity();
	glm::vec2 linVelocityB = bodyB->getLinearVelocity();

	glm::vec2 relativeVelocity = linVelocityB - linVelocityA;

	if (glm::dot(relativeVelocity, normal) > 0.0f) {
		return;
	}

	float e = std::min(bodyA->restitution, bodyB->restitution);


	float j = -(1.0f + e) * glm::dot(relativeVelocity, normal);
	j /= bodyA->invMass + bodyB->invMass;

	glm::vec2 impulse = j * normal;

	bodyA->setLinearVelocity(linVelocityA - (impulse * bodyA->invMass));
	bodyB->setLinearVelocity(linVelocityB + (impulse * bodyB->invMass));
}

void Engine2D::ResolveCollisionsWithRotation(CollisionManifold contact) {
	std::shared_ptr<RigidBody2D> bodyA = contact.bodyA;
	std::shared_ptr<RigidBody2D> bodyB = contact.bodyB;
	glm::vec2 normal = contact.normal;
	glm::vec2 contact1 = contact.contactOne;
	glm::vec2 contact2 = contact.contactTwo;
	int contactCount = contact.contactCount;

	float e = std::min(bodyA->restitution, bodyB->restitution);

	glm::vec2 contactList[] = { contact1, contact2 };
	glm::vec2 impulseList[2];
	glm::vec2 raList[2];
	glm::vec2 rbList[2];

	for (int i = 0; i < contactCount; ++i) {
		glm::vec2 ra = contactList[i] - bodyA->getPosition();
		glm::vec2 rb = contactList[i] - bodyB->getPosition();

		raList[i] = ra;
		rbList[i] = rb;

		glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
		glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);

		glm::vec2 angularLinearVelocityA = raPerp * bodyA->getAngularVelocity();
		glm::vec2 angularLinearVelocityB = rbPerp * bodyB->getAngularVelocity();

		glm::vec2 relativeVelocity = (bodyB->getLinearVelocity() + angularLinearVelocityB)
									- (bodyA->getLinearVelocity() + angularLinearVelocityA);

		float contactVelocityMag = glm::dot(relativeVelocity, normal);

		if (contactVelocityMag > 0.0f) {
			continue;
		}
		
		float raPerpDotN = glm::dot(raPerp, normal);
		float rbPerpDotN = glm::dot(rbPerp, normal);

		float denom = bodyA->invMass + bodyB->invMass + (raPerpDotN * raPerpDotN) * bodyA->invInertia +
														(rbPerpDotN * rbPerpDotN) * bodyB->invInertia;

		float j = -(1.0f + e) * contactVelocityMag;
		j /= denom;
		if (contactCount != 1) {
			j /= 2.0f;
		}

		glm::vec2 impulse = j * normal;

		impulseList[i] = impulse;
	}

	for (int i = 0; i < contactCount; ++i) {
		glm::vec2 impulse = impulseList[i];
		glm::vec2 ra = raList[i];
		glm::vec2 rb = rbList[i];

		bodyA->setLinearVelocity(bodyA->getLinearVelocity() - impulse * bodyA->invMass);
		bodyB->setLinearVelocity(bodyB->getLinearVelocity() + impulse * bodyB->invMass);

		float angularImpulseA = glm::cross(glm::vec3(ra, 0.0f), glm::vec3(impulse, 0.0f)).z;
		float angularImpulseB = glm::cross(glm::vec3(rb, 0.0f), glm::vec3(impulse, 0.0f)).z;

		bodyA->setAngularVelocity(bodyA->getAngularVelocity() - angularImpulseA * bodyA->invInertia);
		bodyB->setAngularVelocity(bodyB->getAngularVelocity() + angularImpulseB * bodyB->invInertia);

	}
}

void Engine2D::ResolveCollisionsWithRotationAndFriction(CollisionManifold contact) {
	std::shared_ptr<RigidBody2D> bodyA = contact.bodyA;
	std::shared_ptr<RigidBody2D> bodyB = contact.bodyB;
	glm::vec2 normal = contact.normal;
	glm::vec2 contact1 = contact.contactOne;
	glm::vec2 contact2 = contact.contactTwo;
	int contactCount = contact.contactCount;

	float e = std::min(bodyA->restitution, bodyB->restitution);

	float sf = (bodyA->staticFriction + bodyB->staticFriction) * 0.5f;
	float df = (bodyA->dynamicFriction + bodyB->dynamicFriction) * 0.5f;


	glm::vec2 contactList[] = { contact1, contact2 };
	glm::vec2 impulseList[2];
	glm::vec2 frictionImpulseList[2];
	glm::vec2 raList[2];
	glm::vec2 rbList[2];
	float jList[2] = { 0.0f ,0.0f };

	for (int i = 0; i < contactCount; ++i) {
		glm::vec2 ra = contactList[i] - bodyA->getPosition();
		glm::vec2 rb = contactList[i] - bodyB->getPosition();

		raList[i] = ra;
		rbList[i] = rb;

		glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
		glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);

		glm::vec2 angularLinearVelocityA = raPerp * bodyA->getAngularVelocity();
		glm::vec2 angularLinearVelocityB = rbPerp * bodyB->getAngularVelocity();

		glm::vec2 relativeVelocity = (bodyB->getLinearVelocity() + angularLinearVelocityB)
			- (bodyA->getLinearVelocity() + angularLinearVelocityA);

		float contactVelocityMag = glm::dot(relativeVelocity, normal);

		if (contactVelocityMag > 0.0f) {
			continue;
		}

		float raPerpDotN = glm::dot(raPerp, normal);
		float rbPerpDotN = glm::dot(rbPerp, normal);

		float j = -(1.0f + e) * contactVelocityMag;
		float denom = bodyA->invMass + bodyB->invMass + ((raPerpDotN * raPerpDotN) * bodyA->invInertia) +
			((rbPerpDotN * rbPerpDotN) * bodyB->invInertia);

		j /= denom;
		if (contactCount != 1) {
			j /= 2.0f;
		}

		glm::vec2 impulse = j * normal;

		jList[i] = j;

		impulseList[i] = impulse;
	}

	for (int i = 0; i < contactCount; ++i) {
		glm::vec2 impulse = impulseList[i];
		glm::vec2 ra = raList[i];
		glm::vec2 rb = rbList[i];

		bodyA->setLinearVelocity(bodyA->getLinearVelocity() - impulse * bodyA->invMass);
		bodyB->setLinearVelocity(bodyB->getLinearVelocity() + impulse * bodyB->invMass);

		float angularImpulseA = glm::cross(glm::vec3(ra, 0.0f), glm::vec3(impulse, 0.0f)).z;
		float angularImpulseB = glm::cross(glm::vec3(rb, 0.0f), glm::vec3(impulse, 0.0f)).z;

		bodyA->setAngularVelocity(bodyA->getAngularVelocity() - angularImpulseA * bodyA->invInertia);
		bodyB->setAngularVelocity(bodyB->getAngularVelocity() + angularImpulseB * bodyB->invInertia);

	}
	for (int i = 0; i < contactCount; ++i) {
		glm::vec2 ra = raList[i];
		glm::vec2 rb = rbList[i];

		glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
		glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);

		glm::vec2 angularLinearVelocityA = raPerp * bodyA->getAngularVelocity();
		glm::vec2 angularLinearVelocityB = rbPerp * bodyB->getAngularVelocity();

		glm::vec2 relativeVelocity = (bodyB->getLinearVelocity() + angularLinearVelocityB)
			- (bodyA->getLinearVelocity() + angularLinearVelocityA);

		glm::vec2 tangent = relativeVelocity - glm::dot(relativeVelocity, normal) * normal;

		if (glm::length(tangent) < 0.01f) {
			continue;
		}
		else {
			tangent = glm::normalize(tangent);
		}

		float raPerpDotT = glm::dot(raPerp, tangent);
		float rbPerpDotT = glm::dot(rbPerp, tangent);

		float denom = bodyA->invMass + bodyB->invMass + (raPerpDotT * raPerpDotT) * bodyA->invInertia +
			(rbPerpDotT * rbPerpDotT) * bodyB->invInertia;
		if (denom < 1e-6f) {
			denom = 1e-6f;
		}

		float jt = -glm::dot(relativeVelocity, tangent);
		jt /= denom;

		if (contactCount != 1) {
			jt /= 2.0f;
		}

		glm::vec2 frictionImpulse;

		float j = jList[i];

		//frictionImpulse = jt * tangent;


		if (std::abs(jt) <= (j * 0.6f)) {
			frictionImpulse = jt * tangent;
		}
		else {
			frictionImpulse = -j * 0.4f * tangent;
		}

		frictionImpulseList[i] = frictionImpulse;
	}

	for (int i = 0; i < contactCount; ++i) {
		glm::vec2 impulse = frictionImpulseList[i];
		glm::vec2 ra = raList[i];
		glm::vec2 rb = rbList[i];

		bodyA->setLinearVelocity(bodyA->getLinearVelocity() - impulse * bodyA->invMass);
		bodyB->setLinearVelocity(bodyB->getLinearVelocity() + impulse * bodyB->invMass);

		float angularImpulseA = ra.x * impulse.y - ra.y * impulse.x;
		float angularImpulseB = rb.x * impulse.y - rb.y * impulse.x;

		bodyA->setAngularVelocity(bodyA->getAngularVelocity() - angularImpulseA * bodyA->invInertia);
		bodyB->setAngularVelocity(bodyB->getAngularVelocity() + angularImpulseB * bodyB->invInertia);

	}
}


void Engine2D::Step(float time, int iterations) {
	for (int i = 0; i < iterations; ++i) {
		contactPairs.clear();

		StepBodies(time, iterations);
		BroadPhase();
		NarrowPhase();
	}
}

void Engine2D::BroadPhase() {
	for (int i = 0; i < bodyList.size(); ++i) {
		std::shared_ptr<RigidBody2D> bodyA = bodyList[i];
		AABB bodyAAabb = bodyA->getAABB();

		for (int j = i + 1; j < bodyList.size(); ++j) {
			std::shared_ptr<RigidBody2D> bodyB = bodyList[j];
			AABB bodyBAabb = bodyB->getAABB();

			if (bodyA->isStatic && bodyB->isStatic) {
				continue;
			}

			if (Collisions::IntersectAABBs(bodyAAabb, bodyBAabb)) {
				continue;
			}

			contactPairs.push_back(ContactPair(i, j));
		}
	}
}
void Engine2D::NarrowPhase() {
	for (int i = 0; i < contactPairs.size(); ++i) {
		glm::vec2 normal;
		float depth;
		std::shared_ptr<RigidBody2D> bodyA = bodyList[contactPairs[i].item1];
		std::shared_ptr<RigidBody2D> bodyB = bodyList[contactPairs[i].item2];


		if (Collisions::Collide(bodyA, bodyB, normal, depth)) {
			SeperateBodies(bodyA, bodyB, (normal * depth));

			glm::vec2 contactOne, contactTwo;
			int contactCount = 0;
			Collisions::FindContactPoints(bodyA, bodyB, contactOne, contactTwo, contactCount);
			CollisionManifold contact = CollisionManifold(bodyA, bodyB, depth, normal, contactOne, contactTwo, contactCount);
			this->ResolveCollisionsWithRotationAndFriction(contact);
		}

	}
}

void Engine2D::StepBodies(float time, int iterations) {
	for (int i = 0; i < bodyList.size(); ++i) {
		if (!bodyList[i]->isStatic) {
			bodyList[i]->Step(time, gravity, iterations);
		}
	}
}

void Engine2D::SeperateBodies(std::shared_ptr<RigidBody2D> bodyA, std::shared_ptr<RigidBody2D> bodyB, glm::vec2 mtv) {
	if (bodyA->isStatic) {
		bodyB->Move(mtv);
	}
	else if (bodyB->isStatic) {
		bodyA->Move(-mtv);
	}
	else {
		bodyA->Move(-mtv / 2.0f);
		bodyB->Move(mtv / 2.0f);
	}
}

