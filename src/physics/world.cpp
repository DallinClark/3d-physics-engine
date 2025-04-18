#include "../../include/physics/world.h"
#include <memory>

const float World::MAX_BODY_SIZE = 640.0f * 640.0f;
const float World::MIN_BODY_SIZE = 0.01f;
const float World::MIN_DENSITY = 0.5f;
const float World::MAX_DENSITY = 21.4f;
const glm::vec3 World::GRAVITY_CONSTANT = glm::vec3(0.0f, -9.8, 0.0f);

World::World() {
	createMeshes();
}

void World::createMeshes() {
	std::shared_ptr<Mesh> cubeMesh, sphereMesh, tetraMesh, diamondMesh;
	MeshCreator::createMeshes(cubeMesh, sphereMesh, tetraMesh, diamondMesh);

	meshes[ShapeType::Cube] = cubeMesh;
	meshes[ShapeType::Sphere] = sphereMesh;
	meshes[ShapeType::Tetrahedron] = tetraMesh;
	meshes[ShapeType::Diamond] = diamondMesh;
}

void World::draw(Shader& shader, glm::mat4 view, glm::mat4 projection, glm::vec3 cameraPos, PointLight light) {
	for (int i = 0; i < bodyList.size(); ++i) {
		std::shared_ptr<RigidBody> currBody = bodyList[i];

		glm::mat4 world = currBody->getTransformMatrix();

		Texture& currTexture = currBody->texture;
		currTexture.use();
		shader.setInt("texture1", 0);

		currBody->getMesh()->draw(shader, world, view, projection, cameraPos, light);
	}
}

void World::AddBody(std::shared_ptr<RigidBody> body) {
	bodyList.push_back(body);
}

void World::RemoveBody(int index) {
	this->bodyList.erase(bodyList.begin() + index);
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

void World::resolveCollisions(CollisionManifold contact) {
	std::shared_ptr<RigidBody> bodyA = contact.bodyA;
	std::shared_ptr<RigidBody> bodyB = contact.bodyB;
	glm::vec3 normal = contact.normal;  // 3D collision normal
	std::vector<glm::vec3> contactList = contact.contactPoints;
	int contactCount = contact.contactCount;

	// Use the minimum restitution between the bodies.
	float e = std::min(bodyA->restitution, bodyB->restitution);

	// These vectors will store the computed impulses and the offsets (ra and rb) from each body's center of mass.
	std::vector<glm::vec3> impulseList;
	std::vector<glm::vec3> frictionImpulseList;
	std::vector<glm::vec3> raList;
	std::vector<glm::vec3> rbList;
	std::vector<float> jList;

	// Process each contact point
	for (int i = 0; i < contactCount; ++i) {
		// Compute the offset from each body's center of mass to the contact point.
		glm::vec3 ra = contactList[i] - bodyA->getPosition();
		glm::vec3 rb = contactList[i] - bodyB->getPosition();
		raList.push_back(ra);
		rbList.push_back(rb);

		// Compute the velocity at the contact point due to both linear and angular motion.
		glm::vec3 angularVelocityA = glm::cross(bodyA->getAngularVelocity(), ra); // Angular velocity at point A
		glm::vec3 angularVelocityB = glm::cross(bodyB->getAngularVelocity(), rb); // Angular velocity at point B

		// Relative velocity at the points of contact
		glm::vec3 relativeVelocity = (bodyB->getLinearVelocity() + angularVelocityB) -
                             (bodyA->getLinearVelocity() + angularVelocityA);


		// Compute the magnitude of the contact velocity along the collision normal.
		float contactVelocityMag = glm::dot(relativeVelocity, normal);

		// If the bodies are separating, no corrective impulse is needed.
		// Instead of skipping (which would misalign our contact lists), we assign zero impulse.
		float j = 0.0f;
		if (contactVelocityMag <= 0.0f) {
			// Compute the contribution from rotational inertia.
			glm::vec3 raCrossN = glm::cross(ra, normal);
			glm::vec3 rbCrossN = glm::cross(rb, normal);
			glm::vec3 raInertia = bodyA->invIntertiaTensor * raCrossN;
			if (bodyA->isStatic) {
				raInertia = glm::vec3(0.0f);
			}
			glm::vec3 rbInertia = bodyB->invIntertiaTensor * rbCrossN;
			if (bodyB->isStatic) {
				rbInertia = glm::vec3(0.0f);
			}

			// Denominator combines the inverse masses and the rotational inertia terms.
			float denom = bodyA->invMass + bodyB->invMass +
				glm::dot(glm::cross(raInertia, ra), normal) +
				glm::dot(glm::cross(rbInertia, rb), normal);

			// Compute the impulse scalar.
			j = -(1.0f + e) * contactVelocityMag / denom;

			// If there are multiple contacts, distribute the impulse evenly.
			if (contactCount > 1) {
				j /= static_cast<float>(contactCount);
			}
		}
		// Compute the impulse vector.
		jList.push_back(j);
		glm::vec3 impulse = j * normal;
		impulseList.push_back(impulse);
	}

	// Apply the computed impulses to the bodies.
	for (size_t i = 0; i < impulseList.size(); ++i) {
		glm::vec3 impulse = impulseList[i];
		glm::vec3 ra = raList[i];
		glm::vec3 rb = rbList[i];

		// Update linear velocities: subtract impulse on A, add impulse on B.
		bodyA->setLinearVelocity(bodyA->getLinearVelocity() - impulse * bodyA->invMass);
		bodyB->setLinearVelocity(bodyB->getLinearVelocity() + impulse * bodyB->invMass);

		// Compute the angular impulse contributions.
		glm::vec3 angularImpulseA = bodyA->invIntertiaTensor * glm::cross(ra, impulse);
		if (bodyA->isStatic) {
			angularImpulseA = glm::vec3(0.0f);
		}
		glm::vec3 angularImpulseB = bodyB->invIntertiaTensor * glm::cross(rb, impulse);
		if (bodyB->isStatic) {
			angularImpulseB = glm::vec3(0.0f);
		}

		// Update angular velocities accordingly.
		bodyA->setAngularVelocity(bodyA->getAngularVelocity() - angularImpulseA);
		bodyB->setAngularVelocity(bodyB->getAngularVelocity() + angularImpulseB);
	}

	if (glm::length(bodyA->getAngularVelocity()) < 0.0001f) {
		bodyA->setAngularVelocity(glm::vec3(0.0f));
	}
	if (glm::length(bodyB->getAngularVelocity()) < 0.0001f) {
		bodyB->setAngularVelocity(glm::vec3(0.0f));
	}
	if (glm::length(bodyA->getLinearVelocity()) < 0.0001f) {
		bodyA->setLinearVelocity(glm::vec3(0.0f));
	}
	if (glm::length(bodyB->getLinearVelocity()) < 0.0001f) {
		bodyB->setLinearVelocity(glm::vec3(0.0f));
	}

	for (int i = 0; i < contactCount; ++i) {
		glm::vec3 ra = raList[i];
		glm::vec3 rb = rbList[i];

		// Calculate the velocity at the contact point due to both linear and angular motion.
		glm::vec3 angularVelocityA = glm::cross(bodyA->getAngularVelocity(), ra); // Angular velocity at point A
		glm::vec3 angularVelocityB = glm::cross(bodyB->getAngularVelocity(), rb); // Angular velocity at point B

		// Relative velocity at the points of contact
		glm::vec3 relativeVelocity = (bodyB->getLinearVelocity() + angularVelocityB) -
                             (bodyA->getLinearVelocity() + angularVelocityA);

		// Remove the normal component to isolate the tangential (sliding) component.
		glm::vec3 tangent = relativeVelocity - glm::dot(relativeVelocity, normal) * normal;

		// If the tangential component is nearly zero, skip friction for this contact.
		if (glm::length(tangent) < 0.0001f) {
			continue;
		}
		else {
			tangent = glm::normalize(tangent);
		}

		// Compute the friction denominator (similar to the normal impulse denominator but along the tangent)
		glm::vec3 raCrossT = glm::cross(ra, tangent);
		glm::vec3 rbCrossT = glm::cross(rb, tangent);
		glm::vec3 raInertiaT = bodyA->invIntertiaTensor * raCrossT;
		glm::vec3 rbInertiaT = bodyB->invIntertiaTensor * rbCrossT;

		float frictionDenom = bodyA->invMass + bodyB->invMass +
			glm::dot(glm::cross(raInertiaT, ra), tangent) +
			glm::dot(glm::cross(rbInertiaT, rb), tangent);

		float jt = -glm::dot(relativeVelocity, tangent) / frictionDenom;
		jt /= (float)contactCount;

		float j = jList[i];

		// Compute the friction impulse vector.
		glm::vec3 frictionImpulse;

		if (fabs(jt) <= j * 0.6) {
			frictionImpulse = jt * tangent;
		}
		else {
			frictionImpulse = -j * tangent * 0.4f;
		}

		frictionImpulseList.push_back(frictionImpulse);

	}
	for (size_t i = 0; i < frictionImpulseList.size(); ++i) {
		glm::vec3 ra = raList[i];
		glm::vec3 rb = rbList[i];
		glm::vec3 frictionImpulse = frictionImpulseList[i];

		// Apply friction impulse to the linear velocities.
		bodyA->setLinearVelocity(bodyA->getLinearVelocity() - frictionImpulse * bodyA->invMass);
		bodyB->setLinearVelocity(bodyB->getLinearVelocity() + frictionImpulse * bodyB->invMass);

		// Apply friction impulse to the angular velocities.
		glm::vec3 angularImpulseA = bodyA->invIntertiaTensor * glm::cross(ra, frictionImpulse);
		glm::vec3 angularImpulseB = bodyB->invIntertiaTensor * glm::cross(rb, frictionImpulse);
		bodyA->setAngularVelocity(bodyA->getAngularVelocity() - angularImpulseA);
		bodyB->setAngularVelocity(bodyB->getAngularVelocity() + angularImpulseB);
	}

}


void World::Step(float time, int iterations) {
	for (int i = 0; i < iterations; ++i) {
        contactPairs.clear();

        StepBodies(time, iterations);
        BroadPhase();
        NarrowPhase();
    }
}

void World::BroadPhase() {
	for (int i = 0; i < bodyList.size(); ++i) {
		std::shared_ptr<RigidBody> bodyA = bodyList[i];
		AABB bodyAAabb = bodyA->getAABB();

		for (int j = i + 1; j < bodyList.size(); ++j) {
			std::shared_ptr<RigidBody> bodyB = bodyList[j];
			AABB bodyBAabb = bodyB->getAABB();

			if (bodyA->isStatic && bodyB->isStatic) {
				continue;
			}

			if (AABB::intersectAABBs(bodyAAabb, bodyBAabb)) {
				contactPairs.push_back(ContactPair(i, j));;
			}
		}
	}
}

void World::NarrowPhase() {
    for (int i = 0; i < contactPairs.size(); ++i) {
		ContactPair currPair = contactPairs[i];
		std::shared_ptr<RigidBody> bodyA = bodyList[currPair.object1];
		std::shared_ptr<RigidBody> bodyB = bodyList[currPair.object2];

		glm::vec3 normal;
		float depth = 0.0f;

		// Perform the collision check
		if (Collisions::collide(bodyA, bodyB, normal, depth)) {
			std::vector<glm::vec3> collisionPoints;
			int collisionCount;

			// Find contact points
			Collisions::findContactPoints(bodyA, bodyB, normal, depth, collisionPoints, collisionCount);
			
			// Create a collision manifold
			CollisionManifold contact(bodyA, bodyB, depth, normal, collisionPoints, collisionCount);

			resolveCollisions(contact);
			seperateBodies(bodyA, bodyB, (normal * depth));
        };
    }
}

void World::checkCollision(ContactPair currPair) {
}

void World::StepBodies(float time, int iterations) {
	for (int i = 0; i < bodyList.size(); ++i) {
		if (!bodyList[i]->isStatic) {
			bodyList[i]->Step(time, GRAVITY_CONSTANT, iterations);
		}
	}
}
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

