#include "../include/world.h"
#include <memory>

const float World::MAX_BODY_SIZE = 640.0f * 640.0f;
const float World::MIN_BODY_SIZE = 0.01f;
const float World::MIN_DENSITY = 0.5f;
const float World::MAX_DENSITY = 21.4f;
const glm::vec3 World::GRAVITY_CONSTANT = glm::vec3(0.0f, -9.8, 0.0f);

const int World::SPHERE_STACK_COUNT = 30;
const int World::SPHERE_SECTOR_COUNT = 30;

World::World() {
	createMeshes();
}

void World::createMeshes() {
	// Rectangle Mesh

	// vertices with tex coords for OpenGL
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

	// used to make face structs, counter-clockwise
	std::vector<int> faces = {
	1, 3, 2, 0,  // Front 
	5, 7, 6, 4,  // Back 
	0, 2, 7, 5,  // Left 
	1, 4, 6, 3,  // Right 
	2, 3, 6, 7,  // Top 
	0, 5, 4, 1   // Bottom 
	};

	vector<Face> faceStructs;
	for (size_t i = 0; i < faces.size(); i += 4) {
		Face face;

		// Loop over each of the 4 indices.
		for (int j = 0; j < 4; ++j) {
			int index = faces[i + j];
			face.indices.push_back(index);
			// Convert the glm::vec4 to glm::vec3 (ignoring the w component).
			glm::vec4 v4 = verticesNoDuplicates[index];
			glm::vec3 v3(v4.x, v4.y, v4.z);
			face.vertices.push_back(v3);
		}
		// Compute the face normal using the first three vertices.
		// (Assumes the vertices are ordered such that the computed normal points outward.)
		glm::vec3 edge1 = face.vertices[1] - face.vertices[0];
		glm::vec3 edge2 = face.vertices[3] - face.vertices[0];
		face.normal = glm::normalize(glm::cross(edge1, edge2));

		faceStructs.push_back(face);
	}

	std::vector<int> edges = {
		0, 1,  // Front bottom edge
		1, 3,  // Front right edge
		3, 2,  // Front top edge
		2, 0,  // Front left edge

		5, 4,  // Back bottom edge 
		4, 6,  // Back right edge
		6, 7,  // Back top edge
		7, 5,  // Back left edge

		0, 5,  // Left-bottom side edge
		1, 4,  // Right-bottom side edge
		2, 7,  // Left-top side edge
		3, 6   // Right-top side edge
	};


	std::shared_ptr<Mesh> newMesh = std::make_shared<Mesh>(vertices, indices, verticesNoDuplicates, ShapeType::Cube);
	newMesh->setFaces(faceStructs);
	newMesh->setEdges(edges);
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
    xy = radius * cosf(stackAngle);                    // r * cos(u)
    z = radius * sinf(stackAngle);                     // r * sin(u)

    // Add (sectorCount + 1) vertices per stack
    for (int j = 0; j <= SPHERE_SECTOR_COUNT; ++j) {
        sectorAngle = j * sectorStep;                  // starting from 0 to 2pi

        // Vertex position (x, y, z)
        x = xy * cosf(sectorAngle);                    // r * cos(u) * cos(v)
        y = xy * sinf(sectorAngle);                    // r * cos(u) * sin(v)

        // Vertex tex coord (u, v) range between [0, 1]
        u = (float)j / SPHERE_SECTOR_COUNT;
        v = (float)i / SPHERE_STACK_COUNT;

        // Push the vertex with position, tex coord, and normal
        vertices.push_back({ glm::vec3{x, y, z}, glm::vec2{u, v} });
        
        // Optionally, push the vertex without duplicates (assuming this is part of the structure you're using)
        verticesNoDuplicates.push_back({ glm::vec4{x, y, z, 1}});
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

	std::shared_ptr<Mesh> newSphereMesh = std::make_shared<Mesh>(vertices, indices,verticesNoDuplicates, ShapeType::Sphere);
	meshes[ShapeType::Sphere] = newSphereMesh;
}

void World::draw(Shader& shader, glm::mat4 view, glm::mat4 projection) {
	for (int i = 0; i < bodyList.size(); ++i) {
		std::shared_ptr<RigidBody> currBody = bodyList[i];

		glm::mat4 world = currBody->getTransformMatrix();

		Texture& currTexture = currBody->texture;
		currTexture.use();
		shader.setInt("texture1", 0);

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
	// double broadPhaseAverage = 0.0;
	// double narrowPhaseAverage = 0.0;

	// using std::chrono::high_resolution_clock;
    // using std::chrono::duration_cast;
    // using std::chrono::duration;
    // using std::chrono::milliseconds;

	for (int i = 0; i < iterations; ++i) {
        contactPairs.clear();

        StepBodies(time, iterations);

        //auto broadStart = high_resolution_clock::now(); 
        BroadPhase();
        //auto broadEnd = high_resolution_clock::now(); 

        //auto narrowStart = high_resolution_clock::now();  
        NarrowPhase();
        //auto narrowEnd = high_resolution_clock::now();  

        // Calculate durations for each phase in milliseconds
        // broadPhaseAverage += duration_cast<milliseconds>(broadEnd - broadStart).count();
        // narrowPhaseAverage += duration_cast<milliseconds>(narrowEnd - narrowStart).count();
    }

	// narrowPhaseAverage /= (double)iterations;
	// broadPhaseAverage /= (double)iterations;

	// std::cout << "Broad Phase Time: " << broadPhaseAverage << "\n";
	// std::cout << "Narrow Phase Time: " << narrowPhaseAverage << "\n";

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
    std::vector<std::thread> threadList;

    for (int i = 0; i < contactPairs.size(); ++i) {
		//threadList.push_back(std::thread([this, i]() {
		checkAndResolveCollisions(contactPairs[i]);
		//}));
	}

    // Wait for all threads to finish
    for (auto& t : threadList) {
        t.join();
    }
}

void World::checkAndResolveCollisions(ContactPair currPair) {
	std::shared_ptr<RigidBody> bodyA = bodyList[currPair.object1];
	std::shared_ptr<RigidBody> bodyB = bodyList[currPair.object2];

	glm::vec3 normal;
	float depth = 0.0f;

	bool locked = true;

	bodyA->lockMutex();
	bodyB->lockMutex();

	if (Collisions::collide(bodyA, bodyB, normal, depth)) {
		vector<glm::vec3> collisionPoints;
		int collisionCount;

		Collisions::findContactPoints(bodyA, bodyB, normal, depth, collisionPoints, collisionCount);
		CollisionManifold contact(bodyA, bodyB, depth, normal, collisionPoints, collisionCount);

		resolveCollisions(contact);

		seperateBodies(bodyA, bodyB, (normal * depth));
	}

	bodyA->unlockMutex();
	bodyB->unlockMutex();
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

