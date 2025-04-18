#include "../../include/physics/rigid_body.h"
#include "../../include/physics/world.h"

RigidBody::RigidBody(glm::vec3 position, float density, float mass, float restitution, float area,
    bool isStatic, float radius, float width, float height, float depth, ShapeType shapeType, std::shared_ptr<Mesh> mesh, Texture& texture)
    : position(position), density(density), mass(mass), restitution(restitution), area(area), depth(depth),
    isStatic(isStatic), radius(radius), width(width), height(height), shapeType(shapeType), mesh(mesh), texture(texture) {

    linearVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
    angularVelocity = glm::vec3();
    rotation = glm::mat4(1.0f);
    inertia = CalculateRotationalInertia();

    staticFriction = 0.6f;
    dynamicFriction = 0.4f;

    force = glm::vec2(0.0f, 0.0f);

    transformUpdateRequired = true;
    aabbUpdateRequired = true;

    inertiaTensor = computeInertiaTensor();
    invIntertiaTensor = glm::inverse(inertiaTensor);
    
    if (isStatic) {
        invMass = 0.0f;
        invInertia = 0.0f;
    }
    else {
        invMass = 1.0f / mass;
        invInertia = 1.0f / inertia;
    }

}


float RigidBody::CalculateRotationalInertia() {
    if (shapeType == ShapeType::Cube) {
        return ((1.0f / 12.0f) * mass * (width * width + height * height));
    }
    else if (shapeType == ShapeType::Tetrahedron) {
        return 0.0f;
    }
    else if (shapeType == ShapeType::Sphere) {
        return ((1.0f / 2.0f) * mass * radius * radius);
    }
    return 0.0f; // SHOULDN'T REACH HERE
}


bool RigidBody::CreateCircleBody(float radius, glm::vec3 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh, Texture& texture) {
    body = nullptr;
    errorMessage = "";

    float area = (4.0f /3.0f) * M_PI * (radius * radius * radius);

    if (area < World::MIN_BODY_SIZE) {
        errorMessage = "CIRCLE RADIUS TOO SMALL";
        return false;
    }

    if (area > World::MAX_BODY_SIZE) {
        errorMessage = "CIRCLE RADIUS TOO LARGE";
        return false;
    }

    if (density < World::MIN_DENSITY) {
        errorMessage = "DENSITY IS TOO SMALL";
        return false;
    }

    if (density > World::MAX_DENSITY) {
        errorMessage = "DENSITY IS TOO LARGE";
        return false;
    }

    restitution = glm::clamp(restitution, 0.0f, 1.0f);

    float mass = density * area; 

    body = std::make_shared<RigidBody>(position, density, mass, restitution, area, isStatic, radius, 0.0f, 0.0f, 0.0f, ShapeType::Sphere, mesh, texture);

    return true;
}
bool RigidBody::CreateTetrahedronBody(float width, float height, float depth, glm::vec3 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh, Texture& texture) {
    body = nullptr;
    errorMessage = "";

    float area = width * height;

    if (area < World::MIN_BODY_SIZE) {
        errorMessage = "AREA TOO SMALL";
        return false;
    }

    if (area > World::MAX_BODY_SIZE) {
        errorMessage = "AREA TOO LARGE";
        return false;
    }

    if (density < World::MIN_DENSITY) {
        errorMessage = "DENSITY IS TOO SMALL";
        return false;
    }

    if (density > World::MAX_DENSITY) {
        errorMessage = "DENSITY IS TOO LARGE";
        return false;
    }

    restitution = glm::clamp(restitution, 0.0f, 1.0f);

    float mass = area * density * depth * 0.5f; 
    if (isStatic) {
        mass = FLT_MAX;
    }


    body = std::make_shared<RigidBody>(position, density, mass, restitution, area, isStatic, 0.0f, width, height, depth, ShapeType::Tetrahedron, mesh, texture);
    return true;
}

bool RigidBody::CreateSquareBody(float width, float height, float depth, glm::vec3 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh, Texture& texture) {
    body = nullptr;
    errorMessage = "";

    float area = width * height;

    if (area < World::MIN_BODY_SIZE) {
        errorMessage = "AREA TOO SMALL";
        return false;
    }

    if (area > World::MAX_BODY_SIZE) {
        errorMessage = "AREA TOO LARGE";
        return false;
    }

    if (density < World::MIN_DENSITY) {
        errorMessage = "DENSITY IS TOO SMALL";
        return false;
    }

    if (density > World::MAX_DENSITY) {
        errorMessage = "DENSITY IS TOO LARGE";
        return false;
    }

    restitution = glm::clamp(restitution, 0.0f, 1.0f);

    float mass = area * density * depth; 
    if (isStatic) {
        mass = FLT_MAX;
    }

    body = std::make_shared<RigidBody>(position, density, mass, restitution, area, isStatic, 0.0f, width, height, depth, ShapeType::Cube, mesh, texture);
    
    return true;
}

bool RigidBody::CreateDiamondBody(float width, float height, float depth, glm::vec3 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh, Texture& texture) {
    body = nullptr;
    errorMessage = "";

    float area = width * height;

    if (area < World::MIN_BODY_SIZE) {
        errorMessage = "AREA TOO SMALL";
        return false;
    }

    if (area > World::MAX_BODY_SIZE) {
        errorMessage = "AREA TOO LARGE";
        return false;
    }

    if (density < World::MIN_DENSITY) {
        errorMessage = "DENSITY IS TOO SMALL";
        return false;
    }

    if (density > World::MAX_DENSITY) {
        errorMessage = "DENSITY IS TOO LARGE";
        return false;
    }

    restitution = glm::clamp(restitution, 0.0f, 1.0f);

    float mass = area * density * depth * 0.5; 
    if (isStatic) {
        mass = FLT_MAX;
    }

    body = std::make_shared<RigidBody>(position, density, mass, restitution, area, isStatic, 0.0f, width, height, depth, ShapeType::Diamond, mesh, texture);
    
    return true;
}

void RigidBody::Move(glm::vec3 amount) {
    position += amount;
    transformUpdateRequired = true;
    aabbUpdateRequired = true;
}

glm::mat3 gramSchmidtOrthonormalize(const glm::mat3& A) {
    glm::vec3 v1 = A[0]; // First column vector
    glm::vec3 v2 = A[1]; // Second column vector
    glm::vec3 v3 = A[2]; // Third column vector

    // Step 1: Orthonormalize the first column (v1)
    v1 = glm::normalize(v1);  // Normalize v1

    // Step 2: Orthonormalize the second column (v2) relative to v1
    v2 = v2 - glm::dot(v2, v1) * v1; // Subtract projection onto v1
    v2 = glm::normalize(v2);  // Normalize v2

    // Step 3: Orthonormalize the third column (v3) relative to v1 and v2
    v3 = v3 - glm::dot(v3, v1) * v1; // Subtract projection onto v1
    v3 = v3 - glm::dot(v3, v2) * v2; // Subtract projection onto v2
    v3 = glm::normalize(v3);  // Normalize v3

    // Step 4: Create a new matrix with orthonormal columns
    glm::mat3 orthonormalizedMatrix;
    orthonormalizedMatrix[0] = v1;
    orthonormalizedMatrix[1] = v2;
    orthonormalizedMatrix[2] = v3;

    return orthonormalizedMatrix;
}

void RigidBody::rotate(float angleDegrees, const glm::vec3& axis) {
    float angleRadians = glm::radians(angleDegrees);

    // Create a rotation matrix using glm::rotate (converts axis-angle to mat4)
    glm::mat4 deltaRotation = glm::rotate(glm::mat4(1.0f), angleRadians, glm::normalize(axis));

    // Apply the rotation to the current rotation matrix
    rotation = deltaRotation * rotation;  // Local rotation 

    rotation = gramSchmidtOrthonormalize(rotation);

    transformUpdateRequired = true;
}


glm::mat4 RigidBody::getTransformMatrix() {
    if (transformUpdateRequired) {
        glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
        glm::mat4 rotationMatrix = rotation;

        glm::vec3 scaleVec(1.0f);
        if (shapeType == ShapeType::Cube || shapeType == ShapeType::Tetrahedron || shapeType == ShapeType::Diamond) {
            scaleVec = glm::vec3(width, height, depth);
        }
        else if (shapeType == ShapeType::Sphere) {
            scaleVec = glm::vec3(radius);
        }

        worldMatrix = trans * rotationMatrix * glm::scale(glm::mat4(1.0f), scaleVec);
        transformUpdateRequired = false;
    }
    return worldMatrix;
}


//TODO fix this so it combines with getTransformedVertices
vector<Face> RigidBody::getFaces() {
    vector<Face> faces = mesh->faces;
    vector<Face> outFaces;
    glm::mat4 transform = getTransformMatrix();

    for (Face face : faces) {
        for (auto& vertex : face.vertices) {
            glm::vec4 transformed = transform * glm::vec4(vertex, 1.0f);
            vertex = glm::vec3(transformed); 
        }

        // Update the normal
        glm::vec3 edge1 = face.vertices[1] - face.vertices[0];
        glm::vec3 edge2 = face.vertices[face.vertices.size() - 1] - face.vertices[0];
        face.normal = glm::normalize(glm::cross(edge1, edge2));
        outFaces.push_back(face);
    }
    return outFaces;
};


vector<glm::vec3> RigidBody::getTransformedVertices() {
    vector<glm::vec4> vertices = mesh->getVertexPositions();
    vector<glm::vec3> outVertices;
    glm::mat4 trans = getTransformMatrix();
    for (int i = 0; i < vertices.size(); ++i) {
        outVertices.push_back(glm::vec3(trans * vertices[i]));
    }
    return outVertices;
}

void RigidBody::Step(float time, glm::vec3 gravity, int iterations) {

    if (isStatic) {
        return;
    }
   
    time /= (float)iterations;

    linearVelocity += gravity * time;
    position += linearVelocity * time;

    if (glm::length(angularVelocity) > 0.0f) {  
        float angle = glm::length(angularVelocity) * time; // Get the rotation magnitude
        glm::vec3 axis = glm::normalize(angularVelocity);   // The rotation axis

        // Construct the small-angle rotation matrix
        glm::mat4 deltaRotation = glm::mat4(glm::rotate(glm::mat4(1.0f), angle, axis));

        // Apply rotation update
        rotation = deltaRotation * rotation;

        // Re-orthogonalization using Gram-Schmidt process
        rotation = gramSchmidtOrthonormalize(rotation);
    }

    transformUpdateRequired = true;
    aabbUpdateRequired = true;
}

void RigidBody::AddForce(glm::vec2 amount) {
    force = amount;
}

AABB RigidBody::getAABB() {
    if (aabbUpdateRequired) {
        float minX = FLT_MAX;
        float minY = FLT_MAX;
        float minZ = FLT_MAX;
        float maxX = -FLT_MAX;
        float maxY = -FLT_MAX;
        float maxZ = -FLT_MAX;

        if (shapeType == ShapeType::Cube || shapeType == ShapeType::Tetrahedron || shapeType == ShapeType::Diamond) {
            vector<glm::vec3> vertices = getTransformedVertices();

            for (int i = 0; i < vertices.size(); ++i) {
                glm::vec3 v = vertices[i];

                if (v.x < minX) { minX = v.x; }
                if (v.y < minY) { minY = v.y; }
                if (v.z < minZ) { minZ = v.z; }
                if (v.x > maxX) { maxX = v.x; }
                if (v.y > maxY) { maxY = v.y; }
                if (v.z > maxZ) { maxZ = v.z; }
            }
        }
        else {
            minX = position.x - radius;
            minY = position.y - radius;
            minZ = position.z - radius;
            maxX = position.x + radius;
            maxY = position.y + radius;
            maxZ = position.z + radius;
        }
        this->aabb = AABB(minX, minY, minZ, maxX, maxY, maxZ);
        aabbUpdateRequired = false;
    }
    return this->aabb;
}

glm::mat3 RigidBody::computeInertiaTensor() {
    if (isStatic) {
        return glm::mat3(99999.0f);
    }
    if (shapeType == ShapeType::Cube) {
        float w = width;
        float h = height;
        float d = depth;

        float Ixx = (1.0f / 12.0f) * mass * (h * h + d * d);
        float Iyy = (1.0f / 12.0f) * mass * (w * w + d * d);
        float Izz = (1.0f / 12.0f) * mass * (w * w + h * h);

        return glm::mat3(
            Ixx, 0.0f, 0.0f,
            0.0f, Iyy, 0.0f,
            0.0f, 0.0f, Izz
        );
    }
    else if (shapeType == ShapeType::Tetrahedron) {
        float w = width;
        float h = height;
        float d = depth;

        float Ixx = (1.0f / 20.0f) * mass * (h * h + d * d);
        float Iyy = (1.0f / 20.0f) * mass * (w * w + d * d);
        float Izz = (1.0f / 20.0f) * mass * (w * w + h * h);

        return glm::mat3(
            Ixx, 0.0f, 0.0f,
            0.0f, Iyy, 0.0f,
            0.0f, 0.0f, Izz
        );
    }
    else if (shapeType == ShapeType::Sphere) {
        float r = radius; 
        float I = (2.0f / 5.0f) * mass * r * r;
        return glm::mat3(
            I, 0.0f, 0.0f,
            0.0f, I, 0.0f,
            0.0f, 0.0f, I
        );
    }
    // TODO make more accurate
    else if (shapeType == ShapeType::Diamond) {
        float w = width;
        float h = height;
        float d = depth;

        float Ixx = (1.0f / 12.0f) * mass * (h * h + d * d);
        float Iyy = (1.0f / 12.0f) * mass * (w * w + d * d);
        float Izz = (1.0f / 12.0f) * mass * (w * w + h * h);

        return glm::mat3(
            Ixx, 0.0f, 0.0f,
            0.0f, Iyy, 0.0f,
            0.0f, 0.0f, Izz
        );
    }
    return glm::mat3(0.0f); // SHOULDN'T REACH HERE
}
