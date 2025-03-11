#include "../include/rigid_body.h"
#include "../include/world.h"

RigidBody::RigidBody(glm::vec3 position, float density, float mass, float restitution, float area,
    bool isStatic, float radius, float width, float height, float depth, ShapeType shapeType, glm::vec3 color, std::shared_ptr<Mesh> mesh, Texture& texture)
    : position(position), density(density), mass(mass), restitution(restitution), area(area), depth(depth),
    isStatic(isStatic), radius(radius), width(width), height(height), shapeType(shapeType), color(color), mesh(mesh), texture(texture) {

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

glm::vec3 RigidBody::getRandomColor() {
    static std::random_device rd;  // Seed for random number engine
    static std::mt19937 gen(rd()); // Mersenne Twister PRNG
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f); // Range [0,1]

    return glm::vec3(dist(gen), dist(gen), dist(gen));
}

float RigidBody::CalculateRotationalInertia() {
    if (shapeType == ShapeType::Cube) {
        return ((1.0f / 12.0f) * mass * (width * width + height * height));
    }
    else if (shapeType == ShapeType::Sphere) {
        return ((1.0f / 2.0f) * mass * radius * radius);
    }
}


bool RigidBody::CreateCircleBody(float radius, glm::vec3 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh, Texture& texture) {
    body = nullptr;
    errorMessage = "";

    float area = M_PI * (radius * radius);

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

    float mass = area * density; // mass is in grams, density in g/cm^2

    body = std::make_shared<RigidBody>(position, density, mass, restitution, area, isStatic, radius, 0.0f, 0.0f, 0.0f, ShapeType::Sphere, getRandomColor(), mesh, texture);

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

    float mass = area * density; // also * depth

    body = std::make_shared<RigidBody>(position, density, mass, restitution, area, isStatic, 0.0f, width, height, depth, ShapeType::Cube, getRandomColor(), mesh, texture);
    
    return true;
}

void RigidBody::Move(glm::vec3 amount) {
    position += amount;
    transformUpdateRequired = true;
    aabbUpdateRequired = true;
}

void RigidBody::rotate(float angleDegrees, const glm::vec3& axis) {
    float angleRadians = glm::radians(angleDegrees);

    // Create a rotation matrix using glm::rotate (converts axis-angle to mat4)
    glm::mat4 deltaRotation = glm::rotate(glm::mat4(1.0f), angleRadians, glm::normalize(axis));

    // Apply the rotation to the current rotation matrix
    rotation = deltaRotation * rotation;  // Local rotation 

    // Re-orthogonalize to prevent drift
    // Using Gram-Schmidt-like normalization
    rotation[0] = glm::normalize(rotation[0]);
    rotation[1] = glm::normalize(rotation[1] - glm::dot(rotation[1], rotation[0]) * rotation[0]);

    transformUpdateRequired = true;
}


glm::mat4 RigidBody::getTransformMatrix() {
    if (transformUpdateRequired) {
        glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
        glm::mat4 rotationMatrix = rotation;

        glm::vec3 scaleVec(1.0f);
        if (shapeType == ShapeType::Cube) {
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


//vector<glm::vec4> RigidBody::getTransformedVertices() {
//    vector<glm::vec4> vertices = mesh->getVertexPositions();
//    glm::mat4 trans = getTransformMatrix();
//    for (int i = 0; i < vertices.size(); ++i) {
//        vertices[i] = trans * vertices[i];
//    }
//    return vertices;
//}

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

        // Update the normal.
        glm::vec3 edge1 = face.vertices[1] - face.vertices[0];
        glm::vec3 edge2 = face.vertices[3] - face.vertices[0];
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
        rotation[0] = glm::normalize(rotation[0]);
        rotation[1] = glm::normalize(rotation[1] - glm::dot(rotation[1], rotation[0]) * rotation[0]);
    }

    transformUpdateRequired = true;
    aabbUpdateRequired = true;
}

void RigidBody::AddForce(glm::vec2 amount) {
    force = amount;
}

AABB RigidBody::getAABB() {
    if (aabbUpdateRequired) {
        float minX = 99999.9f;
        float minY = 99999.9f;
        float maxX = -99999.9f;
        float maxY = -99999.9f;

        if (shapeType == ShapeType::Cube) {
            vector<glm::vec3> vertices = getTransformedVertices();

            for (int i = 0; i < vertices.size(); ++i) {
                glm::vec3 v = vertices[i];

                if (v.x < minX) { minX = v.x; }
                if (v.y < minY) { minY = v.y; }
                if (v.x > maxX) { maxX = v.x; }
                if (v.y > maxY) { maxY = v.y; }
            }
        }
        else {
            minX = position.x - radius;
            minY = position.y - radius;
            maxX = position.x + radius;
            maxY = position.y + radius;
        }
        this->aabb = AABB(minX, minY, maxX, maxY);
        aabbUpdateRequired = false;
    }
    return this->aabb;
}

glm::mat3 RigidBody::computeInertiaTensor() {
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
