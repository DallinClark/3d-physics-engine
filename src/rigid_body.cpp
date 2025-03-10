#include "../include/rigid_body.h"
#include "../include/world.h"

RigidBody::RigidBody(glm::vec3 position, float density, float mass, float restitution, float area,
    bool isStatic, float radius, float width, float height, float depth, ShapeType shapeType, glm::vec3 color, std::shared_ptr<Mesh> mesh)
    : position(position), density(density), mass(mass), restitution(restitution), area(area), depth(depth),
    isStatic(isStatic), radius(radius), width(width), height(height), shapeType(shapeType), color(color), mesh(mesh){

    linearVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
    angularVelocity = 0.0f;
    rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    inertia = CalculateRotationalInertia();

    staticFriction = 0.6f;
    dynamicFriction = 0.4f;

    force = glm::vec2(0.0f, 0.0f);

    transformUpdateRequired = true;
    aabbUpdateRequired = true;

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


bool RigidBody::CreateCircleBody(float radius, glm::vec3 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh) {
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

    body = std::make_shared<RigidBody>(position, density, mass, restitution, area, isStatic, radius, 0.0f, 0.0f, 0.0f, ShapeType::Sphere, getRandomColor(), mesh);

    return true;
}

bool RigidBody::CreateSquareBody(float width, float height, float depth, glm::vec3 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh) {
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

    body = std::make_shared<RigidBody>(position, density, mass, restitution, area, isStatic, 0.0f, width, height, depth, ShapeType::Cube, getRandomColor(), mesh);
    
    return true;
}

void RigidBody::Move(glm::vec3 amount) {
    position += amount;
    transformUpdateRequired = true;
    aabbUpdateRequired = true;
}

void RigidBody::rotate(float angleDegrees, const glm::vec3& axis) {
    float angleRadians = glm::radians(angleDegrees);

    // Create a quaternion representing the new rotation
    glm::quat deltaRotation = glm::angleAxis(angleRadians, glm::normalize(axis));

    // Update the object's rotation by multiplying quaternions
    rotation = deltaRotation * rotation;

    // Normalize to prevent drift
    rotation = glm::normalize(rotation);

    transformUpdateRequired = true;
}

glm::mat4 RigidBody::getTransformMatrix() {
    if (transformUpdateRequired) {
        glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
        glm::mat4 rotationMatrix = glm::mat4_cast(rotation);

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
    
    //glm::vec2 acc = force / (mass / 1000.0f); // conversion to kilograms, cm/s^2
    //linearVelocity += acc * time;  // cm/s

    time /= (float)iterations;

    linearVelocity += gravity * time;
    position += linearVelocity * time;

    //angle += angularVelocity * time;

    force = glm::vec2(0.0f, 0.0f);

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