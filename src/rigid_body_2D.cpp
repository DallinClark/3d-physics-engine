#include "../include/rigid_body_2D.h"
#include "../include/engine_2D.h"

RigidBody2D::RigidBody2D(glm::vec2 position, float density, float mass, float restitution, float area,
    bool isStatic, float radius, float width, float height, ShapeType shapeType, glm::vec3 color, std::shared_ptr<Mesh> mesh)
    : position(position), density(density), mass(mass), restitution(restitution), area(area),
    isStatic(isStatic), radius(radius), width(width), height(height), shapeType(shapeType), color(color), mesh(mesh){

    linearVelocity = glm::vec2(0.0f, 0.0f);
    angularVelocity = 0.0f;
    angle = 0.0f;
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

glm::vec3 RigidBody2D::getRandomColor() {
    static std::random_device rd;  // Seed for random number engine
    static std::mt19937 gen(rd()); // Mersenne Twister PRNG
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f); // Range [0,1]

    return glm::vec3(dist(gen), dist(gen), dist(gen));
}

float RigidBody2D::CalculateRotationalInertia() {
    if (shapeType == ShapeType::Square) {
        return ((1.0f / 12.0f) * mass * (width * width + height * height));
    }
    else if (shapeType == ShapeType::Circle) {
        return ((1.0f / 2.0f) * mass * radius * radius);
    }
}


bool RigidBody2D::CreateCircleBody(float radius, glm::vec2 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody2D>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh) {
    body = nullptr;
    errorMessage = "";

    float area = M_PI * (radius * radius);



    if (area < Engine2D::MIN_BODY_SIZE) {
        errorMessage = "CIRCLE RADIUS TOO SMALL";
        return false;
    }

    if (area > Engine2D::MAX_BODY_SIZE) {
        errorMessage = "CIRCLE RADIUS TOO LARGE";
        return false;
    }

    if (density < Engine2D::MIN_DENSITY) {
        errorMessage = "DENSITY IS TOO SMALL";
        return false;
    }

    if (density > Engine2D::MAX_DENSITY) {
        errorMessage = "DENSITY IS TOO LARGE";
        return false;
    }

    restitution = glm::clamp(restitution, 0.0f, 1.0f);

    float mass = area * density; // mass is in grams, density in g/cm^2


    body = std::make_shared<RigidBody2D>(position, density, mass, restitution, area, isStatic, radius, 0.0f, 0.0f, ShapeType::Circle, getRandomColor(), mesh);

    return true;
}

bool RigidBody2D::CreateSquareBody(float width, float height, glm::vec2 position, float density, bool isStatic, float restitution, std::shared_ptr<RigidBody2D>& body, std::string& errorMessage, std::shared_ptr<Mesh> mesh) {
    body = nullptr;
    errorMessage = "";

    float area = width * height;

    if (area < Engine2D::MIN_BODY_SIZE) {
        errorMessage = "AREA TOO SMALL";
        return false;
    }

    if (area > Engine2D::MAX_BODY_SIZE) {
        errorMessage = "AREA TOO LARGE";
        return false;
    }

    if (density < Engine2D::MIN_DENSITY) {
        errorMessage = "DENSITY IS TOO SMALL";
        return false;
    }

    if (density > Engine2D::MAX_DENSITY) {
        errorMessage = "DENSITY IS TOO LARGE";
        return false;
    }

    restitution = glm::clamp(restitution, 0.0f, 1.0f);

    float mass = area * density; // also * depth

    body = std::make_shared<RigidBody2D>(position, density, mass, restitution, area, isStatic, 0.0f, width, height, ShapeType::Square, getRandomColor(), mesh);
    
    return true;
}

void RigidBody2D::Move(glm::vec2 amount) {
    position += amount;
    transformUpdateRequired = true;
    aabbUpdateRequired = true;
}

void RigidBody2D::Rotate(float amount) {
    angle += amount;
    transformUpdateRequired = true;
    aabbUpdateRequired = true;
}

glm::mat4 RigidBody2D::getTransformMatrix() {
    if (transformUpdateRequired) {
        glm::mat4 trans = glm::mat4(1.0f);
        trans = glm::translate(trans, glm::vec3(position.x, position.y, 0.0f));
        trans = glm::rotate(trans, angle, glm::vec3(0.0f, 0.0f, 1.0f));
        if (shapeType == ShapeType::Square) {
            trans = glm::scale(trans, glm::vec3(width, height, 1.0f));
        }
        else if (shapeType == ShapeType::Circle) {
            trans = glm::scale(trans, glm::vec3(radius, radius, 1.0f));
        }
        transformMatrix = trans;
        transformUpdateRequired = false;
    }
    return transformMatrix;
}

vector<glm::vec4> RigidBody2D::getTransformedVertices() {
    vector<glm::vec4> vertices = mesh->getVertexPositions();
    glm::mat4 trans = getTransformMatrix();
    for (int i = 0; i < vertices.size(); ++i) {
        vertices[i] = trans * vertices[i];
    }
    return vertices;
}

void RigidBody2D::Step(float time, glm::vec2 gravity, int iterations) {

    if (isStatic) {
        return;
    }
    
    //glm::vec2 acc = force / (mass / 1000.0f); // conversion to kilograms, cm/s^2
    //linearVelocity += acc * time;  // cm/s

    time /= (float)iterations;

    linearVelocity += gravity * time;
    position += linearVelocity * time;

    angle += angularVelocity * time;

    force = glm::vec2(0.0f, 0.0f);

    transformUpdateRequired = true;
    aabbUpdateRequired = true;
}

void RigidBody2D::AddForce(glm::vec2 amount) {
    force = amount;
}

AABB RigidBody2D::getAABB() {
    if (aabbUpdateRequired) {
        float minX = 99999.9f;
        float minY = 99999.9f;
        float maxX = -99999.9f;
        float maxY = -99999.9f;

        if (shapeType == ShapeType::Square) {
            vector<glm::vec4> vertices = getTransformedVertices();

            for (int i = 0; i < vertices.size(); ++i) {
                glm::vec4 v = vertices[i];

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