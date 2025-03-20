#include "../include/setup.h"

void Setup::makeBallFalling(std::shared_ptr<World> engine, Texture& crateTexture, Texture& squareTexture) {
    std::shared_ptr<RigidBody> body;
    std::string errorMessage = "";
    bool success = RigidBody::CreateSquareBody(50.0f, 5.0f, 50.0f, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), squareTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(1.3f,3.3f,1.5f, glm::vec3(0.0f, 1.5f,9.5f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), crateTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);
    body->rotate(50, glm::vec3(1.0f, 0.2f, 0.4f));
    success = RigidBody::CreateCircleBody(1.5f, glm::vec3(-18.0f, 2.0f, 9.5f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), crateTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(1.5, 2.3f, 0.7f, glm::vec3(-11.0f, 1.0f, 9.5f), 1.0f, false, 0.8f, body, errorMessage, engine->getSquareMesh(), crateTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    body->rotate(-20, glm::vec3(0.0f, 0.0f, 1.0f));
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(7.5, 0.3f, 5.0f, glm::vec3(-15.0f, 0.0f, 9.5f), 1.0f, true, 0.8f, body, errorMessage, engine->getSquareMesh(), crateTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    body->rotate(-20, glm::vec3(0.0f, 0.0f, 1.0f));
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(3.0, 3.0f, 3.0f, glm::vec3(-3.0f, 0.0f, 9.5f), 0.5, false, 0.8f, body, errorMessage, engine->getSquareMesh(), crateTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);
    body->rotate(20,glm::vec3(0.5f,0.0f,0.0f));

    success = RigidBody::CreateSquareBody(1.0, 1.0f, 1.0f, glm::vec3(10.0f, 0.0f, 9.5f), 0.5, false, 0.8f, body, errorMessage, engine->getSquareMesh(), crateTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);
    body->rotate(20,glm::vec3(0.5f,0.0f,0.0f));
    body->setLinearVelocity(glm::vec3(-10.0f,0.0f,0.0f));

    success = RigidBody::CreateSquareBody(1.5f, 1.5f, 1.5f, glm::vec3(-3.0f, 5.0f, 11.0f), 0.5f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), crateTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);
    body->rotate(45,glm::vec3(0.0f,1.0f,0.0f));
    body->rotate(20,glm::vec3(1.0f,0.0f,0.0f));
}

void Setup::makeJenga(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture) {
    std::shared_ptr<RigidBody> body;
    std::string errorMessage = "";
    bool success = RigidBody::CreateSquareBody(50.0f, 5.0f, 50.0f, glm::vec3(0.0f, -2.5f, 0.0f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(2.0f,1.0f,8.0f, glm::vec3(0.0f, 0.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(2.0f,1.0f,8.0f, glm::vec3(-5.0f, 0.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(8.0f,1.0f,2.0f, glm::vec3(-2.5f, 1.5f,3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(8.0f,1.0f,2.0f, glm::vec3(-2.5f, 1.5f,-3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(2.0f,1.0f,8.0f, glm::vec3(0.0f, 2.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(2.0f,1.0f,8.0f, glm::vec3(-5.0f, 2.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);
}

void Setup::makeAngryBirds(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, std::shared_ptr<Camera> camera) {
    std::shared_ptr<RigidBody> body;
    std::string errorMessage = "";
    bool success = RigidBody::CreateSquareBody(50.0f, 5.0f, 50.0f, glm::vec3(0.0f, -2.5f, 0.0f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(10.0f,50.0f,50.0f, glm::vec3(-30.0f, 22.5f,0.0f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);
    success = RigidBody::CreateSquareBody(50.0f,50.0f,10.0f, glm::vec3(0.0f, 22.5f,-30.0f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(2.0f,1.0f,8.7f, glm::vec3(-5.0f, 5.2f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(2.0f,5.0f,2.3f, glm::vec3(-5.0f, 2.5f,-3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(1.5f,5.0f,1.2f, glm::vec3(-5.2f, 2.5f,3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.5f, glm::vec3(20.0, 5.0f, -3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    body->setLinearVelocity(glm::vec3(-25.0f,0.0f,0.0f));

    success = RigidBody::CreateSquareBody(2.0f,7.0f,2.3f, glm::vec3(-5.0f, 2.5f,-9.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(2.0f,1.0f,8.7f, glm::vec3(-5.0f, 7.2f,-6.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    camera->setPosition(glm::vec3(20.0f,10.0f,10.0f));
    camera->lookAt(glm::vec3(0.0f, 0.0f,0.0f));
}

