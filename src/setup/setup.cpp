#include "../../include/setup/setup.h"

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

    camera->setPosition(glm::vec3(15.0f,10.0f,7.0f));
    camera->lookAt(glm::vec3(0.0f, 05.0f,0.0f));
}

void Setup::makeBallPit(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, std::shared_ptr<Camera> camera) {
    std::shared_ptr<RigidBody> body;
    std::string errorMessage = "";
    bool success = RigidBody::CreateSquareBody(10.0f, 5.0f, 10.0f, glm::vec3(0.0f, 0.0f, 0.0f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(1.0f,10.0f,10.0f, glm::vec3(-5.1f, 5.0f,0.0f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(1.0f,10.0f,10.0f, glm::vec3(5.1f, 5.0f,0.0f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(10.0f,10.0f,1.0f, glm::vec3(0.0f, 5.0f,5.1f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(10.0f,10.0f,1.0f, glm::vec3(0.0f, 5.0f,-5.1f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);
    
    success = RigidBody::CreateCircleBody(1.5f, glm::vec3(0.0, 15.0f, 0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.2f, glm::vec3(2.0, 27.0f, 0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.0f, glm::vec3(-2.0, 33.0f, 1.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(0.8f, glm::vec3(2.0, 43.0f, -1.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.1f, glm::vec3(3.0, 57.0f,-2.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.4f, glm::vec3(2.0, 62.0f,-3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.5f, glm::vec3(1.0, 79.0f,-1.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.4f, glm::vec3(-3.0, 4.0f,-2.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.3f, glm::vec3(-1.0, 14.0f,-3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.5f, glm::vec3(1.0, 10.0f, 0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.2f, glm::vec3(-2.0, 17.0f, 0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.0f, glm::vec3(-2.0, 28.0f, -1.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(0.8f, glm::vec3(-2.0, 39.0f, -1.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.1f, glm::vec3(-3.0, 12.0f,-2.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.4f, glm::vec3(-2.0, 12.0f,3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.2f, glm::vec3(-2.0, 13.0f,-1.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.1f, glm::vec3(-3.0, 42.0f,2.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.3f, glm::vec3(-1.0, 54.0f,3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.0f, glm::vec3(2.0, 68.0f, -3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(0.8f, glm::vec3(2.0, 79.0f, -2.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.1f, glm::vec3(-3.0, 9.0f,2.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.4f, glm::vec3(-4.0, 2.0f,3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.2f, glm::vec3(-1.0, 10.0f,1.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.1f, glm::vec3(-1.0, 29.0f,4.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.3f, glm::vec3(-1.0, 19.0f,-3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.3f, glm::vec3(-2.0, 45.0f,1.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.1f, glm::vec3(3.0, 16.0f,2.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.3f, glm::vec3(-3.0, 26.0f,4.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.0f, glm::vec3(2.0, 36.0f, 3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(0.8f, glm::vec3(2.0, 68.0f, -3.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.1f, glm::vec3(-3.0, 22.0f,4.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    camera->setPosition(glm::vec3(5.0f,25.0f,5.0f));
    camera->lookAt(glm::vec3(-1.0f,0.0f,-1.0f));
}

void Setup::makeDiamond(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, Texture& diamondTexture, std::shared_ptr<Camera> camera) {

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

    success = RigidBody::CreateSquareBody(10.0f,1.0f,5.0f, glm::vec3(-10.0f, 10.0f,0.0f), 1.0f, true, 0.5f, body, errorMessage, engine->getSquareMesh(), groundTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);
    body->rotate(-30, glm::vec3(0.0,0.0,1.0));

    success = RigidBody::CreateDiamondBody(2.0f,2.0f,2.0f, glm::vec3(-13.0f, 30.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getDiamondMesh(), diamondTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    body->rotate(40,glm::vec3(0.3f,0.4f,0.0f));
    engine->AddBody(body);


    success = RigidBody::CreateTetrahedronBody(4.0f,2.0f,5.0f, glm::vec3(-13.2f,40.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getTetrahedronMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    //body->rotate(45,glm::vec3(0.3f,0.4f,0.0f));
    engine->AddBody(body);

    success = RigidBody::CreateTetrahedronBody(3.0f,2.0f,2.0f, glm::vec3(-13.2f, 55.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getTetrahedronMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    body->rotate(45,glm::vec3(0.3f,0.4f,0.0f));
    engine->AddBody(body);

    success = RigidBody::CreateTetrahedronBody(2.0f,2.0f,4.0f, glm::vec3(-13.2f, 25.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getTetrahedronMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    body->rotate(45,glm::vec3(0.6f,0.4f,0.0f));
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(3.0f,2.0f,3.0f, glm::vec3(-13.2f, 35.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    //body->rotate(45,glm::vec3(0.3f,0.4f,0.0f));
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(3.0f,3.0f,1.0f, glm::vec3(-13.2f, 39.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    body->rotate(45,glm::vec3(0.3f,0.4f,0.0f));
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(1.1f, glm::vec3(-13.0f, 12.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);


    camera->setPosition(glm::vec3(25.0f,20.0f,25.0f));
    camera->lookAt(glm::vec3(0.0f, 05.0f,0.0f));
}

void Setup::makeDiamondLauncher(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, Texture& diamondTexture, std::shared_ptr<Camera> camera) {
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

    success = RigidBody::CreateSquareBody(7.0f,1.0f,2.0f, glm::vec3(0.0f, -1.5f,0.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateSquareBody(2.0f,1.0f,15.0f, glm::vec3(0.0f, -.5f,4.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getSquareMesh(), boxTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    success = RigidBody::CreateDiamondBody(2.0f,2.0f,2.0f, glm::vec3(0.0f, -.5f,9.0f), 1.0f, false, 0.5f, body, errorMessage, engine->getDiamondMesh(), diamondTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    body->rotate(40,glm::vec3(0.3f,0.4f,0.0f));
    engine->AddBody(body);

    success = RigidBody::CreateCircleBody(3.0f, glm::vec3(0.0f, 20.0f,-3.0f), 2.0f, false, 0.5f, body, errorMessage, engine->getSphereMesh(), sphereTexture);
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine->AddBody(body);

    camera->setPosition(glm::vec3(35.0f,30.0f,35.0f));
    camera->lookAt(glm::vec3(0.0f, 05.0f,0.0f));
}


