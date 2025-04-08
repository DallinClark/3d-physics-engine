#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>

#include "../include/physics/world.h"
#include "../include/glm/gtc/matrix_transform.hpp"

#include "../include/rendering/shader.h"
#include "../include/setup/setup.h"
#include "../include/rendering/texture.h"
#include "../include/physics/world.h"
#include "../include/rendering/camera.h"
#include "../include/rendering/point_light.h"
#include "../include/setup/window.h"

int main()
{
    // Set up the camera and window
    glm::vec3 cameraPos(0.0f, 10.0f, 30.0f);
    glm::vec3 cameraLookAt(0.0f, 0.0f, 0.0f);
    glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);

    std::shared_ptr<Camera> camera = std::make_shared<Camera>(cameraPos, cameraLookAt, cameraUp);

    Window window(camera);

    // Set up engine and shaders
    std::shared_ptr<World> engine = std::make_shared<World>();

    Shader ourShader("resources/shaders/tex_vertex.vs", "resources/shaders/tex_fragment.fs");
    ourShader.use();

    Texture woodTexture("resources/textures/wood2.jpg", 0);
    Texture grayTexture("resources/textures/gray.jpg", 0);
    Texture ballTexture("resources/textures/dodgeball.jpg", 0);
    Texture diamondTexture("resources/textures/diamond.jpg",0);

    // Create a light, and set up rigid bodies
    PointLight light(glm::vec3(20.0f, 70.0f, 20.0f), glm::vec3(1.0f, 1.0f, 1.0f));

    // Uncomment setup to run that specific sim, set up new sims within setup class

    //Setup::makeBallFalling(engine, crateTexture, squareTexture);
    //Setup::makeJenga(engine,squareTexture,crateTexture,crateTexture);
    //Setup::makeAngryBirds(engine,squareTexture,crateTexture,silverTexture, camera);
    //Setup::makeBallPit(engine,squareTexture,crateTexture,silverTexture, camera);
    //Setup::makeDiamond(engine,grayTexture,woodTexture,ballTexture, diamondTexture, camera);
    Setup::makeDiamondLauncher(engine,grayTexture,woodTexture,ballTexture, diamondTexture, camera);

    // Rendering and simulation loop

    double currentTime = window.getTime();
    double accumulator = 0.0;

    while (!window.shouldClose()) {

        double newTime = window.getTime();
        double frameTime = newTime - currentTime;
        currentTime = newTime;
        accumulator += frameTime;

        while (accumulator >= FIXED_TIMESTEP) {
            window.processInput();
            engine->Step(FIXED_TIMESTEP, SUBSTEPS);

            accumulator -= FIXED_TIMESTEP;
        }        
        window.preRender();

        glm::mat4 projection = glm::perspective(glm::radians(camera->getZoom()), ASPECT_RATIO, NEAR_PLANE, FAR_PLANE);
        glm::mat4 view = camera->getViewMatrix();

        engine->draw(ourShader, view, projection, camera->getPosition(), light);

        // Check and call events and swap the buffers
        window.render();
    }

    return 0;
}
