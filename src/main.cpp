#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <random>
#include <string>
#include <memory>

#include "../include/rigid_body_2D.h"
#include "../include/engine_2D.h"
#include "../include/glm/glm.hpp"
#include "../include/glm/gtc/matrix_transform.hpp"
#include "../include/glm/gtc/type_ptr.hpp"

#include "../include/shader.h"
#include "../include/texture.h"
#include "../include/collisions.h"
#include "../include/engine_2D.h"
#include "../include/aabb.h"

#define _USE_MATH_DEFINES

const float FIXED_TIMESTEP = 1.0f / 30.0f;
const float RES_WIDTH = 1280;
const float RES_HEIGHT = 720;
const float SCALE_FACTOR = 1.6f; // THIS IS THE AMOUNT OF PIXELS PER CENTIMETER
const int SUBSTEPS = 20;
int lastMouseState = GLFW_RELEASE;
bool prevFrameC = false;
bool prevFrameS = false;


// Resizes OpenGL viewport when window is resized
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void CullScreen(Engine2D& engine) {
    float lowX = -RES_WIDTH / (2 * SCALE_FACTOR);
    float highX = RES_WIDTH / (2 * SCALE_FACTOR);
    float lowY = -RES_HEIGHT / (2 * SCALE_FACTOR);
    float highY = RES_HEIGHT / (2 * SCALE_FACTOR);

    std::vector<int> removeQueue;  

    for (int i = 0; i < engine.GetBodyCount(); ++i) {
        std::shared_ptr<RigidBody2D> body;
        engine.GetBody(i, body);
        AABB box = body->getAABB();

        if (box.max.x < lowX || box.min.x > highX || box.max.y < lowY || box.min.y > highY) {
            removeQueue.push_back(i);
            std::cout << body->getPosition().x << ", " << body->getPosition().y << endl;
        }

    }
    for (int i = removeQueue.size() - 1; i >= 0; --i) {
        engine.RemoveBody(removeQueue[i]);
    }
}

// Checks for keyboard presses
void processInput(GLFWwindow* window, Engine2D& engine) {

    // Closes Window on ESC
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        if (!prevFrameC) {

            int width, height;
            glfwGetWindowSize(window, &width, &height);

            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            std::string errorMessage = "";

            float normalizedX = (2.0f * xpos) / width - 1.0f;
            float normalizedY = 1.0f - (2.0f * ypos) / height;

            glm::vec4 screenCoord(normalizedX, normalizedY, 0.0f, 1.0f);

            glm::mat4 projection = glm::ortho(-RES_WIDTH / (2.0f * SCALE_FACTOR), RES_WIDTH / (2.0f * SCALE_FACTOR),
                -RES_HEIGHT / (2.0f * SCALE_FACTOR), RES_HEIGHT / (2.0f * SCALE_FACTOR),
                -1.0f, 1.0f);
            glm::mat4 invProj = glm::inverse(projection);

            glm::vec4 worldCoord = invProj * screenCoord;

            worldCoord /= worldCoord.w;

            std::cout << "coordinates are" << worldCoord.x << ", " << worldCoord.y << "\n";

            std::shared_ptr<RigidBody2D> body;

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<float> dist(10.0f, 30.0f);

            bool success = RigidBody2D::CreateSquareBody(dist(gen), dist(gen), glm::vec2(worldCoord.x, worldCoord.y), 0.5f, false, 0.5f, body, errorMessage, engine.getSquareMesh());
            if (!success) {
                std::cerr << "Failed to create RigidBody2D: " << errorMessage << std::endl;
            }

            engine.AddBody(body);
            std::cout << engine.GetBodyCount() << endl;

            prevFrameC = true;
        }
    }
    else {
        prevFrameC = false;
    }

    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        if (!prevFrameS) {
            int width, height;
            glfwGetWindowSize(window, &width, &height);

            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);

            float normalizedX = (2.0f * xpos) / width - 1.0f;
            float normalizedY = 1.0f - (2.0f * ypos) / height;

            glm::vec4 screenCoord(normalizedX, normalizedY, 0.0f, 1.0f);

            glm::mat4 projection = glm::ortho(-RES_WIDTH / (2.0f * SCALE_FACTOR), RES_WIDTH / (2.0f * SCALE_FACTOR),
                -RES_HEIGHT / (2.0f * SCALE_FACTOR), RES_HEIGHT / (2.0f * SCALE_FACTOR),
                -1.0f, 1.0f);
            glm::mat4 invProj = glm::inverse(projection);

            glm::vec4 worldCoord = invProj * screenCoord;

            worldCoord /= worldCoord.w;

            std::cout << "coordinates are" << worldCoord.x << ", " << worldCoord.y << "\n";

            std::shared_ptr<RigidBody2D> body;
            std::string errorMessage = "";


            bool success = RigidBody2D::CreateCircleBody(12.0f, glm::vec2(worldCoord.x, worldCoord.y), 0.5f, false, 0.5f, body, errorMessage, engine.getCircleMesh());
            if (!success) {
                std::cerr << "Failed to create RigidBody2D: " << errorMessage << std::endl;
            }

            engine.AddBody(body);
            std::cout << engine.GetBodyCount() << endl;
            prevFrameS = true;
        }
        
    }
    else {
        prevFrameS = false;
    }
    /*int mouseState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);

    if (mouseState == GLFW_PRESS && lastMouseState == GLFW_RELEASE) {
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);

        float normalizedX = (2.0f * xpos) / width - 1.0f;
        float normalizedY = 1.0f - (2.0f * ypos) / height;

        glm::vec4 screenCoord(normalizedX, normalizedY, 0.0f, 1.0f);

        glm::mat4 projection = glm::ortho(-RES_WIDTH / (2.0f * SCALE_FACTOR), RES_WIDTH / (2.0f * SCALE_FACTOR),
            -RES_HEIGHT / (2.0f * SCALE_FACTOR), RES_HEIGHT / (2.0f * SCALE_FACTOR),
            -1.0f, 1.0f);
        glm::mat4 invProj = glm::inverse(projection);

        glm::vec4 worldCoord = invProj * screenCoord;

        worldCoord /= worldCoord.w;

        std::cout << "coordinates are" << worldCoord.x << ", " << worldCoord.y << "\n";

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distr(0, 1);

        std::shared_ptr<RigidBody2D> body;
        std::string errorMessage = "";
        if (distr(gen) == 0) {
            bool success = RigidBody2D::CreateSquareBody(20.0f, 20.0f, glm::vec2(worldCoord.x, worldCoord.y), 1.0f, false, 0.5f, body, errorMessage);
            if (!success) {
                std::cerr << "Failed to create RigidBody2D: " << errorMessage << std::endl;
            }
        }
        else {
            bool success = RigidBody2D::CreateCircleBody(12.0f, glm::vec2(worldCoord.x, worldCoord.y), 0.7f, false, 0.5f, body, errorMessage);
            if (!success) {
                std::cerr << "Failed to create RigidBody2D: " << errorMessage << std::endl;
            }
        }
        engine.AddBody(body);
        std::cout << engine.GetBodyCount() <<endl;

    }

    lastMouseState = mouseState;*/

}

void processDirectionInput(GLFWwindow* window, RigidBody2D& body, int ticks) {
    float dx = 0.0f;
    float dy = 0.0f;
    float speed = 20.0f;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        dy++;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        dx--;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        dy--;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        dx++;
    }

    if (dx != 0.0f || dy != 0.0f) {
        glm::vec2 force_direction = glm::normalize(glm::vec2(dx, dy));
        glm::vec2 force = force_direction * speed;
        body.AddForce(force);
    }
}

int main()
{
    // Initialize GLFW
    if (!glfwInit())
    {
        std::cout << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Specify OpenGL version and profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create a GLFW window
    GLFWwindow* window = glfwCreateWindow(static_cast<int>(RES_WIDTH), static_cast<int>(RES_HEIGHT), "Physics2D", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Visable Coordinates in system are x = [-400, 400] and y = [-225, 225] and is in centimeters

    // Set OpenGL viewport
    glViewport(0, 0, RES_WIDTH, RES_HEIGHT);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    
    Engine2D engine = Engine2D();

    int bodyCount = 0;

    std::shared_ptr<RigidBody2D> body;
    std::string errorMessage = "";
    bool success = RigidBody2D::CreateSquareBody(RES_WIDTH / SCALE_FACTOR - 20.0f, 10.0f, glm::vec2(0, -(RES_HEIGHT / SCALE_FACTOR / 2.0f) + 20.0f ), 1.0f, true, 0.5f, body, errorMessage, engine.getSquareMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody2D: " << errorMessage << std::endl;
    }
    engine.AddBody(body);
    success = RigidBody2D::CreateSquareBody(RES_WIDTH / (SCALE_FACTOR * 3), 10.0f, glm::vec2((RES_WIDTH / (SCALE_FACTOR * 2)) - 200.0f, 0), 1.0f, true, 0.5f, body, errorMessage, engine.getSquareMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody2D: " << errorMessage << std::endl;
    }
    body->Rotate(M_PI/6);
    engine.AddBody(body);

    success = RigidBody2D::CreateSquareBody(RES_WIDTH / (SCALE_FACTOR * 3), 10.0f, glm::vec2(-(RES_WIDTH / (SCALE_FACTOR * 2)) + 200.0f, 0), 1.0f, true, 0.5f, body, errorMessage, engine.getSquareMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody2D: " << errorMessage << std::endl;
    }
    body->Rotate(-(M_PI / 6));
    engine.AddBody(body);



    // creates shader program
    Shader ourShader("resources/shaders/col_vertex.vs", "resources/shaders/col_fragment.fs");
    ourShader.use();

    glm::mat4 projection = glm::ortho( - RES_WIDTH / (2.0f * SCALE_FACTOR), RES_WIDTH / (2.0f * SCALE_FACTOR),
        -RES_HEIGHT / (2.0f * SCALE_FACTOR), RES_HEIGHT / (2.0f * SCALE_FACTOR),
        -1.0f, 1.0f);

    double currentTime = glfwGetTime();
    double accumulator = 0.0;
    int ticks = 0;


    while (!glfwWindowShouldClose(window)) {
        double newTime = glfwGetTime();
        double frameTime = newTime - currentTime;
        currentTime = newTime;
        accumulator += frameTime;

        while (accumulator >= FIXED_TIMESTEP) {
            ++ticks;
            engine.Step(FIXED_TIMESTEP, SUBSTEPS);
            CullScreen(engine);
            // Fixed time step logic here (e.g., update physics)
            accumulator -= FIXED_TIMESTEP;
        }
        // Input
        processInput(window, engine);
        
        // Rendering Commands 
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        engine.Draw(ourShader, projection);

        // Check and call events and swap the buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup and exit
    glfwTerminate();
    return 0;
}
