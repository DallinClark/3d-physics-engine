#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <random>
#include <string>
#include <memory>

#include "../include/rigid_body.h"
#include "../include/world.h"
#include "../include/glm/glm.hpp"
#include "../include/glm/gtc/matrix_transform.hpp"
#include "../include/glm/gtc/type_ptr.hpp"

#include "../include/shader.h"
#include "../include/texture.h"
#include "../include/collisions.h"
#include "../include/world.h"
#include "../include/aabb.h"
#include "../include/camera.h"

#define _USE_MATH_DEFINES

const float FIXED_TIMESTEP = 1.0f / 30.0f;
const float FOV = 45.0f;
const float RES_WIDTH = 1280;
const float RES_HEIGHT = 720;
const float ASPECT_RATIO = RES_WIDTH / RES_HEIGHT;
const float NEAR_PLANE = 0.1f;
const float FAR_PLANE = 100.0f;
const float SCALE_FACTOR = 1.6f; // THIS IS THE AMOUNT OF PIXELS PER CENTIMETER
const int SUBSTEPS = 20;
float lastX = RES_WIDTH / 2, lastY = RES_HEIGHT / 2;
bool firstMouse = true;


// Resizes OpenGL viewport when window is resized
static void framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

static void mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    Camera* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    if (!camera) return; // Safety check

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xOffset = xpos - lastX;
    float yOffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;

    camera->processMouseMovement(xOffset, yOffset);
}

static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    Camera* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    camera->processMouseScroll(yoffset);
}

// Checks for keyboard presses
static void processInput(GLFWwindow* window, std::shared_ptr<Camera> camera) {

    // Closes Window on ESC
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        camera->processKeyboard(Camera::FORWARD, FIXED_TIMESTEP);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        camera->processKeyboard(Camera::BACKWARD, FIXED_TIMESTEP);
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        camera->processKeyboard(Camera::LEFT, FIXED_TIMESTEP);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        camera->processKeyboard(Camera::RIGHT, FIXED_TIMESTEP);
    }   
}

int main()
{
    // Initialize GLFW
    if (!glfwInit()) {
        std::cout << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Specify OpenGL version and profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create a GLFW window
    GLFWwindow* window = glfwCreateWindow(static_cast<int>(RES_WIDTH), static_cast<int>(RES_HEIGHT), "Rigid Body Physics", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Set OpenGL viewport
    glViewport(0, 0, RES_WIDTH, RES_HEIGHT);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    glm::vec3 cameraPos(0.0f, 0.0f, 20.0f);
    glm::vec3 cameraLookAt(0.0f, 0.0f, 0.0f);
    glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);

    std::shared_ptr<Camera> camera = std::make_shared<Camera>(cameraPos, cameraLookAt, cameraUp);
    // Store camera in window user pointer
    glfwSetWindowUserPointer(window, camera.get());

    // Set mouse callback
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetScrollCallback(window, scroll_callback);

    glm::mat4 projection = glm::perspective(glm::radians(camera->getZoom()), ASPECT_RATIO, NEAR_PLANE, FAR_PLANE);

    World engine = World();

    // Creates shader program
    Shader ourShader("resources/shaders/tex_vertex.vs", "resources/shaders/tex_fragment.fs");
    ourShader.use();

    // Creates Texture
    Texture crateTexture("resources/textures/container.jpg", 0);
    crateTexture.use();

    double currentTime = glfwGetTime();
    double accumulator = 0.0;
    int ticks = 0;

    std::shared_ptr<RigidBody> body;
    std::string errorMessage = "";
    bool success = RigidBody::CreateSquareBody(100.0f, 10.0f, 100.0f, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f, true, 0.5f, body, errorMessage, engine.getSquareMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine.AddBody(body);

    /*success = RigidBody::CreateSquareBody(1.0f, 1.0f, 1.0f, glm::vec3(2.0f, 2.0f, 10.0f), 1.0f, false, 0.5f, body, errorMessage, engine.getSquareMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine.AddBody(body);*/

    /*success = RigidBody::CreateSquareBody(1.0f, 1.0f, 1.0f, glm::vec3(0.1f, 1.5f, 10.0f), 1.0f, false, 0.5f, body, errorMessage, engine.getSquareMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine.AddBody(body);*/

    /*success = RigidBody::CreateCircleBody(0.5f, glm::vec3(0.1f, 1.5f, 10.0f), 1.0f, false, 0.5f, body, errorMessage, engine.getSphereMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine.AddBody(body);

    success = RigidBody::CreateSquareBody(1.0f,1.0f,1.0f, glm::vec3(0.1f, 1.5f, 10.0f), 1.0f, false, 0.5f, body, errorMessage, engine.getSquareMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine.AddBody(body);*/
    success = RigidBody::CreateSquareBody(1.0f, 1.0f, 1.0f, glm::vec3(0.2f, 1.5f, 10.0f), 1.0f, false, 0.5f, body, errorMessage, engine.getSquareMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine.AddBody(body);

    success = RigidBody::CreateSquareBody(1.0f,1.0f,1.0f, glm::vec3(-5.0f, 2.0f, 10.0f), 1.0f, false, 0.5f, body, errorMessage, engine.getSquareMesh());
    if (!success) {
        std::cerr << "Failed to create RigidBody: " << errorMessage << std::endl;
    }
    engine.AddBody(body);
    body->rotate(45.0f, glm::vec3(1.0, 1.0, 1.0));

    glEnable(GL_DEPTH_TEST);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    while (!glfwWindowShouldClose(window)) {

        double newTime = glfwGetTime();
        double frameTime = newTime - currentTime;
        currentTime = newTime;
        accumulator += frameTime;

        while (accumulator >= FIXED_TIMESTEP) {
            processInput(window, camera);

            body->Move(glm::vec3(0.1,0.0,0.0));
            //body->rotate(1.0f, glm::vec3(1.0f, 0.5f, 0.0f));

            ++ticks;
            engine.Step(FIXED_TIMESTEP, SUBSTEPS);

            accumulator -= FIXED_TIMESTEP;
        }        
        // Rendering Commands 
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection = glm::perspective(glm::radians(camera->getZoom()), ASPECT_RATIO, NEAR_PLANE, FAR_PLANE);
        glm::mat4 view = camera->getViewMatrix();

        engine.draw(ourShader, view, projection);

        // Check and call events and swap the buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup and exit
    glfwTerminate();
    return 0;
}
