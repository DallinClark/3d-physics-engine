#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <random>
#include <string>
#include <memory>

#include "../physics/rigid_body.h"
#include "../physics/world.h"
#include "../glm/glm.hpp"
#include "../glm/gtc/matrix_transform.hpp"
#include "../glm/gtc/type_ptr.hpp"

#include "../rendering/shader.h"
#include "../setup/setup.h"
#include "../rendering/texture.h"
#include "../physics/collisions/collisions.h"
#include "../physics/world.h"
#include "../physics/collisions/aabb.h"
#include "../rendering/camera.h"
#include "../rendering/point_light.h"

const float RES_WIDTH = 3940;
const float RES_HEIGHT = 2160;
const float ASPECT_RATIO = RES_WIDTH / RES_HEIGHT;

class Window {
private:
    GLFWwindow* window;
    std::shared_ptr<Camera> camera;
    float lastX, lastY;
    bool firstMouse;

public:


    Window(std::shared_ptr<Camera>& camera);

    // Resizes OpenGL viewport when window is resized
    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);

    // Gets input for the camera
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    void processInput();

    float getTime() { return glfwGetTime(); }

    bool shouldClose() { return glfwWindowShouldClose(window); }

    void render();
    void preRender();

    ~Window() { glfwTerminate(); }
};