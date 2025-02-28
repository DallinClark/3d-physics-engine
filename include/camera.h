#pragma once

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <glad/glad.h>

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

class Camera {
private:
    glm::vec3 position;// where the camera is
    glm::vec3 direction;  // way it is looking at
    glm::vec3 up;
    glm::vec3 right;
    glm::vec3 worldUp;
    glm::mat4 viewMatrix;

    // euler angles
    float yaw;
    float pitch;

    float movementSpeed;
    float mouseSensitivity;
    float zoom;

    bool matrixUpdateRequired;

    void updateViewMatrix();
    void updateCameraVectors();

public:
    // Defines several possible options for camera movement
    enum CameraMovement {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    };

    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),glm::vec3 lookAt = glm::vec3(0.0,0.0,-1.0), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH);

    glm::mat4 getViewMatrix() const { return viewMatrix; }

    // Set the position of the camera
    void setPosition(const glm::vec3& newPos);

    // Set the target the camera is looking at
    void lookAt(const glm::vec3& newTarget);

    // Getters for position and target
    glm::vec3 getPosition() const { return position; }
    glm::vec3 getDirection() const { return direction; }
    glm::mat4 getViewMatrix();
    float getZoom() const { return zoom; }

    void processMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true);
    void processMouseScroll(float yoffset);
    void processKeyboard(CameraMovement directionToMove, float deltaTime);
};
