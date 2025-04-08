#pragma once

#include "../glm/glm.hpp"
#include "../glm/gtc/matrix_transform.hpp"
#include <glad/glad.h>


/* Class for a camera that looks onto the scene, creates a view matrix for object transformations */

const float SPEED = 5.0f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;
const float FOV = 45.0f;
const float NEAR_PLANE = 0.1f;
const float FAR_PLANE = 100.0f;

class Camera {
private:
    glm::vec3 position; // where the camera is
    glm::vec3 direction;  // way it is looking
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
    enum CameraMovement {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    };

    // constuctor that takes intial position, lookAt point, up point and rotations
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),glm::vec3 lookAt = glm::vec3(0.0,0.0,-1.0), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = 0.0f, float pitch = 0.0f);

    void setPosition(const glm::vec3& newPos);

    // Set the target the camera is looking at
    void lookAt(const glm::vec3& newLookAtPoint);

    glm::vec3 getPosition() const { return position; }
    glm::vec3 getDirection() const { return direction; }
    glm::mat4 getViewMatrix();
    float getZoom() const { return zoom; }

    // changes camera based on user input, Unreal-like controls
    void processMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true);
    void processMouseScroll(float yoffset);
    void processKeyboard(CameraMovement directionToMove, float deltaTime);
};
