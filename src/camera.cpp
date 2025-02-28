#include "../include/camera.h"

#include <iostream>

Camera::Camera(glm::vec3 position,glm::vec3 lookAt, glm::vec3 up, float yaw, float pitch) : position(position), up(up), yaw(yaw), pitch(pitch), worldUp(up), zoom(45.0f) {
    direction = glm::normalize(position - lookAt);
    right = glm::normalize(glm::cross(direction, worldUp));
    updateCameraVectors();
    updateViewMatrix(); 
    matrixUpdateRequired = false;
}

void Camera::setPosition(const glm::vec3& newPos) {
    position = newPos;
    matrixUpdateRequired = true;
}

void Camera::lookAt(const glm::vec3& newDirection) {
    direction = glm::normalize(position - newDirection);
    matrixUpdateRequired = true;
}

glm::mat4 Camera::getViewMatrix() {
    if (matrixUpdateRequired) {
        updateCameraVectors();
        updateViewMatrix();
    }
    return viewMatrix;
}

void Camera::updateViewMatrix() {
    viewMatrix = glm::lookAt(position, position + direction, up);
}

void Camera::processKeyboard(CameraMovement directionToMove, float deltaTime) {
    float velocity = SPEED * deltaTime;
    if (directionToMove == FORWARD)
        position += direction * velocity;
    if (directionToMove == BACKWARD)
        position -= direction * velocity;
    if (directionToMove == LEFT)
        position -= right * velocity;
    if (directionToMove == RIGHT)
        position += right * velocity;

    matrixUpdateRequired = true;
}

void Camera::processMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch) {
    xoffset *= SENSITIVITY;
    yoffset *= SENSITIVITY;

    yaw += xoffset;
    pitch += yoffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (constrainPitch)
    {
        if (pitch > 89.0f)
            pitch = 89.0f;
        if (pitch < -89.0f)
            pitch = -89.0f;
    }
    matrixUpdateRequired = true;
}
void Camera::processMouseScroll(float yoffset) {
    zoom -= (float)yoffset;
    if (zoom < 1.0f)
        zoom = 1.0f;
    if (zoom > 45.0f)
        zoom = 45.0f;
}


void Camera::updateCameraVectors() {
    // calculate the new direction vector
    glm::vec3 newDirection;
    newDirection.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    newDirection.y = sin(glm::radians(pitch));
    newDirection.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    direction = glm::normalize(newDirection);
    // also re-calculate the Right and Up vector
    right = glm::normalize(glm::cross(direction, worldUp));  // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
    up = glm::normalize(glm::cross(right, direction));
}