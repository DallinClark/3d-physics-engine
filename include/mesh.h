#pragma once
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "shader.h"

#include <string>
#include <vector>
using namespace std;

// FOR 3d LOOK AT OPENGL CLASS, THIS IS ADAPTED FOR 2D AND NOT USING TEXTURES

enum class ShapeType {
    Circle,
    Square
};

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Color;
};


class Mesh {
public:
    // mesh Data
    vector<Vertex>       vertices;
    vector<unsigned int> indices;
    unsigned int VAO;
    ShapeType shapeType;

    // constructor
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices, ShapeType type);

    // render the mesh
    void Draw(Shader& shader, glm::mat4 trans, glm::vec3 color);

    vector<Vertex> GetVertices() { return vertices; }
    std::vector<glm::vec4> getVertexPositions();

private:
    // render data 
    unsigned int VBO, EBO;

    // initializes all the buffer objects/arrays
    void setupMesh();
};
