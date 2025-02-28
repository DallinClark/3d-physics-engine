#pragma once
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "shader.h"

#include <string>
#include <vector>
using namespace std;

enum class ShapeType {
    Sphere,
    Cube
};

struct Vertex {
    glm::vec3 Position;
    //glm::vec3 Color;
    glm::vec2 TexCoords;
};


class Mesh {
public:
    // mesh Data
    vector<Vertex>       vertices;
    vector<glm::vec4>    verticesNoDuplicates;
    vector<unsigned int> indices;
    unsigned int VAO;
    ShapeType shapeType;

    // constructor
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices, ShapeType type);

    // render the mesh
    void draw(Shader& shader, glm::mat4 world, glm::mat4 view, glm::mat4 proj);

    // getters
    vector<Vertex> getVertices() const { return vertices; }
    std::vector<glm::vec4> getVertexPositions();

    void setVerticesNoDuplicates(vector<glm::vec4> newVertices) { verticesNoDuplicates = newVertices; }

private:
    // render data 
    unsigned int VBO, EBO;

    // initializes all the buffer objects/arrays
    void setupMesh();
};
