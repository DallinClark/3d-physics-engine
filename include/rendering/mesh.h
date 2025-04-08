#pragma once
#include "../glm/glm.hpp"
#include "../glm/gtc/matrix_transform.hpp"

#include "point_light.h"
#include "camera.h"
#include "shader.h"

#include <string>
#include <vector>
using namespace std;

enum class ShapeType {
    Sphere,
    Cube,
    Tetrahedron,
    Diamond
};

struct Vertex {
    glm::vec3 Position;
    //glm::vec3 Color;
    glm::vec2 TexCoords;
    glm::vec3 Normal;
};

struct Face {
    std::vector<glm::vec3> vertices;
    std::vector<int> indices;
    glm::vec3 normal;
};

class Mesh {
public:
    // mesh Data
    vector<Vertex>       vertices;
    vector<glm::vec4>    verticesNoDuplicates;
    vector<unsigned int> indices;
    vector<int> edges;
    vector<Face> faces;
    unsigned int VAO;
    ShapeType shapeType;

    // constructor
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices,vector<glm::vec4> verticesNoDuplicates, ShapeType type);

    // render the mesh
    void draw(Shader& shader, glm::mat4 world, glm::mat4 view, glm::mat4 proj, glm::vec3 cameraPosition, PointLight light);

    // getters
    vector<Vertex> getVertices() const { return vertices; }
    std::vector<glm::vec4> getVertexPositions();

    void setFaces(vector<Face> newFaces) { faces = newFaces; }
    void setEdges(vector<int> newEdges) { edges = newEdges; }

private:
    // render data 
    unsigned int VBO, EBO;

    // initializes all the buffer objects/arrays
    void setupMesh();
};
