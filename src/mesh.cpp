#include "../include/mesh.h"

Mesh::Mesh(vector<Vertex> vertices, vector<unsigned int> indices, ShapeType type) {
    this->vertices = vertices;
    this->indices = indices;
    this->shapeType = type;

    setupMesh();
}

void Mesh::setupMesh() {
    // Generate and bind VAO, VBO, and EBO
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    // Load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Set the vertex attribute pointers
    // Vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Position));

    // Vertex Colors
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));

    glBindVertexArray(0); // Unbind VAO after setting up attributes
}


void Mesh::draw(Shader& shader, glm::mat4 world, glm::mat4 view, glm::mat4 proj) {
    shader.use();
    glBindVertexArray(VAO);
    shader.setMatrix4fv("world", world);
    shader.setMatrix4fv("view", view);
    shader.setMatrix4fv("proj", proj);

    //shader.set3f("myColor", color.x, color.y, color.z);
    glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);

    glBindVertexArray(0);
}

std::vector<glm::vec4> Mesh::getVertexPositions() {
    std::vector<glm::vec4> positions;
    positions.reserve(verticesNoDuplicates.size()); 

    for (const auto& vertex : verticesNoDuplicates) {
        positions.emplace_back(vertex); 
    }

    return positions;
}



