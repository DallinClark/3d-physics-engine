#include "../../include/setup/mesh_creator.h"

const int MeshCreator::SPHERE_STACK_COUNT = 30;
const int MeshCreator::SPHERE_SECTOR_COUNT = 30;

void MeshCreator::createMeshes(std::shared_ptr<Mesh>& cubeMesh, std::shared_ptr<Mesh>& sphereMesh, std::shared_ptr<Mesh>& tetraMesh, std::shared_ptr<Mesh>& diamondMesh) {
    std::vector<Vertex> vertices = {
		// Front face
		{glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)}, // 0
		{glm::vec3( 0.5f, -0.5f,  0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)}, // 1
		{glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec2(0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f)}, // 2
		{glm::vec3( 0.5f,  0.5f,  0.5f), glm::vec2(1.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f)}, // 3
	
		// Back face
		{glm::vec3( 0.5f, -0.5f, -0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)}, // 4
		{glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)}, // 5
		{glm::vec3( 0.5f,  0.5f, -0.5f), glm::vec2(0.0f, 1.0f), glm::vec3(0.0f, 0.0f, -1.0f)}, // 6
		{glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec2(1.0f, 1.0f), glm::vec3(0.0f, 0.0f, -1.0f)}, // 7
	
		// Left face
		{glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f)}, // 8
		{glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f)}, // 9
		{glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec2(0.0f, 1.0f), glm::vec3(-1.0f, 0.0f, 0.0f)}, // 10
		{glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec2(1.0f, 1.0f), glm::vec3(-1.0f, 0.0f, 0.0f)}, // 11
	
		// Right face
		{glm::vec3( 0.5f, -0.5f,  0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)}, // 12
		{glm::vec3( 0.5f, -0.5f, -0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)}, // 13
		{glm::vec3( 0.5f,  0.5f,  0.5f), glm::vec2(0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f)}, // 14
		{glm::vec3( 0.5f,  0.5f, -0.5f), glm::vec2(1.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f)}, // 15
	
		// Bottom face
		{glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)}, // 16
		{glm::vec3( 0.5f, -0.5f, -0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)}, // 17
		{glm::vec3(-0.5f, -0.5f,  0.5f), glm::vec2(0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)}, // 18
		{glm::vec3( 0.5f, -0.5f,  0.5f), glm::vec2(1.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)}, // 19
	
		// Top face
		{glm::vec3(-0.5f,  0.5f,  0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)}, // 20
		{glm::vec3( 0.5f,  0.5f,  0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)}, // 21
		{glm::vec3(-0.5f,  0.5f, -0.5f), glm::vec2(0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f)}, // 22
		{glm::vec3( 0.5f,  0.5f, -0.5f), glm::vec2(1.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f)}  // 23
	};
	
	std::vector<unsigned int> indices = {
		// Front face
		0, 1, 2,  1, 2, 3,
		// Back face
		4, 5, 6,  5, 6, 7,
		// Left face
		8, 9, 10,  9, 10, 11,
		// Right face
		12, 13, 14,  13, 14, 15,
		// Bottom face
		16, 17, 18,  17, 18, 19,
		// Top face
		20, 21, 22,  21, 22, 23
	};

	std::vector<glm::vec4> verticesNoDuplicates = {
		// Front face
		{glm::vec4(-0.5f, -0.5f,  0.5f, 1.0f)}, // 0
		{glm::vec4(0.5f, -0.5f,  0.5f, 1.0f)}, // 1
		{glm::vec4(-0.5f,  0.5f,  0.5f, 1.0f)}, // 2
		{glm::vec4(0.5f,  0.5f,  0.5f, 1.0f)}, // 3

		// Back face
		{glm::vec4(0.5f, -0.5f, -0.5f, 1.0f)}, // 4
		{glm::vec4(-0.5f, -0.5f, -0.5f, 1.0f)}, // 5
		{glm::vec4(0.5f,  0.5f, -0.5f, 1.0f)}, // 6
		{glm::vec4(-0.5f,  0.5f, -0.5f, 1.0f)} // 7
	};

	// used to make face structs, counter-clockwise
	std::vector<int> faces = {
	1, 3, 2, 0,  // Front 
	5, 7, 6, 4,  // Back 
	0, 2, 7, 5,  // Left 
	1, 4, 6, 3,  // Right 
	2, 3, 6, 7,  // Top 
	0, 5, 4, 1   // Bottom 
	};

	vector<Face> faceStructs;
	for (size_t i = 0; i < faces.size(); i += 4) {
		Face face;

		// Loop over each of the 4 indices.
		for (int j = 0; j < 4; ++j) {
			int index = faces[i + j];
			face.indices.push_back(index);
			// Convert the glm::vec4 to glm::vec3 (ignoring the w component).
			glm::vec4 v4 = verticesNoDuplicates[index];
			glm::vec3 v3(v4.x, v4.y, v4.z);
			face.vertices.push_back(v3);
		}
		// Compute the face normal using the first three vertices.
		// (Assumes the vertices are ordered such that the computed normal points outward.)
		glm::vec3 edge1 = face.vertices[1] - face.vertices[0];
		glm::vec3 edge2 = face.vertices[3] - face.vertices[0];
		face.normal = glm::normalize(glm::cross(edge1, edge2));

		faceStructs.push_back(face);
	}

	std::vector<int> edges = {
		0, 1,  // Front bottom edge
		1, 3,  // Front right edge
		3, 2,  // Front top edge
		2, 0,  // Front left edge

		5, 4,  // Back bottom edge 
		4, 6,  // Back right edge
		6, 7,  // Back top edge
		7, 5,  // Back left edge

		0, 5,  // Left-bottom side edge
		1, 4,  // Right-bottom side edge
		2, 7,  // Left-top side edge
		3, 6   // Right-top side edge
	};


	std::shared_ptr<Mesh> newMesh = std::make_shared<Mesh>(vertices, indices, verticesNoDuplicates, ShapeType::Cube);
	newMesh->setFaces(faceStructs);
	newMesh->setEdges(edges);
	cubeMesh = newMesh;

	vertices.clear();
	indices.clear();

	// Circle Mesh
	float radius = 1.0f;
	float x, y, z, xy;                              // vertex position
	float u, v;                                     // vertex texCoord

	float sectorStep = 2 * M_PI / SPHERE_SECTOR_COUNT;
	float stackStep = M_PI / SPHERE_STACK_COUNT;
	float sectorAngle, stackAngle;

	for (int i = 0; i <= SPHERE_STACK_COUNT; ++i) {
		stackAngle = (M_PI / 2) - (i * stackStep);        // starting from pi/2 to -pi/2
		xy = radius * cosf(stackAngle);                    // r * cos(u)
		z = radius * sinf(stackAngle);                     // r * sin(u)

		// Add (sectorCount + 1) vertices per stack
		for (int j = 0; j <= SPHERE_SECTOR_COUNT; ++j) {
			sectorAngle = j * sectorStep;                  // starting from 0 to 2pi

			// Vertex position (x, y, z)
			x = xy * cosf(sectorAngle);                    // r * cos(u) * cos(v)
			y = xy * sinf(sectorAngle);                    // r * cos(u) * sin(v)

			// Vertex tex coord (u, v) range between [0, 1]
			u = (float)j / SPHERE_SECTOR_COUNT;
			v = (float)i / SPHERE_STACK_COUNT;

			glm::vec3 normal = glm::normalize(glm::vec3{x, y, z});
			// Push the vertex with position, tex coord, and normal
			vertices.push_back({ glm::vec3{x, y, z}, glm::vec2{u, v}, normal });
			
			// Optionally, push the vertex without duplicates (assuming this is part of the structure you're using)
			verticesNoDuplicates.push_back({ glm::vec4{x, y, z, 1}});
    }
}

	// generate index list of sphere triangles
	// k1--k1+1
	// |  / |
	// | /  |
	// k2--k2+1

	int k1, k2;
	for (int i = 0; i < SPHERE_STACK_COUNT; ++i) {
		k1 = i * (SPHERE_SECTOR_COUNT + 1);      // beginning of current stack
		k2 = k1 + SPHERE_SECTOR_COUNT + 1;         // beginning of next stack

		for (int j = 0; j < SPHERE_SECTOR_COUNT; ++j, ++k1, ++k2) {
			// First triangle of the quad (all stacks except the top pole)
			if (i != 0) {
				indices.push_back(k1);
				indices.push_back(k2);
				indices.push_back(k1 + 1);
			}
			// Second triangle of the quad (all stacks except the bottom pole)
			if (i != (SPHERE_STACK_COUNT - 1)) {
				indices.push_back(k1 + 1);
				indices.push_back(k2);
				indices.push_back(k2 + 1);
			}
		}
	}

	std::shared_ptr<Mesh> newSphereMesh = std::make_shared<Mesh>(vertices, indices,verticesNoDuplicates, ShapeType::Sphere);
    sphereMesh = newSphereMesh;

	vertices.clear();
	indices.clear();
	verticesNoDuplicates.clear();
	faceStructs.clear();
	edges.clear();

	vertices = {
		// Bottom face
		{glm::vec3( -0.5f,  -0.5f,  0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)}, // 0
		{glm::vec3(0.0f,  -0.5f,  -0.5f), glm::vec2(0.5f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)}, // 1
		{glm::vec3( 0.5f, -0.5f,  0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)}, // 2
	
		// Front face
		{glm::vec3( -0.5f,  -0.5f,  0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, 0.447f, 0.8944f)}, // 3
		{glm::vec3( 0.5f, -0.5f,  0.5f), glm::vec2(0.5f, 1.0f), glm::vec3(0.0f, 0.447f, 0.8944f)}, // 4
		{glm::vec3( 0.0f,  0.5f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec3(0.0f, 0.447f, 0.8944f)}, // 5

		// Left Face
		{glm::vec3( -0.5f,  -0.5f,  0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(-0.9045f, 0.3015f, -0.3015f)}, // 6
		{glm::vec3( 0.0f,  0.5f, 0.0f), glm::vec2(0.5f, 1.0f), glm::vec3(-0.9045f, 0.3015f, -0.3015f)}, // 7
		{glm::vec3( 0.0f, -0.5f,  -0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(-0.9045f, 0.3015f, -0.3015f)}, // 8
	
		// Right face
		{glm::vec3( 0.5f, -0.5f,  0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(0.9045f, 0.3015f, -0.3015f)}, // 9
		{glm::vec3( 0.0f, -0.5f,  -0.5f), glm::vec2(0.5f, 1.0f), glm::vec3(0.9045f, 0.3015f, -0.3015f)}, // 10
		{glm::vec3( 0.0f,  0.5f, 0.0f), glm::vec2(0.5f, 1.0f), glm::vec3(0.9045f, 0.3015f, -0.3015f)} // 11
	};
	
	indices = {
		// Bottom face
		0, 1, 2,
		// Front face
		3, 4, 5,
		// Left face
		6, 7, 8,
		// Right face
		9, 10, 11
	};

	verticesNoDuplicates = {
		// Bottom face
		{glm::vec4( -0.5f,  -0.5f,  0.5f,1.0f)}, // 0 Left Front
		{glm::vec4(0.0f,  -0.5f,  -0.5f, 1.0f)}, // 1 Back
		{glm::vec4( 0.5f, -0.5f,  0.5f, 1.0f)}, // 2 Right Front
		// Top Vertex
		{glm::vec4( 0.0f,  0.5f, 0.0f, 1.0f)} // 3 Top
	};

	// used to make face structs, counter-clockwise
	faces = {
	0, 1, 2,  // Bottom Face
	0, 2, 3, // Front Face
	0, 3, 2, // Left Face
	2, 1, 3 // Right Face
	};

	for (size_t i = 0; i < faces.size(); i += 3) {
		Face face;

		// Loop over each of the 3 indices.
		for (int j = 0; j < 3; ++j) {
			int index = faces[i + j];
			face.indices.push_back(index);
			// Convert the glm::vec4 to glm::vec3 (ignoring the w component).
			glm::vec4 v4 = verticesNoDuplicates[index];
			glm::vec3 v3(v4.x, v4.y, v4.z);
			face.vertices.push_back(v3);
		}
		// Compute the face normal using the first three vertices.
		// (Assumes the vertices are ordered such that the computed normal points outward.)
		glm::vec3 edge1 = face.vertices[1] - face.vertices[0];
		glm::vec3 edge2 = face.vertices[2] - face.vertices[0];
		face.normal = glm::normalize(glm::cross(edge1, edge2));

		faceStructs.push_back(face);
	}

	edges = {
		0,2, // Front Bottom
		0,1, // Left
		2,1, // Right
		0,3, // Left Top
		2,3, // Right Top
		1,3 // Back
	};

	std::shared_ptr<Mesh> newTetraMesh = std::make_shared<Mesh>(vertices, indices, verticesNoDuplicates, ShapeType::Tetrahedron);
	newTetraMesh->setFaces(faceStructs);
	newTetraMesh->setEdges(edges);
	tetraMesh = newTetraMesh;

	vertices.clear();
	indices.clear();
	verticesNoDuplicates.clear();
	faceStructs.clear();
	edges.clear();

	vertices = {
		// Bottom of diamond
		{glm::vec3( 0.0f,  -0.5f,  0.0f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, -.5812f, 0.8137f)}, // 0
		{glm::vec3(0.25f,  0.2f,  0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(0.0f, -.5812f, 0.8137f)}, // 1
		{glm::vec3( -0.25f, 0.2f,  0.5f), glm::vec2(0.5f, 1.0f), glm::vec3(0.0f, -.5812f, 0.8137f)}, // 2

		{glm::vec3( 0.0f,  -0.5f,  0.0f), glm::vec2(0.0f, 0.0f), glm::vec3(0.7537f, -.5383f, 0.3768f)}, // 3
		{glm::vec3(0.5f,  0.2f,  0.0), glm::vec2(1.0f, 0.0f), glm::vec3(0.7537f, -.5383f, 0.3768f)}, // 4
		{glm::vec3(0.25f,  0.2f,  0.5f), glm::vec2(0.5f, 1.0f), glm::vec3(0.7537f, -.5383f, 0.3768f)}, // 5

		{glm::vec3( 0.0f,  -0.5f,  0.0f), glm::vec2(0.0f, 0.0f), glm::vec3(0.7537f, -.5383f, -0.3768f)}, // 6
		{glm::vec3(0.25f,  0.2f,  -0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(0.7537f, -.5383f, -0.3768f)}, // 7
		{glm::vec3(0.5f,  0.2f,  0.0), glm::vec2(0.5f, 1.0f), glm::vec3(0.7537f, -.5383f, -0.3768f)}, // 8

		{glm::vec3( 0.0f,  -0.5f,  0.0f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, -.5812f, -0.8137f)}, // 9
		{glm::vec3( -0.25f, 0.2f,  -0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(0.0f, -.5812f, -0.8137f)}, // 10
		{glm::vec3(0.25f,  0.2f,  -0.5f), glm::vec2(0.5f, 1.0f), glm::vec3(0.0f, -.5812f, -0.8137f)}, // 11

		{glm::vec3( 0.0f,  -0.5f,  0.0f), glm::vec2(0.0f, 0.0f), glm::vec3(-0.7537f, -.5383f, -0.3768f)}, // 12
		{glm::vec3(-0.5f,  0.2f,  0.0), glm::vec2(1.0f, 0.0f), glm::vec3(-0.7537f, -.5383f, -0.3768f)}, // 13
		{glm::vec3(-0.25f,  0.2f,  -0.5f), glm::vec2(0.5f, 1.0f), glm::vec3(-0.7537f, -.5383f, -0.3768f)}, // 14

		{glm::vec3( 0.0f,  -0.5f,  0.0f), glm::vec2(0.0f, 0.0f), glm::vec3(-0.7537f, -.5383f, 0.3768f)}, // 15
		{glm::vec3(-0.25f,  0.2f,  0.5f), glm::vec2(1.0f, 0.0f), glm::vec3(-0.7537f, -.5383f, 0.3768f)}, // 16
		{glm::vec3(-0.5f,  0.2f,  0.0), glm::vec2(0.5f, 1.0f), glm::vec3(-0.7537f, -.5383f, 0.3768f)}, // 17

		// Middle Section
		{glm::vec3(-0.5f,  0.2f,  0.0), glm::vec2(0.0f, 0.0f), glm::vec3(-0.5963, .7454, 0.2981)}, // 18
		{glm::vec3(-0.25f,  0.2f,  0.5f), glm::vec2(0.25f, 0.0f), glm::vec3(-0.5963, .7454, 0.2981)}, // 19
		{glm::vec3(-0.125f,  0.4f,  0.25f), glm::vec2(0.0f, 0.25f), glm::vec3(-0.5963, .7454, 0.2981)}, // 20
		{glm::vec3(-0.25f,  0.4f,  0.0), glm::vec2(0.25f, 0.25f), glm::vec3(-0.5963, .7454, 0.2981)}, // 21

		{glm::vec3(-0.25f,  0.2f,  0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, .7809, 0.62495)}, // 22
		{glm::vec3(0.25f,  0.2f,  0.5f), glm::vec2(0.25f, 0.0f), glm::vec3(0.0f, .7809, 0.62495)}, // 23
		{glm::vec3(0.125f,  0.4f,  0.25f), glm::vec2(0.0f, 0.25f), glm::vec3(0.0f, .7809, 0.62495)}, // 24
		{glm::vec3(-0.125f,  0.4f,  0.25f), glm::vec2(0.25f, 0.25f), glm::vec3(0.0f, .7809, 0.62495)}, // 25

		{glm::vec3(0.25f,  0.2f,  0.5f), glm::vec2(0.0f, 0.0f), glm::vec3(0.5963, .7454, 0.2981)}, // 26
		{glm::vec3(0.5f,  0.2f,  0.0), glm::vec2(0.25f, 0.0f), glm::vec3(0.5963, .7454, 0.2981)}, // 27
		{glm::vec3(0.25f,  0.4f,  0.0),glm::vec2(0.0f, 0.25f), glm::vec3(0.5963, .7454, 0.2981)}, // 28
		{glm::vec3(0.125f,  0.4f,  0.25f), glm::vec2(0.25f, 0.25f), glm::vec3(0.5963, .7454, 0.2981)}, // 29

		{glm::vec3(-0.25f,  0.4f,  0.0), glm::vec2(0.0f, 0.0f), glm::vec3(-0.5963, .7454, -0.2981)}, // 30
		{glm::vec3(-0.125f,  0.4f,  -0.25f), glm::vec2(0.25f, 0.0f), glm::vec3(-0.5963, .7454, -0.2981)}, // 31
		{glm::vec3(-0.25f,  0.2f,  -0.5f), glm::vec2(0.0f, 0.25f), glm::vec3(-0.5963, .7454, -0.2981)}, // 32
		{glm::vec3(-0.5f,  0.2f,  0.0), glm::vec2(0.5f, 0.5f), glm::vec3(-0.5963, .7454, -0.2981)}, // 33
		
		{glm::vec3(-0.125f,  0.4f,  -0.25f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, .78086f, -0.6247f)}, // 34
		{glm::vec3(0.125f,  0.4f,  -0.25f), glm::vec2(0.25f, 0.0f), glm::vec3(0.0f, .78086f, -0.6247f)}, // 35
		{glm::vec3(0.25f,  0.2f,  -0.5f), glm::vec2(0.0f, 0.25f), glm::vec3(0.0f, .78086f, -0.6247f)}, // 36
		{glm::vec3(-0.25f,  0.2f,  -0.5f), glm::vec2(0.25f, 0.25f), glm::vec3(0.0f, .78086f, -0.6247f)}, // 37

		{glm::vec3(0.125f,  0.4f,  -0.25f), glm::vec2(0.0f, 0.0f), glm::vec3(0.5963, .7454, -0.2981)}, // 38
		{glm::vec3(0.25f,  0.4f,  0.0), glm::vec2(0.25f, 0.0f), glm::vec3(0.5963, .7454, -0.2981)}, // 39
		{glm::vec3(0.5f,  0.2f,  0.0), glm::vec2(0.0f, 0.25f), glm::vec3(0.5963, .7454, -0.2981)}, // 40
		{glm::vec3(0.25f,  0.2f,  -0.5f), glm::vec2(0.25f, 0.25f), glm::vec3(0.5963, .7454, -0.2981)}, // 41

		// Top Section
		{glm::vec3(-0.125f,  0.4f,  0.25f), glm::vec2(0.0f, 0.0f), glm::vec3(-0.3651,.91287,.18257)}, // 42
		{glm::vec3(0.0f,  0.5f,  0.0f), glm::vec2(0.2f, 0.0f), glm::vec3(-0.3651,.91287,.18257)}, // 43
		{glm::vec3(-0.25f,  0.4f,  0.0), glm::vec2(0.1f, 0.2f), glm::vec3(-0.3651,.91287,.18257)}, // 44

		{glm::vec3(0.125f,  0.4f,  0.25f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, .92847, 0.3714)}, // 45
		{glm::vec3(0.0f,  0.5f,  0.0f), glm::vec2(0.2f, 0.0f), glm::vec3(0.0f, .92847, 0.3714)}, // 46
		{glm::vec3(-0.125f,  0.4f,  0.25f), glm::vec2(0.1f, 0.2f), glm::vec3(0.0f, .92847, 0.3714)}, // 47

		{glm::vec3(0.25f,  0.4f,  0.0), glm::vec2(0.0f, 0.0f), glm::vec3(0.3651,.91287,.18257)}, // 48
		{glm::vec3(0.0f,  0.5f,  0.0f), glm::vec2(0.2f, 0.0f), glm::vec3(0.3651,.91287,.18257)}, // 49
		{glm::vec3(0.125f,  0.4f,  0.25f), glm::vec2(0.1f, 0.2f), glm::vec3(0.3651,.91287,.18257)}, // 50

		{glm::vec3(0.125f,  0.4f,  -0.25f), glm::vec2(0.0f, 0.0f), glm::vec3(0.3651,.91287,-.18257)}, // 51
		{glm::vec3(0.0f,  0.5f,  0.0f), glm::vec2(0.2f, 0.0f), glm::vec3(0.3651,.91287,-.18257)}, // 52
		{glm::vec3(0.25f,  0.4f,  0.0), glm::vec2(0.1f, 0.2f), glm::vec3(0.3651,.91287,-.18257)}, // 53

		{glm::vec3(-0.125f,  0.4f,  -0.25f), glm::vec2(0.0f, 0.0f), glm::vec3(0.0f, .92847, -0.3714)}, // 54
		{glm::vec3(0.0f,  0.5f,  0.0f), glm::vec2(0.2f, 0.0f), glm::vec3(0.0f, .92847, -0.3714)}, // 55
		{glm::vec3(0.125f,  0.4f,  -0.25f), glm::vec2(0.1f, 0.2f), glm::vec3(0.0f, .92847, -0.3714)}, // 56

		{glm::vec3(-0.25f,  0.4f,  0.0), glm::vec2(0.0f, 0.0f), glm::vec3(-0.3651,.91287,-.18257)}, // 57
		{glm::vec3(0.0f,  0.5f,  0.0f), glm::vec2(0.2f, 0.0f), glm::vec3(-0.3651,.91287,-.18257)}, // 58
		{glm::vec3(-0.125f,  0.4f,  -0.25f), glm::vec2(0.1f, 0.2f), glm::vec3(-0.3651,.91287,-.18257)}, // 59
	};
	
	indices = {
		// Bottom of diamond
		0, 1, 2,
		3, 4, 5,
		6, 7, 8,
		9, 10, 11,
		12, 13, 14,
		15, 16, 17,

		// Middle of diamond
		18, 19, 20,  18, 20, 21,
		22, 23, 24,  22, 24, 25,
		26, 27, 28,  26, 28, 29,
		30, 31, 32,  30, 32, 33,
		34, 35, 36,  34, 36, 37,
		38, 39, 40,  38, 40, 41,

		// Top of diamond
		42, 43, 44,
		45, 46, 47,
		48, 49, 50,
		51, 52, 53,
		54, 55, 56,
		57, 58, 59
	};

	verticesNoDuplicates = {
		{glm::vec4( 0.0f,  -0.5f,  0.0f, 1.0f)} , // 0 
		{glm::vec4( 0.25f,  0.2f,  0.5f, 1.0f)}, // 1
		{glm::vec4( -0.25f, 0.2f,  0.5f, 1.0f)}, // 2
		{glm::vec4( 0.5f,  0.2f,  0.0f, 1.0f)},  // 3
		{glm::vec4( 0.25f,  0.2f,  -0.5f, 1.0f)}, // 4
		{glm::vec4( -0.25f, 0.2f,  -0.5f, 1.0f)}, // 5
		{glm::vec4( -0.5f,  0.2f,  0.0f, 1.0f)},  // 6
		{glm::vec4( 0.125f,  0.4f,  0.25f, 1.0f)}, // 7
		{glm::vec4( -0.125f,  0.4f,  0.25f, 1.0f)}, // 8
		{glm::vec4( -0.125f,  0.4f,  -0.25f, 1.0f)}, // 9
		{glm::vec4( 0.125f,  0.4f,  -0.25f, 1.0f)}, // 10
		{glm::vec4( 0.0f,  0.5f,  0.0f, 1.0f)},   // 11
		{glm::vec4( -0.25f, 0.4f,  0.0f, 1.0f)},  // 12
		{glm::vec4( 0.25f,  0.4f,  0.0f, 1.0f)}  // 13
	};


	// used to make face structs, counter-clockwise
	vector<int> quadFaces = {
		6, 2, 8, 12,
		2, 1, 7, 8,
		1, 3, 13,7,
		12,9,5,6,
		9, 10,4,5,
		10,13,3,4
	};

	vector<int> triFaces = {
	0, 1, 2,  
	0,3,1,
	0,4, 3,
	0, 5,4,
	0,6,5,
	0,2,6,

	8,11,12,
	7,11, 8,
	13,11,7,
	10,11,13,
	9,11,10,
	12,11,9
	};

	edges = {
		0,1,
		1,2,
		2,0,
		0,3,
		3,1,
		0,4,
		4,3,
		0,5,
		5,4,
		0,6,
		6,5,
		2,6,
		8,11,
		11,12,
		12,8,
		7,11,
		11,8,
		8,7,
		13,11,
		7,13,
		10,11,
		13,10,
		9,11,
		10,9,
		9,12,
		2,8,
		12,6,
		1,7,
		8,2,
		3,13,
		9,5,
		10,4,
		13,3,
	};


	for (size_t i = 0; i < triFaces.size(); i += 3) {
		Face face;

		// Loop over each of the 3 indices.
		for (int j = 0; j < 3; ++j) {
			int index = triFaces[i + j];
			face.indices.push_back(index);
			// Convert the glm::vec4 to glm::vec3 (ignoring the w component).
			glm::vec4 v4 = verticesNoDuplicates[index];
			glm::vec3 v3(v4.x, v4.y, v4.z);
			face.vertices.push_back(v3);
		}
		// Compute the face normal using the first three vertices.
		// (Assumes the vertices are ordered such that the computed normal points outward.)
		glm::vec3 edge1 = face.vertices[1] - face.vertices[0];
		glm::vec3 edge2 = face.vertices[2] - face.vertices[0];
		face.normal = glm::normalize(glm::cross(edge1, edge2));

		faceStructs.push_back(face);
	}

	for (size_t i = 0; i < quadFaces.size(); i += 4) {
		Face face;

		// Loop over each of the 3 indices.
		for (int j = 0; j < 4; ++j) {
			int index = quadFaces[i + j];
			face.indices.push_back(index);
			// Convert the glm::vec4 to glm::vec3 (ignoring the w component).
			glm::vec4 v4 = verticesNoDuplicates[index];
			glm::vec3 v3(v4.x, v4.y, v4.z);
			face.vertices.push_back(v3);
		}
		// Compute the face normal using the first three vertices.
		// (Assumes the vertices are ordered such that the computed normal points outward.)
		glm::vec3 edge1 = face.vertices[1] - face.vertices[0];
		glm::vec3 edge2 = face.vertices[3] - face.vertices[0];
		face.normal = glm::normalize(glm::cross(edge1, edge2));

		faceStructs.push_back(face);
	}


	std::shared_ptr<Mesh> newDiamondMesh = std::make_shared<Mesh>(vertices, indices, verticesNoDuplicates, ShapeType::Diamond);
	newDiamondMesh->setFaces(faceStructs);
	newDiamondMesh->setEdges(edges);
	diamondMesh = newDiamondMesh;
}
