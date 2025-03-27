#pragma once
#include "mesh.h"
#include <memory>

class MeshCreator {
public:
    // Static method to create meshes
    static void createMeshes(std::shared_ptr<Mesh>& cubeMesh, std::shared_ptr<Mesh>& sphereMesh, std::shared_ptr<Mesh>& tetraMesh);

    static const int SPHERE_STACK_COUNT;
	static const int SPHERE_SECTOR_COUNT;
};
