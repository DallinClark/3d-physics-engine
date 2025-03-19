#pragma once
#include <iostream>
#include <glad/glad.h>
#include "stb_image.h"

class Texture {
public:
	// texture ID
	unsigned int ID;
	int textureUnit; // which unit to bind to 

	// Constructer and deconstructer
	Texture(const char* texturePath, int unit);
	~Texture();

	void use();

};
