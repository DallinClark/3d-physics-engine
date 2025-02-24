#pragma once

#include <glad/glad.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include "../include/glm/gtc/type_ptr.hpp"

class Shader {
public:
	// Program ID
	unsigned int ID;

	// Constructer and deconstructer
	Shader(const char* vertexPath, const char* fragmentPath);
	~Shader();
	// uses and activates the shader
	void use();
	// utility uniform functions
	void setBool(const std::string& name, bool value) const;
	void setInt(const std::string& name, int value) const;
	void setFloat(const std::string& name, float value) const;
	void setMatrix4fv(const std::string& name, const glm::mat4& values) const;
	void set3f(const std::string& name, float value1, float value2, float value3) const;
};


