#version 440 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;
layout (location = 2) in vec3 aNormal;

out vec2 TexCoord;
out vec3 FragPosition;
out vec3 Normal;

uniform mat4 view;
uniform mat4 world;
uniform mat4 proj;

void main()
{
   FragPosition = vec3(world * vec4(aPos, 1.0));
   TexCoord = aTexCoord;
   Normal = mat3(transpose(inverse(world))) * aNormal;

   gl_Position = proj * view * world * vec4(aPos, 1.0);
}