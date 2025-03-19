#version 440 core

in vec2 TexCoord;

out vec4 FragColor;

uniform sampler2D texture1;

uniform vec3 viewPos;  // Position of the camera

void main() {
    FragColor = texture(texture1, TexCoord);
}
