#version 440 core

in vec2 TexCoord;
in vec3 Normal;
in vec3 FragPosition;

out vec4 FragColor;

uniform sampler2D texture1;

uniform vec3 lightPosition; // Position of the light
uniform vec3 lightColor;    // Color of the light
uniform vec3 viewPosition;  // Position of the camera

void main() {
    vec4 textureColor = texture(texture1, TexCoord);
    vec3 normal = normalize(Normal);

    vec3 lightDir = normalize(lightPosition - FragPosition);
    vec3 viewDir = normalize(viewPosition - FragPosition);

    vec3 reflectDir = reflect(-lightDir, normal);

    vec3 ambient = 0.1 * lightColor;

    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = 0.8 * diff * lightColor;

    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 16);
    vec3 specular = 0.3 * spec * lightColor;

    vec3 result = (ambient + diffuse + specular) * textureColor.rgb;

    FragColor = vec4(result, textureColor.a);
}
