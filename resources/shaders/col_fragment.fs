#version 440 core

in vec3 vertexColor;                      
out vec4 FragColor;                    

uniform vec3 myColor;                  

void main()
{
    if (vertexColor.x > 0.9 && vertexColor.y > .9 && vertexColor.z > .9) {  
        FragColor = vec4(1.0, 1.0, 1.0, 1.0); 
    }
    else {
        FragColor = vec4(myColor, 1.0); 
    }
}