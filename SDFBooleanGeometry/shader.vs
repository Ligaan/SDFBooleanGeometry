#version 330 core
layout (location = 0) in vec3 aPos;   // Vertex position
layout (location = 1) in vec3 aNormal; // Vertex normal
layout (location = 2) in vec3 aColor;  // Vertex color

out vec3 ourColor; // Pass color to fragment shader
out vec3 Normal;   // Pass transformed normal
out vec3 FragPos;  // Pass world-space position

uniform mat4 model;      // Model-to-world transform
uniform mat4 view;       // World-to-view transform
uniform mat4 projection; // View-to-clip transform

void main()
{
    FragPos = vec3(model * vec4(aPos, 1.0));
    ourColor = aColor;
    Normal = mat3(transpose(inverse(model))) * aNormal;
    gl_Position = projection * view * vec4(FragPos, 1.0);
}