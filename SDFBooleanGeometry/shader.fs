#version 330 core
out vec4 FragColor;  

in vec3 ourColor;
in vec3 Normal;
in vec3 FragPos; 


struct Light {
    vec3 direction;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

uniform vec3 viewPos;
uniform Light light;
uniform float Multi = 1.0f;
  
void main()
{
   // Ambient
    vec3 ambient = light.ambient * ourColor;

    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(-light.direction);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * ourColor;

    vec3 result = ambient + diffuse;
    FragColor = vec4(result, 1.0) * Multi;
}