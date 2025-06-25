#version 330 core
out vec4 FragColor;

in vec3 ourColor;
in vec3 Normal;
in vec3 FragPos;

struct Light {
    vec3 direction;
    vec3 color;
    float ambientStrength;
    float diffuseStrength;
};

uniform vec3 viewPos;
uniform Light light;
uniform float Multi = 1.0;
uniform sampler3D sdfTexture;
uniform mat4 worldToLocalMatrix;
uniform vec3 minBound;
uniform vec3 maxBound;

void main() {
    // Transform world-space position to local space
    vec4 localPosHomogeneous = worldToLocalMatrix * vec4(FragPos, 1.0);
    vec3 localPos = localPosHomogeneous.xyz / localPosHomogeneous.w;

    // Compute normalized texture coordinates
    vec3 texCoord = (localPos - minBound) / (maxBound - minBound);

    // Initialize result color
    vec3 result;

    // Compute lighting
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(-light.direction);
    vec3 ambient = light.ambientStrength * light.color * ourColor;
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuseStrength * light.color * diff * ourColor;
    result = (ambient + diffuse);

    // Check if within SDF grid bounds
    if (all(greaterThanEqual(texCoord, vec3(0.0))) && all(lessThanEqual(texCoord, vec3(1.0)))) {
        // Sample SDF value
        float sdfValue = texture(sdfTexture, texCoord).r;

        // Color based on SDF
        if (sdfValue <= 0.0) {
            result *= vec3(1.0, 0.0, 0.0); // Red for inside
        } else {
            result *= vec3(0.0, 1.0, 0.0); // Green for outside
        }
    } else {
        // Out of bounds: use base lighting
        result = vec3(0.0,0.0,0.0);
    }

    FragColor = vec4(result, 1.0) * Multi;
}