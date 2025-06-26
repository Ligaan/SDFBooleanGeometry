#version 330 core
out vec4 FragColor;

in vec3 ourColor;
in vec3 Normal;
in vec3 FragPos;

struct Light {
    vec3 direction;
    vec3 color;
    double ambientStrength;
    double diffuseStrength;
};

uniform vec3 viewPos;
uniform Light light;
uniform double Multi = 1.0;
uniform sampler3D sdfTexture1;
uniform sampler3D sdfTexture2;
uniform mat4 worldToLocalMatrix1;
uniform mat4 worldToLocalMatrix2;
uniform vec3 minBound1;
uniform vec3 maxBound1;
uniform vec3 minBound2;
uniform vec3 maxBound2;

double SDF(vec3 worldPos,mat4 worldToLocalMatrix,vec3 minBound,vec3 maxBound,sampler3D sdfTexture) {
    // Transform world-space position to local space
    vec4 localPosHomogeneous = worldToLocalMatrix * vec4(worldPos, 1.0);
    vec3 localPos = localPosHomogeneous.xyz / localPosHomogeneous.w;

    // Compute normalized texture coordinates [0,1]
    vec3 texCoord = (localPos - minBound) / (maxBound - minBound);

    // Check if within texture bounds
    if (all(greaterThanEqual(texCoord, vec3(0.0))) && all(lessThanEqual(texCoord, vec3(1.0)))) {
        return texture(sdfTexture, texCoord).r;
    } else {
        return 100.0; // out of bounds → black
    }
}

void main() {

    // Compute lighting
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(-light.direction);
    vec3 ambient = light.ambientStrength * light.color * ourColor;
    double diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuseStrength * light.color * diff * ourColor;

    vec3 result = ambient + diffuse;

    // Check if within SDF texture bounds

        const double EPSILON = 1e-3;
        double sdfValue1 = SDF(FragPos,worldToLocalMatrix1,minBound1,maxBound1,sdfTexture1);
        double sdfValue2 = SDF(FragPos,worldToLocalMatrix2,minBound2,maxBound2,sdfTexture2);

        bool a1=true;
        bool a2=true;

        if (sdfValue1 > EPSILON) {
            a1 = false;
        }

        if (sdfValue2 > EPSILON) {
            a2 = false;
        }

        if(a1 && a2){
            result = vec3(1.0, 0.0, 0.0);
        }else{
            result = vec3(0.0, 1.0, 0.0);
        }

        //if (sdfValue < -EPSILON) {
            // inside
            //result = vec3(1.0, 0.0, 0.0);
        //} else if (sdfValue > EPSILON) {
            // outside
          //  result = vec3(0.0, 1.0, 0.0);
        //} else {
            // near surface
          //  result = vec3(1.0, 0.0, 0.0);
       // }
        // Sample SDF

        // Override color based on inside/outside
        //if (sdfValue <= 0.0) {
          //  result = vec3(1.0, 0.0, 0.0); // Red for inside
        //} else {
           // result = vec3(0.0, 1.0, 0.0); // Green for outside
       // }
    //} else {
     //   result = vec3(0.0); // Out of bounds
   // }

    FragColor = vec4(result, 1.0) * Multi;
}
