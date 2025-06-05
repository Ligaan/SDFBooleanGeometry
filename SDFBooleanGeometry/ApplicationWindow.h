#pragma once
#include "glad/glad.h"
#include "glfw3.h"
#include "Shader.h"
#include "Camera.h"
#include "Shapes.h"

struct Light {
    glm::vec3 position;

    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;

    float constant;
    float linear;
    float quadratic;
};

class ApplicationWindow
{
public:
	void Initialize();
	void Update();
	void Render();
    void Shutdown();

    const unsigned int SCR_WIDTH = 1920;
    const unsigned int SCR_HEIGHT = 1080;

    GLFWwindow* window;

    // camera
    static Camera camera;
    static float lastX;
    static float lastY;
    static bool firstMouse;
    static bool buttonPressed;

    // timing
    static float deltaTime;
    static float lastFrame;

    // lighting
    glm::vec3 lightPos = glm::vec3(1.2f, 1.0f, 2.0f);

    Shader* ourShader = nullptr;
    Mesh shape1, shape2;
    std::vector<Mesh> face;
};

