#pragma once
#include "glad/glad.h"
#include "glfw3.h"
#include "Shader.h"
#include "Camera.h"
#include "Shapes.h"
#include <Eigen/Core>

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

    void ConvertMeshToLibigl(
        const std::vector<float>& vertices,
        const std::vector<unsigned int>& indices,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        int vertex_stride = 9, // Default: position (3), normal (3), color (3)
        bool is_quad_mesh = false);

    void ComputeSDFGrid(
        const Mesh& mesh,
        const glm::mat4& transform, // Unused, kept for compatibility
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        std::vector<float>& sdf_values,
        Eigen::Vector3i grid_res,
        Eigen::Vector3d min_bound,
        Eigen::Vector3d max_bound);

    GLuint CreateSDFTexture(const std::vector<float>& sdf_values, const Eigen::Vector3i& grid_res);

    void ComputeMeshAABB(const Mesh& mesh, Eigen::Vector3d& min_bound, Eigen::Vector3d& max_bound);

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
    GLuint texture1, texture2;
    Eigen::Vector3d min_bound, max_bound; // Store SDF bounds for rendering
};

