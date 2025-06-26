#pragma once
#include "glad/glad.h"
#include "glfw3.h"
#include "Shader.h"
#include "Camera.h"
#include "Shapes.h"
#include <Eigen/Core>

struct Dynamic3DArray {
    // Constructor: Initialize with dimensions nx, ny, nz
    Dynamic3DArray(int nx, int ny, int nz, float initial_value = 0.0f)
        : nx_(nx), ny_(ny), nz_(nz), data_(nx* ny* nz, initial_value) {
        if (nx <= 0 || ny <= 0 || nz <= 0) {
            throw std::invalid_argument("Dimensions must be positive");
        }
    }

    // Access element at (i, j, k)
    float& operator()(int i, int j, int k) {
        CheckBounds(i, j, k);
        return data_[k * nx_ * ny_ + j * nx_ + i];
    }

    const float& operator()(int i, int j, int k) const {
        CheckBounds(i, j, k);
        return data_[k * nx_ * ny_ + j * nx_ + i];
    }

    // Get dimensions
    int get_nx() const { return nx_; }
    int get_ny() const { return ny_; }
    int get_nz() const { return nz_; }

    // Resize the array, preserving data where possible
    void resize(int nx, int ny, int nz, float initial_value = 0.0f) {
        if (nx <= 0 || ny <= 0 || nz <= 0) {
            throw std::invalid_argument("Dimensions must be positive");
        }
        std::vector<float> new_data(nx * ny * nz, initial_value);
        // Copy existing data if possible
        int min_nx = std::min(nx_, nx);
        int min_ny = std::min(ny_, ny);
        int min_nz = std::min(nz_, nz);
        for (int k = 0; k < min_nz; ++k) {
            for (int j = 0; j < min_ny; ++j) {
                for (int i = 0; i < min_nx; ++i) {
                    new_data[k * nx * ny + j * nx + i] = (*this)(i, j, k);
                }
            }
        }
        data_ = std::move(new_data);
        nx_ = nx;
        ny_ = ny;
        nz_ = nz;
    }

    void CheckBounds(int i, int j, int k) const {
        if (i < 0 || i >= nx_ || j < 0 || j >= ny_ || k < 0 || k >= nz_) {
            throw std::out_of_range("Index out of bounds");
        }
    }

    int nx_, ny_, nz_; // Dimensions
    std::vector<float> data_; // Underlying storage
};

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
    Eigen::Vector3d min_bound1, max_bound1, min_bound2, max_bound2; // Store SDF bounds for rendering
    int XSize, YSize, ZSize;
    Dynamic3DArray* VoxelArray = nullptr;
};

