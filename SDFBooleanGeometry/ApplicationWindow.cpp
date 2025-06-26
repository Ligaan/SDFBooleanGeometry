#define _SILENCE_CXX20_IS_POD_DEPRECATION_WARNING
#define NOMINMAX
#include "ApplicationWindow.h"

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>

#include <iostream>
#include <vector>

#include <igl/AABB.h>
#include <igl/signed_distance.h>
#include <igl/readOFF.h>
#include <stdexcept>
#include <Eigen/Dense>
#include <igl/opengl/glfw/Viewer.h>

//#include <algorithm> // For std::min, std::max

float GetSDFValue(
    const std::vector<float>& sdf_values,
    const Eigen::Vector3d& world_point,
    const Eigen::Vector3d& min_bound,
    const Eigen::Vector3d& max_bound,
    const Eigen::Vector3i& grid_res,
    const glm::mat4& world_to_local_matrix,
    float out_of_bounds_value)
{
    // Step 1: Transform world-space point to local space
    glm::vec4 point(world_point.x(), world_point.y(), world_point.z(), 1.0f);
    glm::vec4 local_point = world_to_local_matrix * point;
    Eigen::Vector3d local(local_point.x, local_point.y, local_point.z);

    // Step 2: Check if point is outside AABB
    for (int i = 0; i < 3; ++i) {
        if (local[i] < min_bound[i] || local[i] > max_bound[i]) {
            return out_of_bounds_value; // Return custom value for out-of-bounds points
        }
    }

    // Step 3: Map local point to grid coordinates
    Eigen::Vector3d normalized;
    for (int i = 0; i < 3; ++i) {
        if (max_bound[i] == min_bound[i]) {
            throw std::invalid_argument("AABB bounds are invalid (max_bound equals min_bound)");
        }
        normalized[i] = (local[i] - min_bound[i]) / (max_bound[i] - min_bound[i]);
        normalized[i] *= (grid_res[i] - 1);
    }

    // Step 4: Compute grid indices and interpolation weights
    Eigen::Vector3i idx;
    Eigen::Vector3d weights;
    for (int i = 0; i < 3; ++i) {
        idx[i] = static_cast<int>(std::floor(normalized[i]));
        weights[i] = normalized[i] - idx[i];
        // Clamp indices to ensure they stay within bounds
        idx[i] = std::max(0, std::min(grid_res[i] - 1, idx[i]));
    }

    // Step 5: Get the 8 corner values of the grid cell
    float values[8];
    int nx = grid_res[0], ny = grid_res[1], nz = grid_res[2];
    if (sdf_values.size() != static_cast<size_t>(nx * ny * nz)) {
        throw std::invalid_argument("SDF values size does not match grid resolution");
    }

    int indices[8] = {
        (idx[2] + 0) * nx * ny + (idx[1] + 0) * nx + (idx[0] + 0),
        (idx[2] + 0) * nx * ny + (idx[1] + 0) * nx + (idx[0] + 1),
        (idx[2] + 0) * nx * ny + (idx[1] + 1) * nx + (idx[0] + 0),
        (idx[2] + 0) * nx * ny + (idx[1] + 1) * nx + (idx[0] + 1),
        (idx[2] + 1) * nx * ny + (idx[1] + 0) * nx + (idx[0] + 0),
        (idx[2] + 1) * nx * ny + (idx[1] + 0) * nx + (idx[0] + 1),
        (idx[2] + 1) * nx * ny + (idx[1] + 1) * nx + (idx[0] + 0),
        (idx[2] + 1) * nx * ny + (idx[1] + 1) * nx + (idx[0] + 1)
    };

    for (int i = 0; i < 8; ++i) {
        values[i] = sdf_values[indices[i]];
    }

    // Step 6: Trilinear interpolation
    float c00 = values[0] * (1 - weights[0]) + values[1] * weights[0];
    float c10 = values[2] * (1 - weights[0]) + values[3] * weights[0];
    float c01 = values[4] * (1 - weights[0]) + values[5] * weights[0];
    float c11 = values[6] * (1 - weights[0]) + values[7] * weights[0];

    float c0 = c00 * (1 - weights[1]) + c10 * weights[1];
    float c1 = c01 * (1 - weights[1]) + c11 * weights[1];

    float sdf_value = c0 * (1 - weights[2]) + c1 * weights[2];

    return sdf_value;
}

void ComputeSDFWorldBounds(
    const std::vector<Mesh>& meshes,
    const std::vector<glm::mat4>& transforms,
    double cube_size,
    Eigen::Vector3d& min_bound,
    Eigen::Vector3d& max_bound)
{
    if (meshes.size() != transforms.size()) {
        throw std::invalid_argument("Number of meshes must match number of transforms");
    }
    if (meshes.empty()) {
        throw std::invalid_argument("Mesh list cannot be empty");
    }
    if (cube_size <= 0.0) {
        throw std::invalid_argument("Cube size must be positive");
    }

    // Initialize bounds to extreme values
    min_bound = Eigen::Vector3d(std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max());
    max_bound = Eigen::Vector3d(std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest());

    // Define the 90-degree y-axis rotation matrix (as in ConvertMeshToLibigl)
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    for (size_t i = 0; i < meshes.size(); ++i) {
        const Mesh& mesh = meshes[i];
        const glm::mat4& transform = transforms[i];

        // Step 1: Compute local AABB
        Eigen::Vector3d local_min(std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        Eigen::Vector3d local_max(std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest());

        if (mesh.vertices.empty()) {
            throw std::invalid_argument("Mesh vertices cannot be empty");
        }

        // Assume vertices are [x, y, z, ...] with stride 9 (position, normal, color)
        for (size_t j = 0; j < mesh.vertices.size(); j += 9) {
            Eigen::Vector3d vertex(mesh.vertices[j], mesh.vertices[j + 1], mesh.vertices[j + 2]);
            local_min = local_min.cwiseMin(vertex);
            local_max = local_max.cwiseMax(vertex);
        }

        // Step 2: Apply 90-degree y-axis rotation to AABB corners
        std::vector<Eigen::Vector3d> corners = {
            {local_min.x(), local_min.y(), local_min.z()},
            {local_max.x(), local_min.y(), local_min.z()},
            {local_min.x(), local_max.y(), local_min.z()},
            {local_max.x(), local_max.y(), local_min.z()},
            {local_min.x(), local_min.y(), local_max.z()},
            {local_max.x(), local_min.y(), local_max.z()},
            {local_min.x(), local_max.y(), local_max.z()},
            {local_max.x(), local_max.y(), local_max.z()}
        };

        Eigen::Vector3d rotated_min(std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        Eigen::Vector3d rotated_max(std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest());

        for (const auto& corner : corners) {
            glm::vec4 pos(corner.x(), corner.y(), corner.z(), 1.0f);
            glm::vec4 rotated_pos = rotation * pos;
            Eigen::Vector3d rotated_corner(rotated_pos.x, rotated_pos.y, rotated_pos.z);
            rotated_min = rotated_min.cwiseMin(rotated_corner);
            rotated_max = rotated_max.cwiseMax(rotated_corner);
        }

        // Step 3: Apply world-space transformation
        Eigen::Vector3d world_min(std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        Eigen::Vector3d world_max(std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest());

        for (const auto& corner : corners) {
            glm::vec4 pos(corner.x(), corner.y(), corner.z(), 1.0f);
            // Apply rotation first, then world transform
            glm::vec4 rotated_pos = rotation * pos;
            glm::vec4 world_pos = transform * rotated_pos;
            Eigen::Vector3d world_corner(world_pos.x, world_pos.y, world_pos.z);
            world_min = world_min.cwiseMin(world_corner);
            world_max = world_max.cwiseMax(world_corner);
        }

        // Step 4: Update global bounds
        min_bound = min_bound.cwiseMin(world_min);
        max_bound = max_bound.cwiseMax(world_max);
    }

    // Step 5: Adjust bounds to be divisible by cube_size
    Eigen::Vector3d extent = max_bound - min_bound;
    for (int i = 0; i < 3; ++i) {
        // Calculate number of cubes needed (round up to ensure coverage)
        int num_cubes = static_cast<int>(std::ceil(extent[i] / cube_size));
        // New extent must be a multiple of cube_size
        double new_extent = num_cubes * cube_size;
        // Center the adjusted bounds around the original AABB
        double center = (min_bound[i] + max_bound[i]) / 2.0;
        min_bound[i] = center - new_extent / 2.0;
        max_bound[i] = center + new_extent / 2.0;
    }
}

void visualizeSDF(const Eigen::MatrixXd& P, const Eigen::VectorXd& S, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    igl::opengl::glfw::Viewer viewer;

    // Add mesh for context
    viewer.data().set_mesh(V, F);

    // Add grid points colored by SDF
    Eigen::MatrixXd colors(P.rows(), 3);
    double max_sdf = S.cwiseAbs().maxCoeff();
    for (int i = 0; i < P.rows(); ++i) {
        float normalized_sdf = (S(i) / max_sdf) * 0.5 + 0.5; // Map to [0,1]
        colors(i, 0) = /*normalized_sdf*/ S(i) <= 0 ? 1.0f : 0.0f; // Red: SDF value
        colors(i, 1) = /*normalized_sdf*/S(i) <= 0 ? 1.0f : 0.0f;; // Green: SDF value
        colors(i, 2) = /*normalized_sdf*/S(i) <= 0 ? 1.0f : 0.0f;; // Blue: SDF value
    }
    viewer.data().add_points(P, colors);

    // Launch viewer
    viewer.launch();
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

Camera ApplicationWindow::camera = Camera(glm::vec3(3.5f, 0.0f, 7.8f));        // Define the static Camera object
float ApplicationWindow::lastX = 0.0f;   // Define and initialize static variables
float ApplicationWindow::lastY = 0.0f;
bool ApplicationWindow::firstMouse = true;
bool ApplicationWindow::buttonPressed = false;
float ApplicationWindow::deltaTime = 0.0f;
float ApplicationWindow::lastFrame = 0.0f;

void ApplicationWindow::Initialize()
{
    ApplicationWindow::lastX = SCR_WIDTH / 2.0f;
    ApplicationWindow::lastY = SCR_HEIGHT / 2.0f;
    // glfw: initialize and configure
   // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return;
    }

    glEnable(GL_DEPTH_TEST);

    // Create shapes
    shape1 = Shapes::CreateSphere(1.0f, 64, 64, glm::vec3(0.6f, 0.2f, 0.9f));
    shape2 = Shapes::CreateBox(1.0f, 1.0f, 2.0f, glm::vec3(0.2f, 0.6f, 0.9f)); // Original size

    glBindVertexArray(shape1.VAO);
    glDrawElements(GL_TRIANGLES, shape1.indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    ourShader = new Shader("shader.vs", "shader.fs");

    // Compute SDF grid for shape2 (in local space)
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    std::vector<float> sdf_values1;
    std::vector<float> sdf_values2;
    Eigen::Vector3i grid_res(32, 32, 32);

    // Compute AABB for shape2
    ComputeMeshAABB(shape1, min_bound1, max_bound1); // Assumes shape2 has a Mesh member

    glm::mat4 model2 = glm::mat4(1.0f); // Transform unused in ComputeSDFGrid
    model2 = glm::translate(model2,glm::vec3(0.0f)); // Transform unused in ComputeSDFGrid
    ComputeSDFGrid(shape1, model2, V, F, sdf_values2, grid_res, min_bound1, max_bound1);

    // Create SDF texture
    texture1 = CreateSDFTexture(sdf_values2, grid_res);

    // Compute AABB for shape2
    ComputeMeshAABB(shape2, min_bound2, max_bound2); // Assumes shape2 has a Mesh member

    ComputeSDFGrid(shape2, model2, V, F, sdf_values1, grid_res, min_bound2, max_bound2);

    // Create SDF texture
    texture2 = CreateSDFTexture(sdf_values1, grid_res);


    //Cube marching
    glm::mat4 model10 = glm::mat4(1.0f);
    glm::mat4 model20 = glm::mat4(1.0f);
    model20 = glm::translate(model20, glm::vec3(0.0f, 0.0f, 0.0f));
    model10 = glm::translate(model10, glm::vec3(0.0f, 0.0f, 0.0f));

    std::vector<Mesh> meshes;
    meshes.push_back(shape1);
    meshes.push_back(shape2);
    std::vector<glm::mat4>transforms;
    transforms.push_back(model10);
    transforms.push_back(model20);
    double cube_size = 0.1f;
    Eigen::Vector3d min_bound;
    Eigen::Vector3d max_bound;
    ComputeSDFWorldBounds(meshes, transforms, cube_size, min_bound, max_bound);
    int sizeX = abs(min_bound[0] - max_bound[0])/ cube_size, sizeY = abs(min_bound[1] - max_bound[1])/ cube_size, sizeZ = abs(min_bound[2] - max_bound[2])/ cube_size;
    VoxelArray = new Dynamic3DArray(sizeX, sizeY, sizeZ, 1000);

    for (int i = 0;i < sizeX;i++) {
        for (int j = 0;j < sizeY;j++) {
            for (int k = 0;k < sizeZ;k++) {
                const Eigen::Vector3d WorldPoint = min_bound + Eigen::Vector3d(i,j,k) * cube_size;
                (*VoxelArray)(i, j, k) =std::max(GetSDFValue(sdf_values1, WorldPoint,min_bound1,max_bound1,grid_res, model10,100), GetSDFValue(sdf_values2, WorldPoint, min_bound2, max_bound2, grid_res, model20,100));

                std::cout << i << " " << j << " " << k << " " << (*VoxelArray)(i, j, k) << std::endl;
            }
        }
    }
}

void ApplicationWindow::Update()
{
    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);
        Render();
    }
}

void ApplicationWindow::Render()
{
    // Clear buffers
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use shader
    ourShader->use();

    // Set view/projection
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
    glm::mat4 view = camera.GetViewMatrix();
    ourShader->setMat4("projection", projection);
    ourShader->setMat4("view", view);
    ourShader->setFloat("Multi", 1.0f);

    // Set light properties
    ourShader->setVec3("light.direction", -0.2f, -1.0f, -0.3f);
    ourShader->setVec3("light.color", 1.0f, 1.0f, 1.0f); // White light
    ourShader->setFloat("light.ambientStrength", 0.2f);
    ourShader->setFloat("light.diffuseStrength", 0.5f);
    ourShader->setVec3("viewPos", camera.Position);

    // Set SDF texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, texture1);
    ourShader->setInt("sdfTexture1", 0);  // Tell shader that sdfTexture1 uses unit 0

    // Bind second SDF texture to texture unit 1
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_3D, texture2);
    ourShader->setInt("sdfTexture2", 1);  // Tell shader that sdfTexture2 uses unit 1

    // Render shape1 (sphere, no SDF)
    glm::mat4 model1 = glm::mat4(1.0f);
    glm::mat4 model2 = glm::mat4(1.0f);
    model2 = glm::translate(model2, glm::vec3(0.0f, 0.0f, 0.0f));
    model1 = glm::translate(model1, glm::vec3(0.0f, 0.0f, 0.0f));
    ourShader->setMat4("model", model1);
    ourShader->setMat4("worldToLocalMatrix1", glm::inverse(model1)); // Dummy for shape1
    ourShader->setVec3("minBound1", min_bound1[0], min_bound1[1], min_bound1[2]); // Dummy values
    ourShader->setVec3("maxBound1", max_bound1[0], max_bound1[1], max_bound1[2]);
    ourShader->setMat4("worldToLocalMatrix2", glm::inverse(model2));
    ourShader->setVec3("minBound2", min_bound2[0], min_bound2[1], min_bound2[2]);
    ourShader->setVec3("maxBound2", max_bound2[0], max_bound2[1], max_bound2[2]);
    glBindVertexArray(shape1.VAO);
    glDrawElements(GL_TRIANGLES, shape1.indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    // Render shape2 (with SDF)
    ourShader->setMat4("model", model2);
    ourShader->setMat4("worldToLocalMatrix1", glm::inverse(model1));
    ourShader->setVec3("minBound1", min_bound1[0], min_bound1[1], min_bound1[2]);
    ourShader->setVec3("maxBound1", max_bound1[0], max_bound1[1], max_bound1[2]);
    ourShader->setMat4("worldToLocalMatrix2", glm::inverse(model2));
    ourShader->setVec3("minBound2", min_bound2[0], min_bound2[1], min_bound2[2]);
    ourShader->setVec3("maxBound2", max_bound2[0], max_bound2[1], max_bound2[2]);
    glBindVertexArray(shape2.VAO);
    glDrawElements(GL_TRIANGLES, shape2.indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    // Swap buffers and poll events
    glfwSwapBuffers(window);
    glfwPollEvents();
}

void ApplicationWindow::Shutdown()
{
    glDeleteVertexArrays(1, &shape1.VAO);
    glDeleteVertexArrays(1, &shape2.VAO);
    glfwTerminate();
}

void ApplicationWindow::ConvertMeshToLibigl(
    const std::vector<float>& vertices,
    const std::vector<unsigned int>& indices,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    int vertex_stride, // Default: position (3), normal (3), color (3)
    bool is_quad_mesh)
{
    // Validate inputs
    if (vertex_stride < 3) {
        throw std::invalid_argument("Vertex stride must be at least 3 for positions");
    }
    if (vertices.size() % vertex_stride != 0) {
        throw std::invalid_argument("Vertices size must be divisible by stride");
    }

    // Extract positions and apply rotation
    int num_vertices = vertices.size() / vertex_stride;
    V.resize(num_vertices, 3);

    // Create a 90-degree rotation matrix around the y-axis
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    for (int i = 0; i < num_vertices; ++i) {
        // Extract position (first 3 components)
        glm::vec4 pos(vertices[i * vertex_stride + 0], // x
            vertices[i * vertex_stride + 1], // y
            vertices[i * vertex_stride + 2], // z
            1.0f); // Homogeneous coordinate

        // Apply rotation
        glm::vec4 rotated_pos = rotation * pos;

        // Store rotated position
        V(i, 0) = rotated_pos.x;
        V(i, 1) = rotated_pos.y;
        V(i, 2) = rotated_pos.z;
    }

    // Convert faces (unchanged)
    if (is_quad_mesh) {
        if (indices.size() % 6 != 0) {
            throw std::invalid_argument("Quad mesh indices size must be divisible by 6");
        }
        int num_quads = indices.size() / 6;
        F.resize(num_quads * 2, 3); // 2 triangles per quad
        for (int i = 0; i < num_quads; ++i) {
            F(i * 2 + 0, 0) = indices[i * 6 + 0];
            F(i * 2 + 0, 1) = indices[i * 6 + 1];
            F(i * 2 + 0, 2) = indices[i * 6 + 2];
            F(i * 2 + 1, 0) = indices[i * 6 + 0];
            F(i * 2 + 1, 1) = indices[i * 6 + 2];
            F(i * 2 + 1, 2) = indices[i * 6 + 3];
        }
    }
    else {
        if (indices.size() % 3 != 0) {
            throw std::invalid_argument("Triangle mesh indices size must be divisible by 3");
        }
        int num_triangles = indices.size() / 3;
        F.resize(num_triangles, 3);
        for (int i = 0; i < num_triangles; ++i) {
            F(i, 0) = indices[i * 3 + 0];
            F(i, 1) = indices[i * 3 + 1];
            F(i, 2) = indices[i * 3 + 2];
        }
    }
}

void ApplicationWindow::ComputeSDFGrid(const Mesh& mesh,
const glm::mat4& transform, // Unused, kept for compatibility
Eigen::MatrixXd& V,
Eigen::MatrixXi& F,
std::vector<float>& sdf_values,
Eigen::Vector3i grid_res,
Eigen::Vector3d min_bound,
Eigen::Vector3d max_bound)
{
    if (grid_res[0] <= 1 || grid_res[1] <= 1 || grid_res[2] <= 1) {
        throw std::invalid_argument("Grid resolution must be > 1 in all dimensions");
    }
    if ((max_bound - min_bound).array().minCoeff() <= 0) {
        throw std::invalid_argument("max_bound must be greater than min_bound");
    }
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        throw std::invalid_argument("Mesh vertices or indices cannot be empty");
    }

    ConvertMeshToLibigl(mesh.vertices, mesh.indices, V, F, 9, false);

    int nx = grid_res[0], ny = grid_res[1], nz = grid_res[2];
    int total_points = nx * ny * nz;
    Eigen::MatrixXd P(total_points, 3);
    sdf_values.resize(total_points);

    Eigen::Vector3d delta = (max_bound - min_bound).array() / Eigen::Vector3d(nx - 1, ny - 1, nz - 1).array();

    int idx = 0;
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            for (int k = 0; k < nz; ++k) {
                P(idx, 0) = min_bound[0] + i * delta[0];
                P(idx, 1) = min_bound[1] + j * delta[1];
                P(idx, 2) = min_bound[2] + k * delta[2];
                ++idx;
            }
        }
    }

    Eigen::VectorXd S;
    Eigen::VectorXi I;
    Eigen::MatrixXd C, N;
    igl::signed_distance(P, V, F, igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL, S, I, C, N);

    for (int i = 0; i < total_points; ++i) {
        sdf_values[i] = static_cast<float>(S(i));
    }

    //visualizeSDF(P, S, V, F);
}

GLuint ApplicationWindow::CreateSDFTexture(const std::vector<float>& sdf_values, const Eigen::Vector3i& grid_res)
{
    if (grid_res[0] <= 0 || grid_res[1] <= 0 || grid_res[2] <= 0) {
        throw std::invalid_argument("Grid resolution must be positive");
    }
    if (sdf_values.size() != static_cast<size_t>(grid_res[0] * grid_res[1] * grid_res[2])) {
        throw std::invalid_argument("SDF values size does not match grid resolution");
    }

    GLuint texture;
    glGenTextures(1, &texture);
    if (texture == 0) {
        throw std::runtime_error("Failed to generate texture");
    }

    glBindTexture(GL_TEXTURE_3D, texture);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, grid_res[0], grid_res[1], grid_res[2], 0, GL_RED, GL_FLOAT, sdf_values.data());

    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        glDeleteTextures(1, &texture);
        throw std::runtime_error("Failed to create 3D texture, OpenGL error: " + std::to_string(error));
    }

    glBindTexture(GL_TEXTURE_3D, 0);
    return texture;
}

void ApplicationWindow::ComputeMeshAABB(const Mesh& mesh, Eigen::Vector3d& min_bound, Eigen::Vector3d& max_bound)
{
    min_bound = Eigen::Vector3d(std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max());
    max_bound = Eigen::Vector3d(std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest());

    // Assume vertices are [x, y, z, ...] with stride 3
    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
        Eigen::Vector3d vertex(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]);
        min_bound = min_bound.cwiseMin(vertex);
        max_bound = max_bound.cwiseMax(vertex);
    }

    // Add small padding to ensure SDF captures surface
    Eigen::Vector3d padding(0.0, 0.0, 0.0); // Adjust as needed
    min_bound -= padding;
    max_bound += padding;
}

void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    int state = glfwGetKey(window, GLFW_KEY_SPACE);
    if (state == GLFW_PRESS)
        ApplicationWindow::buttonPressed = true;
    else if (state == GLFW_RELEASE)
        ApplicationWindow::buttonPressed = false;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        ApplicationWindow::camera.ProcessKeyboard(FORWARD, ApplicationWindow::deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        ApplicationWindow::camera.ProcessKeyboard(BACKWARD, ApplicationWindow::deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        ApplicationWindow::camera.ProcessKeyboard(LEFT, ApplicationWindow::deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        ApplicationWindow::camera.ProcessKeyboard(RIGHT, ApplicationWindow::deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (ApplicationWindow::firstMouse)
    {
        ApplicationWindow::lastX = xpos;
        ApplicationWindow::lastY = ypos;
        ApplicationWindow::firstMouse = false;
    }

    float xoffset = xpos - ApplicationWindow::lastX;
    float yoffset = ApplicationWindow::lastY - ypos; // reversed since y-coordinates go from bottom to top

    ApplicationWindow::lastX = xpos;
    ApplicationWindow::lastY = ypos;

    ApplicationWindow::camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    ApplicationWindow::camera.ProcessMouseScroll(static_cast<float>(yoffset));
}