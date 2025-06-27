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

// Compute triangle normal from three vertices
glm::vec3 computeTriangleNormal(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    Eigen::Vector3d normal = edge1.cross(edge2);
    glm::vec3 glm_normal(normal.x(), normal.y(), normal.z());
    return glm::normalize(glm_normal);
}

void buildMeshFromGrid(Dynamic3DArray& array, Mesh& mesh,
    float threshold, float cube_size, const Eigen::Vector3d& min_bound,
    const glm::vec3& color) {
    // Clear mesh
    mesh.vertices.clear();
    mesh.indices.clear();
    mesh.indexCount = 0;

    int nx = array.get_nx();
    int ny = array.get_ny();
    int nz = array.get_nz();

    // Track global vertex index
    unsigned int vertex_index = 0;

    // Corner offsets matching the bit ordering
    const Eigen::Vector3d corner_offsets[8] = {
        {0, 0, 0},     // 0: (i, j, k)
        {0, 0, 1},     // 1: (i, j, k+1)
        {1, 0, 1},     // 2: (i+1, j, k+1)
        {1, 0, 0},     // 3: (i+1, j, k)
        {0, 1, 0},     // 4: (i, j+1, k)
        {0, 1, 1},     // 5: (i, j+1, k+1)
        {1, 1, 1},     // 6: (i+1, j+1, k+1)
        {1, 1, 0}      // 7: (i+1, j+1, k)
    };

    // Edge definitions: each edge is a pair of corner indices
    const std::pair<int, int> edges[12] = {
        {0, 1},  // 0: (i,j,k) to (i,j,k+1)
        {1, 2},  // 1: (i,j,k+1) to (i+1,j,k+1)
        {2, 3},  // 2: (i+1,j,k+1) to (i+1,j,k)
        {3, 0},  // 3: (i+1,j,k) to (i,j,k)
        {4, 5},  // 4: (i,j+1,k) to (i,j+1,k+1)
        {5, 6},  // 5: (i,j+1,k+1) to (i+1,j+1,k+1)
        {6, 7},  // 6: (i+1,j+1,k+1) to (i+1,j+1,k)
        {7, 4},  // 7: (i+1,j+1,k) to (i,j+1,k)
        {0, 4},  // 8: (i,j,k) to (i,j+1,k)
        {1, 5},  // 9: (i,j,k+1) to (i,j+1,k+1)
        {2, 6},  // 10: (i+1,j,k+1) to (i+1,j+1,k+1)
        {3, 7}   // 11: (i+1,j,k) to (i+1,j+1,k)
    };

    // Iterate cubes
    for (int i = 0; i < nx - 1; ++i) {
        for (int j = 0; j < ny - 1; ++j) {
            for (int k = 0; k < nz - 1; ++k) {
                // Compute 8-bit cube index
                uint8_t cube_index = 0;
                cube_index |= (array(i, j, k) <= threshold) << 0;          // Bit 0
                cube_index |= (array(i, j, k + 1) <= threshold) << 1;      // Bit 1
                cube_index |= (array(i + 1, j, k + 1) <= threshold) << 2;  // Bit 2
                cube_index |= (array(i + 1, j, k) <= threshold) << 3;      // Bit 3
                cube_index |= (array(i, j + 1, k) <= threshold) << 4;      // Bit 4
                cube_index |= (array(i, j + 1, k + 1) <= threshold) << 5;  // Bit 5
                cube_index |= (array(i + 1, j + 1, k + 1) <= threshold) << 6; // Bit 6
                cube_index |= (array(i + 1, j + 1, k) <= threshold) << 7;  // Bit 7

                // Get triangulation (up to 15 edge indices for 5 triangles)
                const int* tri = triangulations[cube_index];
                if (tri[0] == -1) { // No triangles for this case
                    continue;
                }

                // Cube origin in world space
                Eigen::Vector3d cube_origin = min_bound + Eigen::Vector3d(i, j, k) * cube_size;


                /*if(k == 20 || k == 19)
                std::cout << cube_origin[0] << " " << cube_origin[1] << " " << cube_origin[2] << "\n";*/

                // Process triangles (3 edge indices at a time)
                for (int t = 0; t < 15 && tri[t] != -1; t += 3) {
                    if (tri[t] < 0 || tri[t + 1] < 0 || tri[t + 2] < 0) {
                        continue; // Skip invalid triangles
                    }

                    // Get edge midpoints for the triangle (reversed winding: v0, v2, v1)
                    Eigen::Vector3d v0 = cube_origin + (corner_offsets[edges[tri[t]].first] + corner_offsets[edges[tri[t]].second]) * cube_size * 0.5;
                    Eigen::Vector3d v1 = cube_origin + (corner_offsets[edges[tri[t + 2]].first] + corner_offsets[edges[tri[t + 2]].second]) * cube_size * 0.5; // Swapped
                    Eigen::Vector3d v2 = cube_origin + (corner_offsets[edges[tri[t + 1]].first] + corner_offsets[edges[tri[t + 1]].second]) * cube_size * 0.5; // Swapped

                    // Compute triangle normal
                    glm::vec3 normal = computeTriangleNormal(v0, v1, v2);

                    // Add vertices for the triangle (v0, v2, v1 order)
                    for (int vi = 0; vi < 3; ++vi) {
                        int edge_idx = (vi == 0) ? tri[t] : (vi == 1) ? tri[t + 2] : tri[t + 1]; // v0, v2, v1
                        Eigen::Vector3d pos = cube_origin + (corner_offsets[edges[edge_idx].first] + corner_offsets[edges[edge_idx].second]) * cube_size * 0.5;
                        mesh.vertices.push_back(pos.x());
                        mesh.vertices.push_back(pos.y());
                        mesh.vertices.push_back(pos.z());
                        mesh.vertices.push_back(normal.x);
                        mesh.vertices.push_back(normal.y);
                        mesh.vertices.push_back(normal.z);
                        mesh.vertices.push_back(color.r);
                        mesh.vertices.push_back(color.g);
                        mesh.vertices.push_back(color.b);
                        mesh.indices.push_back(vertex_index++);
                    }
                }
            }
        }
    }

    // Setup OpenGL buffers
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    glGenVertexArrays(1, &mesh.VAO);
    glGenBuffers(1, &mesh.VBO);
    glGenBuffers(1, &mesh.EBO);

    glBindVertexArray(mesh.VAO);

    glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
    glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float),
        mesh.vertices.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int),
        mesh.indices.data(), GL_DYNAMIC_DRAW);

    // Position: location = 0
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal: location = 1
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Color: location = 2
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    mesh.indexCount = static_cast<GLsizei>(mesh.indices.size());
}


// trilinearInterpolate function (unchanged from previous)
double trilinearInterpolate(double x, double y, double z,
    double minX, double minY, double minZ,
    double cubeSize,
    const std::array<double, 8>& cornerValues,
    double defaultValue = 0.0) {
    // Determine the cube containing the point
    double localMinX = minX;
    double localMinY = minY;
    double localMinZ = minZ;
    std::array<double, 8> localValues = cornerValues;

    // Check if point is outside the original cube
    double x0 = minX;
    double y0 = minY;
    double z0 = minZ;
    double x1 = x0 + cubeSize;
    double y1 = y0 + cubeSize;
    double z1 = z0 + cubeSize;

    if (x < x0 || x > x1 || y < y0 || y > y1 || z < z0 || z > z1) {
        // Calculate the imaginary cube's min corner
        localMinX = x0 + cubeSize * std::floor((x - x0) / cubeSize);
        localMinY = y0 + cubeSize * std::floor((y - y0) / cubeSize);
        localMinZ = z0 + cubeSize * std::floor((z - z0) / cubeSize);

        // Assign values to the imaginary cube's corners
        localValues.fill(defaultValue);
        // Check which corners overlap with the original cube
        for (int i = 0; i < 8; ++i) {
            double cx = (i & 1) ? localMinX + cubeSize : localMinX;
            double cy = (i & 2) ? localMinY + cubeSize : localMinY;
            double cz = (i & 4) ? localMinZ + cubeSize : localMinZ;
            // If this corner lies within the original cube, use the original value
            if (cx >= x0 && cx <= x1 && cy >= y0 && cy <= y1 && cz >= z0 && cz <= z1) {
                int origIndex = ((cx == x1) ? 1 : 0) | ((cy == y1) ? 2 : 0) | ((cz == z1) ? 4 : 0);
                localValues[i] = cornerValues[origIndex];
            }
        }
    }

    // Perform trilinear interpolation
    // Normalize point coordinates to [0,1] within the cube
    double xd = (x - localMinX) / cubeSize;
    double yd = (y - localMinY) / cubeSize;
    double zd = (z - localMinZ) / cubeSize;

    // Clamp to [0,1] to handle numerical precision
    xd = std::clamp(xd, 0.0, 1.0);
    yd = std::clamp(yd, 0.0, 1.0);
    zd = std::clamp(zd, 0.0, 1.0);

    // Interpolate along x
    double c00 = localValues[0] * (1 - xd) + localValues[1] * xd;
    double c01 = localValues[2] * (1 - xd) + localValues[3] * xd;
    double c10 = localValues[4] * (1 - xd) + localValues[5] * xd;
    double c11 = localValues[6] * (1 - xd) + localValues[7] * xd;

    // Interpolate along y
    double c0 = c00 * (1 - yd) + c01 * yd;
    double c1 = c10 * (1 - yd) + c11 * yd;

    // Interpolate along z
    return c0 * (1 - zd) + c1 * zd;
}

// Updated GetSDFValue function
float GetSDFValue(
    const std::vector<float>& sdf_values,
    const Eigen::Vector3d& world_point,
    const Eigen::Vector3d& min_bound,
    const Eigen::Vector3d& max_bound,
    const Eigen::Vector3i& grid_res,
    const glm::mat4& world_to_local_matrix)
{
    // Step 1: Transform world-space point to local space
    glm::vec4 point(world_point.x(), world_point.y(), world_point.z(), 1.0f);
    glm::vec4 local_point = world_to_local_matrix * point;
    Eigen::Vector3d local(local_point.x, local_point.y, local_point.z);

    // Step 2: Check if point is inside AABB and compute distance if outside
    bool is_out_of_bounds = false;
    Eigen::Vector3d query_point = local;
    for (int i = 0; i < 3; ++i) {
        if (local[i] < min_bound[i] || local[i] > max_bound[i]) {
            is_out_of_bounds = true;
            query_point[i] = std::max(min_bound[i], std::min(max_bound[i], local[i]));
        }
    }

    double distance = 0.0;
    if (is_out_of_bounds) {
        glm::vec4 query_local(query_point.x(), query_point.y(), query_point.z(), 1.0f);
        glm::mat4 local_to_world = glm::inverse(world_to_local_matrix);
        glm::vec4 query_world = local_to_world * query_local;
        Eigen::Vector3d query_world_point(query_world.x, query_world.y, query_world.z);
        distance = (world_point - query_world_point).norm();
    }

    // Step 3: Validate grid and compute cell size
    if (sdf_values.size() != static_cast<size_t>(grid_res[0] * grid_res[1] * grid_res[2])) {
        throw std::invalid_argument("SDF values size does not match grid resolution");
    }
    Eigen::Vector3d cell_size;
    for (int i = 0; i < 3; ++i) {
        if (max_bound[i] <= min_bound[i]) {
            throw std::invalid_argument("AABB bounds are invalid (max_bound <= min_bound)");
        }
        cell_size[i] = (max_bound[i] - min_bound[i]) / (grid_res[i] - 1);
    }

    // Step 4: Map query_point to grid cell
    Eigen::Vector3i idx;
    for (int i = 0; i < 3; ++i) {
        double normalized = (query_point[i] - min_bound[i]) / (max_bound[i] - min_bound[i]);
        normalized *= (grid_res[i] - 1);
        idx[i] = static_cast<int>(std::floor(normalized));
        idx[i] = std::max(0, std::min(grid_res[i] - 2, idx[i])); // Ensure valid upper corner
    }

    // Step 5: Get the 8 corner values of the grid cell
    std::array<double, 8> corner_values;
    int nx = grid_res[0], ny = grid_res[1], nz = grid_res[2];
    int indices[8] = {
        (idx[2] + 0) * nx * ny + (idx[1] + 0) * nx + (idx[0] + 0), // (0,0,0)
        (idx[2] + 0) * nx * ny + (idx[1] + 0) * nx + (idx[0] + 1), // (1,0,0)
        (idx[2] + 0) * nx * ny + (idx[1] + 1) * nx + (idx[0] + 0), // (0,1,0)
        (idx[2] + 0) * nx * ny + (idx[1] + 1) * nx + (idx[0] + 1), // (1,1,0)
        (idx[2] + 1) * nx * ny + (idx[1] + 0) * nx + (idx[0] + 0), // (0,0,1)
        (idx[2] + 1) * nx * ny + (idx[1] + 0) * nx + (idx[0] + 1), // (1,0,1)
        (idx[2] + 1) * nx * ny + (idx[1] + 1) * nx + (idx[0] + 0), // (0,1,1)
        (idx[2] + 1) * nx * ny + (idx[1] + 1) * nx + (idx[0] + 1)  // (1,1,1)
    };
    for (int i = 0; i < 8; ++i) {
        if (indices[i] >= 0 && indices[i] < static_cast<int>(sdf_values.size())) {
            corner_values[i] = static_cast<double>(sdf_values[indices[i]]);
        }
        else {
            corner_values[i] = 0.0; // Fallback for invalid indices
        }
    }

    // Step 6: Compute cube parameters for trilinearInterpolate
    double cube_size = cell_size[0]; // Assume uniform cell size (use x-dimension)
    Eigen::Vector3d cell_min_corner = min_bound + idx.cast<double>().cwiseProduct(cell_size);

    // Step 7: Perform trilinear interpolation
    double sdf_value = trilinearInterpolate(
        query_point.x(), query_point.y(), query_point.z(),
        cell_min_corner.x(), cell_min_corner.y(), cell_min_corner.z(),
        cube_size,
        corner_values,
        1.0
    );

    // Step 8: Return SDF value, adding distance for out-of-bounds
    return is_out_of_bounds ? static_cast<float>(sdf_value + distance) : static_cast<float>(sdf_value);
}

void RunTest() {
    // Define cube corners (arbitrary positions, but we'll use min corner and size)
    std::vector<Eigen::Vector3d> cubeCorners = {
        {0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {2, 2, 0}, // Bottom face
        {0, 0, 2}, {2, 0, 2}, {0, 2, 2}, {2, 2, 2}  // Top face
    };

    // Define values at corners
    std::vector<double> cornerValuesVec = { -1, -1, -1, -1, 1, 1, 1, 1 };
    // Convert to std::array for trilinearInterpolate
    std::array<double, 8> cornerValues;
    std::copy(cornerValuesVec.begin(), cornerValuesVec.end(), cornerValues.begin());

    // Define cube size (maximum distance for influence)
    double cubeSize = 2.0;

    // Define minimum corner of the cube
    double minX = 0.0, minY = 0.0, minZ = 0.0;

    // Test points
    std::vector<Eigen::Vector3d> testPoints = {
        {1, 1, 1},      // Inside cube
        {3, 3, 3},      // Outside but within cubeSize
        {10, 10, 10},   // Far outside
        {1, 1, 1.5},    // Inside cube
        {1, 3, 1.5},    // Outside cube
        {1, 1, 2},      // On cube boundary
        {1, 1, 3}       // Outside cube
    };

    for (const auto& point : testPoints) {
        double result = trilinearInterpolate(point.x(), point.y(), point.z(),
            minX, minY, minZ,
            cubeSize,
            cornerValues,
            -1.0);
        std::cout << "Point (" << point.transpose() << "): Value = ";
        if (std::isnan(result)) {
            std::cout << "NaN (invalid)" << std::endl;
        }
        else {
            std::cout << result << std::endl;
        }
    }
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

    // Step 6: Extend bounds by half a cube size
    min_bound -= Eigen::Vector3d(cube_size / 2.0, cube_size / 2.0, cube_size / 2.0);
    max_bound += Eigen::Vector3d(cube_size / 2.0, cube_size / 2.0, cube_size / 2.0);
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
    //shape1 = Shapes::CreateSphere(1.0f, 64, 64, glm::vec3(0.6f, 0.2f, 0.9f));
    shape1 = Shapes::CreateBox(1.0f, 1.0f, 2.0f, glm::vec3(0.6f, 0.2f, 0.9f));
    shape2 = Shapes::CreateBox(1.0f, 1.0f, 2.0f, glm::vec3(0.2f, 0.6f, 0.9f));//Shapes::CreateSphere(1.0f, 64, 64, glm::vec3(0.2f, 0.6f, 0.9f));
    //shape2 = Shapes::CreateBox(1.0f, 1.0f, 2.0f, glm::vec3(0.2f, 0.6f, 0.9f)); // Original size

    glBindVertexArray(shape1.VAO);
    glDrawElements(GL_TRIANGLES, shape1.indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    ourShader = new Shader("shader.vs", "shader.fs");

    //Cube marching
    glm::mat4 model10 = glm::mat4(1.0f);
    glm::mat4 model20 = glm::mat4(1.0f);
    model20 = glm::translate(model20, glm::vec3(0.0f, 0.0f, 0.0f));
    model10 = glm::translate(model10, glm::vec3(0.5f, 0.5f, 0.5f));

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
    ComputeSDFGrid(shape1, model10, V, F, sdf_values2, grid_res, min_bound1, max_bound1);

    // Create SDF texture
    texture1 = CreateSDFTexture(sdf_values2, grid_res);

    // Compute AABB for shape2
    ComputeMeshAABB(shape2, min_bound2, max_bound2); // Assumes shape2 has a Mesh member

    ComputeSDFGrid(shape2, model20, V, F, sdf_values1, grid_res, min_bound2, max_bound2);

    // Create SDF texture
    texture2 = CreateSDFTexture(sdf_values1, grid_res);



    std::vector<Mesh> meshes;
    meshes.push_back(shape1);
    meshes.push_back(shape2);
    std::vector<glm::mat4>transforms;
    transforms.push_back(model10);
    transforms.push_back(model20);
    double cube_size = 0.03f;
    Eigen::Vector3d min_bound;
    Eigen::Vector3d max_bound;
    //RunTest();
    ComputeSDFWorldBounds(meshes, transforms, cube_size, min_bound, max_bound);
    int sizeX = abs(min_bound[0] - max_bound[0])/ cube_size + 1, sizeY = abs(min_bound[1] - max_bound[1])/ cube_size + 1, sizeZ = abs(min_bound[2] - max_bound[2])/ cube_size + 1;
    VoxelArray = new Dynamic3DArray(sizeX, sizeY, sizeZ, 1000);


    // 0 Intersection
    // 1 Union
    // 2 Difference
    int boolCase = 0;
    

    for (int i = 0;i < sizeX;i++) {
        for (int j = 0;j < sizeY;j++) {
            for (int k = 0;k < sizeZ;k++) {
                const Eigen::Vector3d WorldPoint = min_bound + Eigen::Vector3d(i,j,k) * cube_size;
                switch (boolCase) {
                case 0:
                {
                    (*VoxelArray)(i, j, k) = std::max(
                        GetSDFValue(sdf_values1, WorldPoint, min_bound1, max_bound1, grid_res, model10),
                        GetSDFValue(sdf_values2, WorldPoint, min_bound2, max_bound2, grid_res, model20)
                    );
                    break;
                }
                case 1:
                {
                    (*VoxelArray)(i, j, k) = std::min(
                        GetSDFValue(sdf_values1, WorldPoint, min_bound1, max_bound1, grid_res, model10),
                        GetSDFValue(sdf_values2, WorldPoint, min_bound2, max_bound2, grid_res, model20)
                    );
                    break;
                }
                case 2:
                {
                    (*VoxelArray)(i, j, k) = std::max(
                        GetSDFValue(sdf_values1, WorldPoint, min_bound1, max_bound1, grid_res, model10),
                        -GetSDFValue(sdf_values2, WorldPoint, min_bound2, max_bound2, grid_res, model20)
                    );
                    break;
                }
                default:
                {

                }
                }
                /*if(k==21)
                std::cout << i << " " << j << " " << k << " " << (*VoxelArray)(i, j, k) << std::endl;*/
            }
        }
    }
    buildMeshFromGrid((*VoxelArray), test, 0.01f, cube_size, min_bound, glm::vec3(1.0f, 1.0f, 1.0f));
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
    model1 = glm::translate(model1, glm::vec3(0.5f, 0.5f, 0.5f));
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

    glm::mat4 model3 = glm::mat4(1.0f);
    model3 = glm::translate(model3, glm::vec3(2.0f));
    ourShader->setMat4("model", model3);
    ourShader->setMat4("worldToLocalMatrix1", glm::inverse(model3)); // Dummy for shape1
    ourShader->setVec3("minBound1", min_bound1[0], min_bound1[1], min_bound1[2]); // Dummy values
    ourShader->setVec3("maxBound1", max_bound1[0], max_bound1[1], max_bound1[2]);
    ourShader->setMat4("worldToLocalMatrix2", glm::inverse(model2));
    ourShader->setVec3("minBound2", min_bound2[0], min_bound2[1], min_bound2[2]);
    ourShader->setVec3("maxBound2", max_bound2[0], max_bound2[1], max_bound2[2]);
    glBindVertexArray(test.VAO);
    glDrawElements(GL_TRIANGLES, test.indexCount, GL_UNSIGNED_INT, 0);
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