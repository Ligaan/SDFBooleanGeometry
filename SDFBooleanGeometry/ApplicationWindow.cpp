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
#include <Eigen/Core>

void ConvertMeshToLibigl(
    const std::vector<float>& vertices,
    const std::vector<unsigned int>& indices,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F)
{
    int num_vertices = vertices.size() / 9; // 9 floats per vertex (3 pos, 3 normal, 3 color)
    V.resize(num_vertices, 3);
    for (int i = 0; i < num_vertices; ++i) {
        V(i, 0) = vertices[i * 9 + 0]; // x
        V(i, 1) = vertices[i * 9 + 1]; // y
        V(i, 2) = vertices[i * 9 + 2]; // z
    }

    int num_quads = indices.size() / 6; // 6 indices per quad (2 triangles)
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

void ComputeSDFGrid(
    const Mesh& mesh,
    const glm::mat4& transform,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    std::vector<float>& sdf_values,
    Eigen::Vector3i grid_res,
    Eigen::Vector3d min_bound,
    Eigen::Vector3d max_bound)
{
    // Convert mesh to libigl format
    ConvertMeshToLibigl(mesh.vertices, mesh.indices, V, F);

    // Apply transform to vertices
    Eigen::MatrixXd V_transformed(V.rows(), V.cols());
    for (int i = 0; i < V.rows(); ++i) {
        glm::vec4 v(V(i, 0), V(i, 1), V(i, 2), 1.0f);
        glm::vec4 v_t = transform * v;
        V_transformed(i, 0) = v_t.x;
        V_transformed(i, 1) = v_t.y;
        V_transformed(i, 2) = v_t.z;
    }

    // Create query points grid
    int nx = grid_res[0], ny = grid_res[1], nz = grid_res[2];
    int total_points = nx * ny * nz;
    Eigen::MatrixXd P(total_points, 3);
    sdf_values.resize(total_points);

    // Grid spacing
    Eigen::Vector3d delta = (max_bound - min_bound).array() / Eigen::Vector3d(nx - 1, ny - 1, nz - 1).array();

    // Populate query points
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

    // Compute SDF
    Eigen::VectorXd S;
    Eigen::VectorXi I;
    Eigen::MatrixXd C, N;
    igl::signed_distance(P, V_transformed, F, igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL, S, I, C, N);

    // Store SDF values
    for (int i = 0; i < total_points; ++i) {
        sdf_values[i] = static_cast<float>(S(i));
    }
}

GLuint CreateSDFTexture(const std::vector<float>& sdf_values, const Eigen::Vector3i& grid_res)
{
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_3D, texture);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, grid_res[0], grid_res[1], grid_res[2], 0, GL_RED, GL_FLOAT, sdf_values.data());
    glBindTexture(GL_TEXTURE_3D, 0);
    return texture;
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

    shape1 = Shapes::CreateSphere(1.0f, 64, 64, glm::vec3(0.6f, 0.2f, 0.9f)); //Shapes::CreateCylinder(1.0f, 2.0f, 64, glm::vec3(0.6f, 0.2f, 0.9f));//Shapes::CreateSphere(1.0f, 64, 64, glm::vec3(0.6f, 0.2f, 0.9f));
    shape2 = Shapes::CreateBox(1.0f,1.0f,2.0f, glm::vec3(0.2f,0.6f,0.9f));

    // render the cube
    glBindVertexArray(shape1.VAO);
    glDrawElements(GL_TRIANGLES, shape1.indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    ourShader = new Shader("shader.vs", "shader.fs");

    // Compute SDF grid
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    std::vector<float> sdf_values;
    Eigen::Vector3i grid_res(32, 32, 32);
    Eigen::Vector3d min_bound(-3.0, -3.0, -3.0); // Expanded for transformed mesh
    Eigen::Vector3d max_bound(3.0, 3.0, 3.0);

    glm::mat4 model2 = glm::mat4(1.0f);
    model2 = glm::translate(model2, glm::vec3(5.5f, 0.5f, 1.0f));

    ComputeSDFGrid(shape2, model2, V, F, sdf_values, grid_res, min_bound, max_bound);

    // Create SDF texture
    GLuint sdf_texture = CreateSDFTexture(sdf_values, grid_res);
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
    // render
        // ------
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

     // be sure to activate shader when setting uniforms/drawing objects
    ourShader->use();

    ourShader->setVec3("light.direction", -0.2f, -1.0f, -0.3f);
    ourShader->setVec3("viewPos", camera.Position);

    // light properties
    ourShader->setVec3("light.ambient", 0.2f, 0.2f, 0.2f);
    ourShader->setVec3("light.diffuse", 0.5f, 0.5f, 0.5f);
    ourShader->setVec3("light.specular", 1.0f, 1.0f, 1.0f);

    // view/projection transformations
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
    glm::mat4 view = camera.GetViewMatrix();
    ourShader->setMat4("projection", projection);
    ourShader->setMat4("view", view);
    ourShader->setFloat("Multi", 1.0f);

    // render the cube
    glm::mat4 model3 = glm::mat4(1.0f);
    model3 = glm::translate(model3, glm::vec3(0.0f, 0.0f, 0.0f));
    ourShader->setMat4("model", model3);
    ourShader->setFloat("Multi", 1.0f);

    glBindVertexArray(shape1.VAO);
    glDrawElements(GL_TRIANGLES, shape1.indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    // world transformation
    glm::mat4 model2 = glm::mat4(1.0f);
    model2 = glm::translate(model2,glm::vec3(5.5f,0.5f,1.0f));
    ourShader->setMat4("model", model2);
    ourShader->setFloat("Multi", 1.0f);

    glBindVertexArray(shape2.VAO);
    glDrawElements(GL_TRIANGLES, shape2.indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    // -------------------------------------------------------------------------------
    glfwSwapBuffers(window);
    glfwPollEvents();
}

void ApplicationWindow::Shutdown()
{
    glDeleteVertexArrays(1, &shape1.VAO);
    glDeleteVertexArrays(1, &shape2.VAO);
    glfwTerminate();
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