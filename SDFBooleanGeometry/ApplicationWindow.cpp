#include "ApplicationWindow.h"

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>

#include <iostream>
#include <vector>

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