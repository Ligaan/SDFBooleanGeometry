#include "glad/glad.h"
#include "glfw3.h"

#include "ApplicationWindow.h"

int main()
{
    ApplicationWindow window;
    window.Initialize();
    window.Update();
    window.Shutdown();
    return 0;
}