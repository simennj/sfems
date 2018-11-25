#include <GLFW/glfw3.h>
#include <stdio.h>
#include "window_util.h"

// Define window dimensions
#define WIDTH 800
#define HEIGHT 800
#define MARGIN 0

GLFWwindow *window;

// Create error callback to make debugging easier
void error_callback(int code, const char *description) {
    printf("%i: ", code);
    printf(description);
    printf("\n");
}

// Resize viewport with window while keeping aspect ratio
void resize_callback(GLFWwindow *window, int width, int height) {
    glViewport(MARGIN, MARGIN + (height - width)/2, width - MARGIN*2, width - MARGIN*2);
//    if (height >= width) glViewport(MARGIN, MARGIN + (height - width) / 2, width - MARGIN*2, width - MARGIN*2);
//    else glViewport(MARGIN + (width - height) / 2, MARGIN, height - MARGIN*2, height - MARGIN*2);
}

void window_init(void (*key_callback)(GLFWwindow *window, int key, int scancode, int action, int mode)) {
    // Register error callback as quickly as possible to catch any errors from the start
    glfwSetErrorCallback(error_callback);

    glfwInit();
    // Set OpenGL version and profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create a window object
    window = glfwCreateWindow(WIDTH, HEIGHT, "Beams", NULL, NULL);
    glfwMakeContextCurrent(window);

    glfwSetWindowSizeCallback(window, resize_callback);
    glfwSetKeyCallback(window, key_callback);

    // Define the viewport dimensions
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    resize_callback(window, width, height);
}

// Update window and handle input
void window_update() {
    glfwPollEvents();
    glfwSwapBuffers(window);
}

void window_update_wait() {
    glfwWaitEvents();
    glfwSwapBuffers(window);
}

// Return whether the window is still open or not
int window_open() {
    return !glfwWindowShouldClose(window);
}
