#include <vector>
#include <glad/glad.h>
#include "curve.hpp"
#include "ui.hpp"

// (re)loads the initialized graphics parts
void graphics_reload() {
    ui_reload();
    curve_reload();
}

void graphics_init(void *(*loadProc)(const char)) {
    // Initialize glad, use glfw to retrieve GL function pointers
    gladLoadGLLoader((GLADloadproc) loadProc);

    // Enable transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Inits the graphics parts
    ui_init();
    curve_init();

    graphics_reload();
}


void graphics_draw(const std::vector<GLfloat> &vertices) {
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Draws the graphics parts
    ui_draw();
//    marker_draw(boatPosition, targetPosition);
//    graph_draw(boatPosition, minorGraphValues);
    curve_draw(vertices);

    // Unbind any bound vertex arrays to prevent bugs
    glBindVertexArray(0);
}
