#include <vector>
#include <glad/glad.h>
#include "curve.hpp"
#include "shaderUtils.hpp"
extern "C" {
#include "../utils/clamp.h"
}
// Define amount of coordinates per vertex
#define VERTEX_COORDINATE_COUNT 2

// Define constants used for amount, position, and dimensions of graphs
#define TOTAL_GRAPH_WIDTH 1.99f
#define MAIN_GRAPH_HEIGHT (0.5f)

// OpenGL identifiers
GLuint curveShaderProgram;
GLuint curveVertexArray;
GLuint curveVertexBuffer;

/*
 * Various generation, binding, etc for Opengl
 */
void curve_init() {
    curveShaderProgram = glCreateProgram();
    glGenVertexArrays(1, &curveVertexArray);
    glBindVertexArray(curveVertexArray);
    glGenBuffers(1, &curveVertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, curveVertexBuffer);
    glVertexAttribPointer(0, VERTEX_COORDINATE_COUNT, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
}

/*
 * Updates the graph with the new values
 */
void curve_update(const std::vector<GLfloat> &vertices) {
    glBindBuffer(GL_ARRAY_BUFFER, curveVertexBuffer);

    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size() * VERTEX_COORDINATE_COUNT, &vertices.front(),
                 GL_DYNAMIC_DRAW);
}

/*
 * Updates and draws the graph with the new values
 */
void curve_draw(const std::vector<GLfloat> &vertices) {
    curve_update(vertices);

    glUseProgram(curveShaderProgram);
    glBindVertexArray(curveVertexArray);

    // Sets the color to be the first of the colors defined in the shader
    int coordinates_count = static_cast<int>(vertices.size());
    int vertices_count = coordinates_count/VERTEX_COORDINATE_COUNT;
    glUniform1i(2, 0);
    glDrawArrays(GL_LINE_STRIP, 0, vertices_count);
    glUniform1i(2, 1);
    glPointSize(5);
    glDrawArrays(GL_POINTS, 0, vertices_count);
}

/*
 * Reload graph shader program from file
 */
void curve_reload() { shader_programInit(curveShaderProgram, "graph"); }
