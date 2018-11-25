#include <glad/glad.h>
#include "ui.hpp"
#include "../utils/fileUtils.hpp"
#include "shaderUtils.hpp"

// Define amount of coordinates per vertex
#define VERTEX_COORDINATE_COUNT 3

// OpenGL identifiers
GLuint uiShaderProgram;
GLuint uiVertexBuffer;
GLuint uiVertexArray;
GLuint uiElementArray;

// Current amount of triangles to draw
int uiTrianglesCount;

// Fill buffers for drawing polygons with vertex and indic values
void bindPolygons(GLfloat *verts, GLint vertCount, GLint *indic, GLint indicCount) {
    glBindVertexArray(uiVertexArray);

    glBindBuffer(GL_ARRAY_BUFFER, uiVertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertCount * VERTEX_COORDINATE_COUNT, verts, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, uiElementArray);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLint) * indicCount * 3, indic, GL_STATIC_DRAW);

    glVertexAttribPointer(0, VERTEX_COORDINATE_COUNT, GL_FLOAT, GL_FALSE, VERTEX_COORDINATE_COUNT * sizeof(GLfloat),
                          (GLvoid *) 0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0); // Unbind VAO
}

/*
 * Various generation, binding, etc for Opengl
 */
void ui_init() {
    uiShaderProgram = glCreateProgram();
    glGenVertexArrays(1, &uiVertexArray);
    glGenBuffers(1, &uiVertexBuffer);
    glGenBuffers(1, &uiElementArray);
}

void ui_draw() {
    glUseProgram(uiShaderProgram);
    glBindVertexArray(uiVertexArray);
    glDrawElements(GL_TRIANGLES, uiTrianglesCount * VERTEX_COORDINATE_COUNT, GL_UNSIGNED_INT, 0);
}

/*
 * Reload graph shader program, vertices and indices from files
 */
void ui_reload() {
    GLint vertCount = getVectorCountFromFile("ui/vertices.txt");
    GLint indicCount = getVectorCountFromFile("ui/indices.txt");
    GLfloat verts[vertCount * VERTEX_COORDINATE_COUNT];
    GLint indic[indicCount * 3]; // 3 vertices per triangle
    getFloatVectorFromFile("ui/vertices.txt", vertCount, verts);
    getIntVectorFromFile("ui/indices.txt", indicCount, indic);
    bindPolygons(verts, vertCount, indic, indicCount);
    uiTrianglesCount = indicCount;

    shader_programInit(uiShaderProgram, "ui");
}