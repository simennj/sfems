#include <glad/glad.h>
#include <stdio.h>
#include "../utils/fileUtils.hpp"

const GLchar *vertexShaderFile = "vertexShader.glsl";
const GLchar *fragmentShaderFile = "fragmentShader.glsl";

/*
 * Load shader source from file into shader
 */
void loadFromFile(GLuint shader, const GLchar *shaderFile) {
    const GLchar *shaderSource = getShaderSource(shaderFile);
    glShaderSource(shader, 1, &shaderSource, NULL);
    glCompileShader(shader);
    // Free memory used for shader source after use
    free((void *) shaderSource);
    // Check for compile time errors
    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        printf("Shader error (%s):%s\n", shaderFile, infoLog);
    }
}

/*
 * Initializes the given shader program with shaders from the given folder
 */
void shader_programInit(GLuint shaderProgram, const GLchar *folder) {
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    int pathBufferSize = 50;
    char pathBuffer[pathBufferSize];
    // Load vertex shader
    loadFromFile(vertexShader, pathAppend(pathBuffer, pathBufferSize, folder, vertexShaderFile));
    // Load fragment shader
    loadFromFile(fragmentShader, pathAppend(pathBuffer, pathBufferSize, folder, fragmentShaderFile));

    // Link shaders
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    // Check for linking errors
    GLint success;
    GLchar infoLog[512];
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        printf("Shader linking error:%s\n", infoLog);
    }

    // Clean up afterwards
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDetachShader(shaderProgram, vertexShader);
    glDetachShader(shaderProgram, fragmentShader);
}