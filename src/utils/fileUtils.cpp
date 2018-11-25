#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fileUtils.hpp"

// Buffer used for retrieving lines when reading in vectors
// Is ten times as big as the longest line in current files
const int VECTOR_BUFFER_SIZE = 200;

/*
 * Helper function for appending paths.
 * Used to append a file name to a folder path in graphics.
 * result: buffer that the resulting path is put
 * resultSize: size of the buffer
 * path1: first path
 * path2: path to be appended to the first path
 * returns a pointer to the result buffer
 */
char *pathAppend(char *result, int resultSize, const char *path1, const char *path2) {
    // Make sure the buffer size is atleast 1
    if (resultSize <= 0) return result;
    // Make sure there is no overflow if the result buffer is too small
    result[0] = '\0';
    if (strlen(path1) + strlen(path2) + 2 > resultSize) return result;
    return strcat(strcat(strcpy(result, path1), "/"), path2);
}

/*
 * Count the vectors of a file by counting newlines (one vector per line)
 * Is used for preallocating a buffer before reading the vectors from it
 * filename: file to count vectors in
 * returns vector count
 */
int getVectorCountFromFile(const char *filename) {
    FILE *file = fopen(filename, "r");
    int count = 0;
    int bufferChar;
    char lastCharNewline = 1;
    // Iterate over all characters in the file
    for (bufferChar = getc(file); bufferChar != EOF; bufferChar = getc(file)) {
        // If character is newline
        if (bufferChar == '\n') {
            // Add one to vector count if the last read char is not also a newline
            count += !lastCharNewline;
            // Register that the current char is newline
            lastCharNewline = 1;
        } else {
            // Register that the current char is not newline
            lastCharNewline = 0;
        }
    }
    return count;
}

/*
 * Read float vectors from a file
 * filename: file to read vectors from
 * count: amount of vectors to read
 * array: array to put vectors in
 */
void getFloatVectorFromFile(const char *filename, int count, float *array) {
    FILE *file = fopen(filename, "r");
    char *rest;
    char line[VECTOR_BUFFER_SIZE];

    // Iterate i by 3 at a time since vectors are 3 floats
    for (int i = 0; i < count * 3; i += 3) {
        fgets(line, VECTOR_BUFFER_SIZE, file);
        // Everything after the first # is a comment, so ignore it
        rest = strtok(line, "#");
        // Read up to 3 floats from the line (missing floats are set to 0)
        for (int j = 0; j < 3; ++j) {
            array[i + j] = strtof(rest, &rest);
        }
    }
    fclose(file);
}

/*
 * Read int vectors from a file
 * filename: file to read vectors from
 * count: amount of vectors to read
 * array: array to put vectors in
 */
void getIntVectorFromFile(const char *filename, int count, int *array) {
    FILE *file = fopen(filename, "r");
    char *rest;
    char line[VECTOR_BUFFER_SIZE];

    // Iterate i by 3 at a time since vectors are 3 ints
    for (int i = 0; i < count * 3; i += 3) {
        fgets(line, VECTOR_BUFFER_SIZE, file);
        // Everything after the first # is a comment, so ignore it
        rest = strtok(line, "#");
        // Read up to 3 ints from the line (missing ints are set to 0)
        for (int j = 0; j < 3; ++j) {
            array[i + j] = (int) strtol(rest, &rest, 10);
        }
    }
    fclose(file);
}

/*
 * Read shader source code from a file
 * The returned char array has to be freed by the caller
 * filename: file to read shader source from
 * returns the content of the file
 */
char *getShaderSource(const char *filename) {
    FILE *f = fopen(filename, "rb");
    char *fragmentShaderSource = nullptr;
    size_t length;
    if (f) {
        // Find length of file before allocating buffer
        fseek(f, 0, SEEK_END);
        length = (size_t) ftell(f);
        fseek(f, 0, SEEK_SET);
        // Allocates buffer for holding the file content
        // Make space for an extra null char at the end
        fragmentShaderSource = static_cast<char *>(malloc(length + 1));
        if (fragmentShaderSource) {
            fread(fragmentShaderSource, 1, length, f);
            // Put an extra null character at the end to make sure there is no overflow
            fragmentShaderSource[length] = '\0';
        }
        fclose(f);
    }
    return fragmentShaderSource;
}

/*
 * Load constants from a file
 * filename: file to read constants from
 * k_p, k_i, k_d: PID constants output values
 * targetPosition: target position output value
 * motorCenter, motorRadius: motor constants output values
 * returns number of output values filled
 */
int loadConstants(
        const char *filename,
        double *k_p, double *k_i, double *k_d,
        double *targetPosition,
        double *motorCenter, double *motorRadius
) {
    FILE *file = fopen(filename, "r");
    int res = fscanf(file, "%lf %lf %lf %lf %lf %lf", k_p, k_i, k_d, targetPosition, motorCenter, motorRadius);
    fclose(file);
    return res;
}