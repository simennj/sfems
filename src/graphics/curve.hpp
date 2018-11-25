#ifndef SFEMS_CURVE_HPP
#define SFEMS_CURVE_HPP

#include <vector>
#include <glad/glad.h>

void curve_init();

void curve_draw(const std::vector<GLfloat> &vertices);

void curve_reload();

#endif //SFEMS_CURVE_HPP
