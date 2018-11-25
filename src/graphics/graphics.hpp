#ifndef TMR4160_GRAPHICS_H
#define TMR4160_GRAPHICS_H

#include <vector>

void graphics_init(void *(*loadProc)(const char));

void graphics_reload();

void graphics_draw(const std::vector<float> &vertices);

#endif //TMR4160_GRAPHICS_H
