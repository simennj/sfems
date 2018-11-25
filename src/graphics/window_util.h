#ifndef TMR4160_UI_H
#define TMR4160_UI_H

void window_init(void (*key_callback)(GLFWwindow *window, int key, int scancode, int action, int mode));

void window_update();
void window_update_wait();

int window_open();

#endif //TMR4160_UI_H
