#version 430 core

layout(location = 2) uniform int colorIndex;
uniform vec4 colors[5] = vec4[5](
    vec4(1.0f, 0.0f, 0.0f, 1.0f),
    vec4(0.0f, 1.0f, 0.0f, 1.0f),
    vec4(0.0f, 0.0f, 1.0f, 1.0f),
    vec4(1.0f, 0.0f, 1.0f, 1.0f),
    vec4(0.0f, 1.0f, 1.0f, 1.0f)
);
out vec4 color;
void main() {
    color = colors[colorIndex];
}