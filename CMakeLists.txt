cmake_minimum_required(VERSION 3.12)
project(sfems)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(glad/include)

add_executable(sfems
        src/main.cpp

        src/utils/clamp.c
        src/utils/fileUtils.cpp
        src/graphics/ui.cpp
        src/graphics/graphics.cpp
        src/graphics/window_util.c
        src/graphics/shaderUtils.cpp

        glad/src/glad.c
        src/graphics/curve.cpp src/graphics/curve.hpp src/utils/arch.cpp src/utils/arch.hpp src/calculations/structure.cpp src/calculations/BeamElement.cpp src/calculations/logger.cpp src/calculations/logger.hpp)

target_link_libraries (sfems
        Eigen3::Eigen
        yaml-cpp
        glfw
        ${OPENGL_LIBRARY}
        )
