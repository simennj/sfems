#include <iostream>
#include "logFunctions.hpp"

void logVertices(const std::vector<double> &vertices) {
    std::cout << "------------------" << std::endl;
    for (const auto vertex : vertices)
        std::cout << vertex << ", ";
    std::cout << std::endl;
}