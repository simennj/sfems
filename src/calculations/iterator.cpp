#include <iostream>
#include "iterator.hpp"

bool SimpleIterator::iterate(double residual) {
    return currentIteration++ < maxIterations;
}

bool NewtonIterator::iterate(double residual) {
    std::cout << currentIteration << "/" << maxIterations << std::endl;
    std::cout << residual << ", " << " > " << tolerance << ": " << (residual > tolerance) << std::endl;
    return currentIteration++ < maxIterations && residual > tolerance;
}

void Iterator::resetIterations() {
    currentIteration = 0;
}

bool Iterator::reachedMaxIterations() {
    return currentIteration >= maxIterations;
}
