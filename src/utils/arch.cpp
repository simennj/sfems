#include <vector>
#include <cmath>
#include "arch.hpp"

std::vector<double> calculateArch(unsigned int element_count, CurveExpression *expression) {
    auto nodeCount = element_count + 1;
    std::vector<double> arch(nodeCount * 2);
    auto spacing = 1.0f / static_cast<double> ( element_count );
    for (int i = 0; i < nodeCount; ++i) {
        auto[x, y] = expression->getPoint(spacing * i);
        arch[i * 2] = x;
        arch[i * 2 + 1] = y;
    }
    return arch;
};

std::pair<double, double> CircleExpression::getPoint(double percent) {
    auto angle = totalAngle - percent*totalAngle+startAngle;
    return std::make_pair(
            radius * std::cos(angle),
            radius * std::sin(angle)-(radius-height/2)
    );
}

std::pair<double, double> LineExpression::getPoint(double percent) {
    return std::make_pair(percent * length - length / 2, percent * height - height / 2);
}
