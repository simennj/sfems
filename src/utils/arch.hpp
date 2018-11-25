#ifndef SFEMS_ARCH_HPP
#define SFEMS_ARCH_HPP

#include <vector>
#include <math.h>

class CurveExpression { // Memory leak?
public:
    virtual ~CurveExpression() = default;

    virtual std::pair<double, double> getPoint(double t) = 0;
};

class CircleExpression : public CurveExpression {
private:
    const double radius;
    const double totalAngle;
    const double startAngle;
    const double height;
public:
    explicit CircleExpression(double radius, double height) :
            radius(radius),
            height(height),
            totalAngle(2*acos((radius - height) / radius)),
            startAngle((M_PI-totalAngle)/2.0f) {}

    std::pair<double, double> getPoint(double percent) override;
};

class LineExpression : public CurveExpression {
private:
    const double length;
    const double height;
public:
    explicit LineExpression(const double length, const double height)
            : length(length), height(height)
    {}

    std::pair<double, double> getPoint(double percent) override;
};

std::vector<double> calculateArch(unsigned int element_count, CurveExpression *expression);

#endif //SFEMS_ARCH_HPP
