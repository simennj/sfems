#include "BeamElement.hpp"
#include "linear.hpp"
#include "analyzer.hpp"

double Linear::update() {
    Eigen::VectorXd globalForces = getScaledGlobalForces();
    for (const auto &element : elements) {
        globalForces.block(element.index*3, 0, 6, 1)
                += element.localToGlobalRotationMatrix*localForces.block(element.index*3, 0, 6, 1);
    }
    displacement = globalStiffness.ldlt().solve(globalForces);
    return 0;
}


bool Linear::iterate(double residualNorm) {
    return 0;
}
