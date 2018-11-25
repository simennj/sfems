#ifndef SFEMS_LINEAR_HPP
#define SFEMS_LINEAR_HPP

#include <Eigen/Dense>
#include "analyzer.hpp"
#include "BeamElement.hpp"

class Linear : public Analyzer {
public:

    explicit Linear(
            const std::vector<double> &vertices,
            const ElementProperties &properties,
            const std::vector<BoundaryCondition> &boundaryConditions,
            const std::vector<Force> &forces,
            std::unique_ptr<Iterator> iterator
    ) : Analyzer(
            vertices, properties, boundaryConditions, forces, std::move(iterator)
    ) {}

    double update() override;

    bool iterate(double residualNorm) override;
};


#endif //SFEMS_LINEAR_HPP
