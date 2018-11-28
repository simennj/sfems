#include <utility>

#ifndef SFEMS_ANALYZER_HPP
#define SFEMS_ANALYZER_HPP


#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include "BeamElement.hpp"
#include "logger.hpp"

struct BoundaryCondition {
    int globalDegreeOfFreedom;
    double value;
};

enum class ForceType {
    GLOBAL, LOCAL
};

struct Force {
    ForceType forceType;
    int node;
    int degreeOfFreedom;
    double magnitude;
};

class Structure {
    std::vector<double> vertices;
    Eigen::MatrixXd globalStiffness;

    bool firstIteration = true;
    Logger logger;
    const unsigned long long int degreesOfFreedom;
    Eigen::VectorXd nominalLocalLoad;
    Eigen::VectorXd nominalGlobalLoad;

    Eigen::VectorXd displacement;
    Eigen::VectorXd lastDeltaDisplacement;
    Eigen::VectorXd innerForces;

    std::vector<BeamElement> elements{};
    double loadingParameter = 0;
    const std::vector<BoundaryCondition> boundaryConditions;

    Eigen::MatrixXd applyBoundaryConditions(Eigen::MatrixXd &stiffness) const;

    Eigen::VectorXd getNominalLoad();

    bool newtonIterations(double tolerance, int maxIterations);

    Eigen::VectorXd calculateInnerForces();

    void update();

    Eigen::MatrixXd calculateGlobalStiffnessMatrix() const;

public:
    explicit Structure(
            std::vector<double> vertices,
            ElementProperties properties,
            std::vector<BoundaryCondition> boundaryConditions,
            const std::vector<Force> &forces,
            Logger logger
    ) :
            vertices(vertices),
            degreesOfFreedom((vertices.size() * 3) / 2),
            displacement(Eigen::VectorXd::Zero(degreesOfFreedom)),
            nominalLocalLoad(Eigen::VectorXd::Zero(degreesOfFreedom)),
            nominalGlobalLoad(Eigen::VectorXd::Zero(degreesOfFreedom)),
            boundaryConditions(std::move(boundaryConditions)),
            lastDeltaDisplacement(Eigen::VectorXd::Zero(degreesOfFreedom)),
            innerForces(Eigen::VectorXd::Zero(degreesOfFreedom)),
            logger(std::move(logger)) {
        elements.reserve(vertices.size() / 2);
        for (int i = 0; i < vertices.size() - 2; i += 2) {
            elements.push_back(BeamElement{
                    Eigen::Vector2d{vertices[i], vertices[i + 1]},
                    Eigen::Vector2d{vertices[i + 2], vertices[i + 3]},
                    properties,
                    static_cast<unsigned int>(i / 2)
            });
        }
        update();
        for (auto force : forces) {
            if (force.forceType == ForceType::GLOBAL)
                nominalGlobalLoad(force.node * 3 + force.degreeOfFreedom) = force.magnitude;
            else if (force.forceType == ForceType::LOCAL)
                nominalLocalLoad(force.node * 3 + force.degreeOfFreedom) = force.magnitude;
        }
    }

    std::vector<double> getVertices();

    bool newton(double stepSize, double tolerance, int maxIterations);

    bool arcLength(double stepSize, double tolerance, int maxIterations);

};


#endif //SFEMS_ANALYZER_HPP
