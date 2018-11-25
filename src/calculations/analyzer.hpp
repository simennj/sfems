#ifndef SFEMS_ANALYZER_HPP
#define SFEMS_ANALYZER_HPP


#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include "BeamElement.hpp"
#include "iterator.hpp"

struct BoundaryCondition {
    int globalDegreeOfFreedom;
    double value;
};

enum class ForceType {GLOBAL, LOCAL};

struct Force {
    ForceType forceType;
    int node;
    int degreeOfFreedom;
    double magnitude;
};

class Analyzer {

    std::vector<double> vertices;
    ElementProperties properties;

    Eigen::MatrixXd calculateGlobalStiffnessMatrix();

protected:
    unsigned long long int degreesOfFreedom;
    Eigen::VectorXd displacement;
    Eigen::VectorXd localForces;
    Eigen::VectorXd globalForces;
    std::unique_ptr<Iterator> iterator;

    std::vector<BeamElement> elements{};
    double forceScale = 0;
    const std::vector<BoundaryCondition> boundaryConditions;
public:
    double getForceScale() const;

    void setForceScale(double forceScale);

    void applyBoundaryConditions();

protected:
    Eigen::MatrixXd globalStiffness;

    explicit Analyzer(std::vector<double> vertices, ElementProperties properties,
                          const std::vector<BoundaryCondition> &boundaryConditions, const std::vector<Force> &forces,
                          std::unique_ptr<Iterator> iterator) :
            vertices(vertices),
            properties(properties),
            degreesOfFreedom((vertices.size() * 3) / 2),
            displacement(Eigen::VectorXd::Zero(degreesOfFreedom)),
            localForces(Eigen::VectorXd::Zero(degreesOfFreedom)),
            globalForces(Eigen::VectorXd::Zero(degreesOfFreedom)),
            boundaryConditions(boundaryConditions),
            iterator(std::move(iterator)) {
        elements.reserve(vertices.size() / 2);
        for (int i = 0; i < vertices.size() - 2; i += 2) {
            elements.push_back(BeamElement{
                    Eigen::Vector2d{vertices[i], vertices[i + 1]},
                    Eigen::Vector2d{vertices[i + 2], vertices[i + 3]},
                    properties,
                    static_cast<unsigned int>(i / 2)
            });
        }
        globalStiffness = Eigen::MatrixXd::Zero(degreesOfFreedom, degreesOfFreedom);
        calculateGlobalStiffnessMatrix();
        applyBoundaryConditions();
        for (auto boundaryCondition : boundaryConditions) {
            auto degreeOfFreedom = boundaryCondition.globalDegreeOfFreedom < 0 ?
                                   globalStiffness.rows() + boundaryCondition.globalDegreeOfFreedom :
                                   boundaryCondition.globalDegreeOfFreedom;

            globalStiffness.row(degreeOfFreedom).setConstant(boundaryCondition.value);
            globalStiffness.col(degreeOfFreedom).setConstant(boundaryCondition.value);
        }
        setForces(forces);
    }

public:
    std::vector<double> getVertices();
//    std::vector<double> getCurve(int elementSegments);

    virtual double update() = 0;
    virtual bool iterate(double deltaForce) = 0;

    virtual void setForces(std::vector<Force> forces);

    Eigen::VectorXd getScaledLocalForces() const;

    Eigen::VectorXd getScaledGlobalForces() const;

    Eigen::VectorXd getScaledLocalForces(double scalingFactor);

    Eigen::VectorXd getScaledGlobalForces(double scalingFactor);
};


#endif //SFEMS_ANALYZER_HPP
