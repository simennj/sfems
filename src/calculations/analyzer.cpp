#include "corotational.hpp"
#include "linear.hpp"
#include <iostream>
#include "analyzer.hpp"

std::vector<double> Analyzer::getVertices() {
    auto displacedVertices = vertices;
    for (int degreeOfFreedom = 0, vertexIndex = 0; degreeOfFreedom < displacement.size(); ++degreeOfFreedom) {
        if ((degreeOfFreedom+1) % 3 == 0) continue;
        displacedVertices[vertexIndex++] += displacement[degreeOfFreedom];
    }
    return displacedVertices;
}

//std::vector<double> Analyzer::getCurve(int elementSegments) {
//    std::vector<double> curveVertices(vertices.size()*elementSegments);
//    for (int degreeOfFreedom = 0, vertexIndex = 0; degreeOfFreedom < degreesOfFreedom; ++degreeOfFreedom) {
//        if ((degreeOfFreedom+1) % 3 == 0) continue;
//        displacedVertices[vertexIndex++] += displacement[degreeOfFreedom];
//    }
//    return displacedVertices;
//}

Eigen::MatrixXd Analyzer::calculateGlobalStiffnessMatrix() {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(degreesOfFreedom, degreesOfFreedom);
    for (auto &element : elements) {
        matrix.block<6,6>(element.index*3, element.index*3) += element.calculateMaterialStiffness();
    }
    return matrix;
}

void Analyzer::applyBoundaryConditions() {
    for (auto boundaryCondition : boundaryConditions) {
        auto degreeOfFreedom = boundaryCondition.globalDegreeOfFreedom < 0 ?
                               globalStiffness.rows() + boundaryCondition.globalDegreeOfFreedom :
                               boundaryCondition.globalDegreeOfFreedom;
        globalStiffness.row(degreeOfFreedom).setConstant(boundaryCondition.value);
        globalStiffness.col(degreeOfFreedom).setConstant(boundaryCondition.value);
    }
}

void Analyzer::setForces(std::vector<Force> forces) {
    for (auto force : forces){
        if (force.forceType == ForceType::GLOBAL)
            globalForces(force.node*3+force.degreeOfFreedom, 0) = force.magnitude;
        else
            localForces(force.node*3+force.degreeOfFreedom, 0) = force.magnitude;
    }
}

double Analyzer::getForceScale() const {
    return forceScale;
}

void Analyzer::setForceScale(double forceScale) {
    Analyzer::forceScale = forceScale;
}

Eigen::VectorXd Analyzer::getScaledLocalForces() const {
    return localForces*forceScale;
}

Eigen::VectorXd Analyzer::getScaledGlobalForces() const {
    return globalForces*forceScale;
}

Eigen::VectorXd Analyzer::getScaledLocalForces(double scalingFactor) {
    return localForces*scalingFactor;
}

Eigen::VectorXd Analyzer::getScaledGlobalForces(double scalingFactor) {
    return globalForces*scalingFactor;
}
