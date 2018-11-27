#include <iostream>
#include "BeamElement.hpp"

Eigen::Matrix<double, 6, 6> BeamElement::calculateLocalToGlobalRotationMatrix() const {
    Eigen::Matrix<double, 6, 6> localToGlobalTransformation;
    localToGlobalTransformation <<
     deformedBeamUnitTangent[0], -deformedBeamUnitTangent[1],  0,               0,                           0,              0,
     deformedBeamUnitTangent[1],  deformedBeamUnitTangent[0],  0,               0,                           0,              0,
                  0,                          0,               1,               0,                           0,              0,
                  0,                          0,               0,   deformedBeamUnitTangent[0], -deformedBeamUnitTangent[1], 0,
                  0,                          0,               0,   deformedBeamUnitTangent[1],  deformedBeamUnitTangent[0], 0,
                  0,                          0,               0,               0,                           0,              1;
    return localToGlobalTransformation;
}

const Eigen::Matrix<double, 6, 6>
BeamElement::calculateLocalStiffness(const double L, const double E, const double A, const double I) const {
    Eigen::Matrix<double, 6, 6> localStiffness;
    localStiffness <<
    E*A/L,       0,            0,            -E*A/L,      0,               0,
      0,   12*E*I/pow(L,3),   6*E*I/pow(L,2),  0,   -12*E*I/pow(L,3),  6*E*I/pow(L,2),
      0,   6*E*I/pow(L,2),    4*E*I/L,         0,   -6*E*I/pow(L,2),   2*E*I/L,
    -E*A/L,      0,            0,             E*A/L,      0,               0,
      0,   -12*E*I/pow(L,3), -6*E*I/pow(L,2),  0,   12*E*I/pow(L,3),  -6*E*I/pow(L,2),
      0,   6*E*I/pow(L,2),    2*E*I/L,         0,   -6*E*I/pow(L,2),   4*E*I/L;
    return localStiffness;
}

Eigen::Vector2d BeamElement::calculateNodeUnitVector(double nodeAngle) const {
    Eigen::Matrix2d rotationMatrixFromInitialToDeformedCoordinateSystem{};
    rotationMatrixFromInitialToDeformedCoordinateSystem <<
            std::cos(nodeAngle), -std::sin(nodeAngle),
            std::sin(nodeAngle),  std::cos(nodeAngle)
    ;
    return rotationMatrixFromInitialToDeformedCoordinateSystem*beamUnitVector;
}

Eigen::Matrix<double, 6, 6> BeamElement::calculateMaterialStiffness() const {
    Eigen::Matrix<double, 6, 6> globalStiffness {
            localToGlobalRotationMatrix * localStiffness * localToGlobalRotationMatrix.transpose()
    };
    return globalStiffness;
}

Eigen::Matrix<double, 6, 1> BeamElement::calculateInnerForces() {
    Eigen::Vector2d deformationUnitNormal;
    deformationUnitNormal << -deformedBeamUnitTangent(1), deformedBeamUnitTangent(0);

    auto theta1 = std::asin(nodeOneVector.transpose()*deformationUnitNormal);
    auto theta2 = std::asin(nodeTwoVector.transpose()*deformationUnitNormal);

    Eigen::Matrix<double, 6, 1> localDeformation;
    localDeformation << -lengthDeformation/2, 0, theta1, lengthDeformation/2, 0, theta2;
    innerForces = localStiffness * localDeformation;
    Eigen::Matrix<double, 6, 1> forces = localToGlobalRotationMatrix * innerForces;
    return forces;
}

void BeamElement::updateDeformation(const Eigen::Matrix<double, 6, 1> &displacement) {
    auto deformationVector = Eigen::Vector2d{
            displacement(3) - displacement(0),
            displacement(4) - displacement(1)
    };
    auto deformedBeamVector = beamVector + deformationVector;
    auto deformedBeamLength = std::sqrt(deformedBeamVector.transpose() * deformedBeamVector);
    lengthDeformation = deformedBeamLength-beamLength;

    deformedBeamUnitTangent = deformedBeamVector/deformedBeamLength; // e_1

    nodeOneVector = calculateNodeUnitVector(displacement(2));
    nodeTwoVector = calculateNodeUnitVector(displacement(5));
    localToGlobalRotationMatrix = calculateLocalToGlobalRotationMatrix();
}

Eigen::Matrix<double, 6, 6> BeamElement::calculateGeometricStiffness() const {
    auto L = beamLength + lengthDeformation;
    auto N = innerForces(3);
    auto V = innerForces(4);
    Eigen::Matrix<double, 3, 3> quadrant;
    quadrant << 0,        -V/(2*L), 0,
            -V/(2*L),  N/L,     0,
            0,         0,       0;
    Eigen::Matrix<double, 6, 6> geometricStiffness;
    geometricStiffness <<  quadrant, -quadrant,
                          -quadrant,  quadrant;
    return localToGlobalRotationMatrix * geometricStiffness * localToGlobalRotationMatrix.transpose();
}

Eigen::Matrix<double, 6, 6> BeamElement::calculateTotalStiffness() const {
    return calculateMaterialStiffness() + calculateGeometricStiffness();
}
