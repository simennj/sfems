#ifndef SFEMS_BEAMELEMENT_HPP
#define SFEMS_BEAMELEMENT_HPP

#include <Eigen/Dense>

struct ElementProperties {
    double youngsModulus, crossSectionArea, momentOfIntertia;
};

class BeamElement {
private:
    const Eigen::Vector2d beamVector;
    const double beamlength;
    double lengthDeformation = 0;
    const Eigen::Vector2d beamUnitVector;
    const Eigen::Vector2d beamUnitNormalVector;
    Eigen::Vector2d deformedBeamUnitTangent;
    Eigen::Vector2d nodeOneVector = Eigen::Vector2d::Zero();
    Eigen::Vector2d nodeTwoVector = Eigen::Vector2d::Zero();

    Eigen::Matrix<double, 6, 1> innerForces;

    const Eigen::Matrix<double, 6, 6> localStiffness;
    const Eigen::Matrix<double, 6, 6>
    calculateLocalStiffness(double L, double E, double A, double I) const;
    Eigen::Vector2d calculateNodeUnitVector(double nodeAngle) const;
    Eigen::Matrix<double, 6, 6> calculateLocalToGlobalRotationMatrix() const;
public:
    const unsigned int index;
    Eigen::Matrix<double, 6, 6> localToGlobalRotationMatrix;

    BeamElement (const Eigen::Vector2d &firstCoordinate, const Eigen::Vector2d &secondCoordinate,
                             const ElementProperties &properties, const unsigned int index) :
            beamVector(secondCoordinate - firstCoordinate),
            beamlength(std::sqrt(beamVector.transpose() * beamVector)),
            index(index),
            beamUnitVector(beamVector/beamlength),
            deformedBeamUnitTangent(beamUnitVector),
            beamUnitNormalVector(-beamUnitVector(1), beamUnitVector(0)),
            localStiffness(calculateLocalStiffness(beamlength, properties.youngsModulus, properties.crossSectionArea, properties.momentOfIntertia)),
            localToGlobalRotationMatrix(calculateLocalToGlobalRotationMatrix())
            {}

    void updateDeformation(const Eigen::Matrix<double, 6, 1> &displacement);
    Eigen::Matrix<double, 6, 1> calculateInnerForces() ;

    Eigen::Matrix<double, 6, 6> calculateMaterialStiffness() const;
    Eigen::Matrix<double, 6, 6> calculateGeometricStiffness() const;
    Eigen::Matrix<double, 6, 6> calculateTotalStiffness() const;

};

#endif //SFEMS_BEAMELEMENT_HPP
