#include <iostream>
#include <fstream>
#include "asdf.hpp"
#include "corotational.hpp"

double Corotational::update() {
    Eigen::VectorXd outerLocalForces = getScaledLocalForces();
    Eigen::VectorXd innerForces = Eigen::VectorXd::Zero(degreesOfFreedom);
    globalStiffness.setZero();
    Eigen::MatrixXd geometricStiffness = Eigen::MatrixXd::Zero(degreesOfFreedom, degreesOfFreedom);

    Eigen::VectorXd outerForces = getScaledGlobalForces();
    for (auto &element : elements) {
        const auto i = element.index * 3;
        element.updateDeformation(displacement.block<6, 1>(i, 0));
        outerForces.block<6, 1>(i, 0) +=
                element.localToGlobalRotationMatrix * outerLocalForces.block<6, 1>(i, 0);
        innerForces.block<6, 1>(i, 0) += element.calculateInnerForces();
        globalStiffness.block<6, 6>(i, i) += element.calculateMaterialStiffness();
        geometricStiffness.block<6, 6>(i, i) += element.calculateGeometricStiffness();
    }
    globalStiffness += geometricStiffness;
    applyBoundaryConditions();
    Eigen::VectorXd deltaDisplacement = (globalStiffness).ldlt().solve(outerForces - innerForces);
    displacement += deltaDisplacement;

    Eigen::VectorXd residual = outerForces - innerForces;
    for (auto boundaryCondition : boundaryConditions) {
        auto degreeOfFreedom = boundaryCondition.globalDegreeOfFreedom < 0 ?
                               globalStiffness.rows() + boundaryCondition.globalDegreeOfFreedom :
                               boundaryCondition.globalDegreeOfFreedom;
        residual.row(degreeOfFreedom).setConstant(boundaryCondition.value);
    }

    return residual.norm();
}

bool Corotational::iterate(double deltaForce) {
    return iterateArcLength(deltaForce);
    Eigen::VectorXd outerForces = getOuterForces(deltaForce);

    Eigen::MatrixXd stiffness = calculateStiffness();

    Eigen::VectorXd deltaDisplacement = stiffness.ldlt().solve(outerForces);

    displacement += deltaDisplacement;

    forceScale += deltaForce;

    for (auto &element : elements) {
        element.updateDeformation(displacement.block<6, 1>(element.index * 3, 0));
    }

    Eigen::VectorXd innerForces = calculateInnerForces();

    outerForces = getOuterForces(forceScale);

    return newtonIterations(outerForces, innerForces);
}

bool Corotational::newtonIterations(const Eigen::VectorXd &outerForces, Eigen::VectorXd &innerForces) {
    Eigen::VectorXd residual = calculateResidual(outerForces, innerForces);
    Eigen::VectorXd correction = Eigen::VectorXd::Zero(degreesOfFreedom);
    iterator->resetIterations();
    while (iterator->iterate(residual.norm())) {
        Eigen::MatrixXd stiffness = calculateStiffness();

        displacement += stiffness.ldlt().solve(residual);

        for (auto &element : elements) {
            element.updateDeformation(displacement.block<6, 1>(element.index * 3, 0));
        }

        innerForces = calculateInnerForces();

        residual = calculateResidual(outerForces, innerForces);
    }
    return iterator->reachedMaxIterations();
}

bool Corotational::iterateArcLength(double deltaS) {
    allPoints << displacement(degreesOfFreedom/2) << " " << lambda << std::endl;
    predictorPoints << displacement(degreesOfFreedom/2) << " " << lambda << " ";

    Eigen::VectorXd w_q0 = solve(globalStiffness, globalForces);

    long double f = std::sqrt(1.0 + w_q0.transpose()*w_q0);

    long double deltaLambda;
    std::cout << "wv:" << w_q0.transpose() * v_0 << std::endl;
    if (w_q0.transpose() * v_0 < 0 && i++ > 0) {
        deltaLambda = -deltaS / f;
    } else {
        deltaLambda = deltaS / f;
//        std::cout << std::endl << "turning" << std::endl << std::endl;
    }

    v_0 = deltaLambda * w_q0;

    lambda += deltaLambda;

    displacement += v_0;
    for (auto &element : elements) {
        element.updateDeformation(displacement.block<6, 1>(element.index * 3, 0));
    }
    Eigen::VectorXd innerForces = calculateInnerForces();
    globalStiffness = calculateStiffness();

    Eigen::VectorXd residual = calculateResidual(globalForces * lambda, innerForces);
    iterator->resetIterations();
    predictorPoints << v_0(degreesOfFreedom/2) << " " << deltaLambda << std::endl;
    allPoints << displacement(degreesOfFreedom/2) << " " << lambda << std::endl;

    while (true) {
        correctorPoints << displacement(degreesOfFreedom/2) << " " << lambda << " ";

        Eigen::VectorXd w_q = solve(globalStiffness, globalForces);

        residual = calculateResidual(globalForces*lambda, innerForces);

        Eigen::VectorXd w_r  = solve(globalStiffness, residual);

        long double w_qW_r = w_q.transpose()*w_r;
        long double dLambda = -(w_qW_r)/(1+w_q.transpose()*w_q);
        Eigen::VectorXd dDisplacement = w_r + dLambda*w_q;

        displacement += dDisplacement;
        lambda += dLambda;
        correctorPoints << dDisplacement(degreesOfFreedom/2) << " " << dLambda << std::endl;

        if (!iterator->iterate(residual.norm()))
            break;

        for (auto &element : elements) {
            element.updateDeformation(displacement.block<6, 1>(element.index * 3, 0));
        }
        innerForces = calculateInnerForces();
        globalStiffness = calculateStiffness();

        allPoints << displacement(degreesOfFreedom/2) << " " << lambda << std::endl;

    }
    finalPoints << displacement(degreesOfFreedom/2) << " " << lambda << std::endl;
    return iterator->reachedMaxIterations();
}

Eigen::VectorXd Corotational::getOuterForces(double scalingFactor) {
    Eigen::VectorXd outerLocalForces = getScaledLocalForces(scalingFactor);
    Eigen::VectorXd outerForces = getScaledGlobalForces(scalingFactor);
    for (auto &element : elements) {
        const auto i = element.index * 3;
        outerForces.block<6, 1>(i, 0) +=
                element.localToGlobalRotationMatrix * outerLocalForces.block<6, 1>(i, 0);
    }
    return outerForces;
}

Eigen::VectorXd Corotational::calculateInnerForces() {
    Eigen::VectorXd innerForces = Eigen::VectorXd::Zero(degreesOfFreedom);
    for (auto &element : elements) {
        const auto i = element.index * 3;
        innerForces.block<6, 1>(i, 0) += element.calculateInnerForces();
    }
    for (auto boundaryCondition : boundaryConditions) {
        auto degreeOfFreedom = boundaryCondition.globalDegreeOfFreedom < 0 ?
                               degreesOfFreedom + boundaryCondition.globalDegreeOfFreedom :
                               boundaryCondition.globalDegreeOfFreedom;
        innerForces.row(degreeOfFreedom).setConstant(boundaryCondition.value);
    }
    return innerForces;
}

Eigen::VectorXd Corotational::calculateResidual(const Eigen::VectorXd &outerForces, const Eigen::VectorXd &innerForces) const {
    Eigen::VectorXd residual = outerForces - innerForces;
    for (auto boundaryCondition : boundaryConditions) {
        auto degreeOfFreedom = boundaryCondition.globalDegreeOfFreedom < 0 ?
                               degreesOfFreedom + boundaryCondition.globalDegreeOfFreedom :
                               boundaryCondition.globalDegreeOfFreedom;
        residual.row(degreeOfFreedom).setConstant(boundaryCondition.value);
    }
    return residual;
}

Eigen::MatrixXd Corotational::calculateStiffness() const {
    Eigen::MatrixXd stiffness = Eigen::MatrixXd::Zero(degreesOfFreedom, degreesOfFreedom);
    for (auto &element : elements) {
        stiffness.block<6, 6>(element.index * 3, element.index * 3)
                += element.calculateTotalStiffness();
    }
    for (auto boundaryCondition : boundaryConditions) {
        auto degreeOfFreedom = boundaryCondition.globalDegreeOfFreedom < 0 ?
                               stiffness.rows() + boundaryCondition.globalDegreeOfFreedom :
                               boundaryCondition.globalDegreeOfFreedom;
        stiffness.row(degreeOfFreedom).setConstant(boundaryCondition.value);
        stiffness.col(degreeOfFreedom).setConstant(boundaryCondition.value);
        stiffness(degreeOfFreedom, degreeOfFreedom) = 1;
    }
    return stiffness;
}
