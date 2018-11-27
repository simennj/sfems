#include <iostream>
#include "structure.hpp"

std::vector<double> Structure::getVertices() {
    auto displacedVertices = vertices;
    for (int degreeOfFreedom = 0, vertexIndex = 0; degreeOfFreedom < displacement.size(); ++degreeOfFreedom) {
        if ((degreeOfFreedom+1) % 3 == 0) continue;
        displacedVertices[vertexIndex++] += displacement[degreeOfFreedom];
    }
    return displacedVertices;
}

Eigen::MatrixXd Structure::calculateGlobalStiffnessMatrix() const {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(degreesOfFreedom, degreesOfFreedom);
    for (auto &element : elements) {
        matrix.block<6,6>(element.index*3, element.index*3) += element.calculateTotalStiffness();
    }
    return applyBoundaryConditions(matrix);
}

Eigen::MatrixXd Structure::applyBoundaryConditions(Eigen::MatrixXd &stiffness) const {
    for (auto bc : boundaryConditions) {
        stiffness.row(bc.globalDegreeOfFreedom).setConstant(bc.value);
        stiffness.col(bc.globalDegreeOfFreedom).setConstant(bc.value);
        stiffness(bc.globalDegreeOfFreedom, bc.globalDegreeOfFreedom) = 1;
    }
    return stiffness;
}

Eigen::VectorXd Structure::getNominalLoad() {
    Eigen::VectorXd load = nominalGlobalLoad;
    for (auto &element : elements) {
        const auto i = element.index * 3;
        load.block<6, 1>(i, 0) +=
                element.localToGlobalRotationMatrix * nominalLocalLoad.block<6, 1>(i, 0);
    }
    return load;
}

Eigen::VectorXd solve(const Eigen::MatrixXd &A, const Eigen::VectorXd &b) {
//    return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    return A.colPivHouseholderQr().solve(b);
}

bool Structure::newton(double stepSize, double tolerance, int maxIterations) {
    Eigen::VectorXd outerForces = getNominalLoad() * stepSize;

    Eigen::VectorXd deltaDisplacement = solve(globalStiffness, outerForces);

    logger.logPrediction(displacement, loadingParameter, deltaDisplacement, stepSize);

    displacement += deltaDisplacement;

    loadingParameter += stepSize;

    update();

    outerForces = getNominalLoad();

    return newtonIterations(tolerance, maxIterations);
}

bool Structure::newtonIterations(double tolerance, int maxIterations) {
    Eigen::VectorXd load = getNominalLoad()*loadingParameter;
    Eigen::VectorXd residual = load - innerForces;
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        update();

        Eigen::VectorXd deltaDisplacement = globalStiffness.ldlt().solve(residual);
        logger.logCorrection(displacement, loadingParameter, deltaDisplacement, 0);
        displacement += deltaDisplacement;

        update();

        residual = load - innerForces;
        if (residual.norm() < tolerance) {
            logger.logPoint(displacement, loadingParameter);
            return false;
        }
    }
    logger.logPoint(displacement, loadingParameter);
    return true;
}

bool Structure::arcLength(double stepSize, double tolerance, int maxIterations) {
    Eigen::VectorXd w_q0 = solve(globalStiffness, getNominalLoad());

    double f = std::sqrt(1.0 + w_q0.transpose()*w_q0);

    double deltaLambda;
    if (w_q0.transpose() * lastDeltaDisplacement < 0 && !firstIteration) {
        deltaLambda = -stepSize / f;
    } else {
        deltaLambda = stepSize / f;
    }
    firstIteration = false;

    lastDeltaDisplacement = deltaLambda * w_q0;

    logger.logPrediction(displacement, loadingParameter, lastDeltaDisplacement, deltaLambda);

    loadingParameter += deltaLambda;
    displacement += lastDeltaDisplacement;

    for (int iterator = 0; iterator < maxIterations; ++iterator) {
        update();

        Eigen::VectorXd w_q = solve(globalStiffness, getNominalLoad());

        Eigen::VectorXd residual = getNominalLoad()*loadingParameter - innerForces;

        Eigen::VectorXd w_r  = solve(globalStiffness, residual);

        double w_qW_r = w_q.transpose()*w_r;
        double dLambda = -(w_qW_r)/(1+w_q.transpose()*w_q);
        Eigen::VectorXd dDisplacement = w_r + dLambda*w_q;

        logger.logCorrection(displacement, loadingParameter, dDisplacement, dLambda);

        displacement += dDisplacement;
        loadingParameter += dLambda;

        if (residual.norm() < tolerance) {
            logger.logPoint(displacement, loadingParameter);
            return false;
        }
    }
    logger.logPoint(displacement, loadingParameter);
    return true;
}

Eigen::VectorXd Structure::calculateInnerForces() {
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

void Structure::update() {
    for (auto &element : elements) {
        element.updateDeformation(displacement.block<6, 1>(element.index * 3, 0));
    }
    globalStiffness = calculateGlobalStiffnessMatrix();
    innerForces = calculateInnerForces();
}

