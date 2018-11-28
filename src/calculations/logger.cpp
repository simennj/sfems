#include "logger.hpp"

void Logger::logPrediction(const Eigen::VectorXd &displacement, double loadingParameter,
                           const Eigen::VectorXd &deltaDisplacement,
                           double deltaLoadingParameter) {
    predictorPoints << displacement(relevantDegreeOfFreedom) << " " << loadingParameter << " "
                    << deltaDisplacement(relevantDegreeOfFreedom) << " " << deltaLoadingParameter << std::endl;
}

void Logger::logCorrection(const Eigen::VectorXd &displacement, double loadingParameter,
                           const Eigen::VectorXd &deltaDisplacement,
                           double deltaLoadingParameter) {
    correctorPoints << displacement(relevantDegreeOfFreedom) << " " << loadingParameter << " "
                    << deltaDisplacement(relevantDegreeOfFreedom) << " " << deltaLoadingParameter << std::endl;
}

void Logger::logPoint(const Eigen::VectorXd &displacement, double loadingParameter) {
    finalPoints << displacement(relevantDegreeOfFreedom) << " " << loadingParameter << std::endl;
}