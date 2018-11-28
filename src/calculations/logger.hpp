#ifndef SFEMS_LOGGER_HPP
#define SFEMS_LOGGER_HPP


#include <fstream>
#include <iomanip>
#include <Eigen/Dense>

class Logger {
    std::ofstream predictorPoints;
    std::ofstream correctorPoints;
    std::ofstream finalPoints;
    const unsigned int relevantDegreeOfFreedom;

public:

    explicit Logger(unsigned int node, unsigned int degreeOfFreedom) :
            relevantDegreeOfFreedom(node * 3 + degreeOfFreedom),
            predictorPoints("predictorPoints.dat"),
            correctorPoints("correctorPoints.dat"),
            finalPoints("finalPoints.dat") {
        predictorPoints << std::scientific;
        predictorPoints << std::setprecision(10);
        correctorPoints << std::scientific;
        correctorPoints << std::setprecision(10);
        finalPoints << std::scientific;
        finalPoints << std::setprecision(10);
        finalPoints << "0 0" << std::endl;
    }

    void logPrediction(const Eigen::VectorXd &displacement, double loadingParameter,
                       const Eigen::VectorXd &deltaDisplacement,
                       double deltaLoadingParameter);

    void logCorrection(const Eigen::VectorXd &displacement, double loadingParameter,
                       const Eigen::VectorXd &deltaDisplacement, double deltaLoadingParameter);

    void logPoint(const Eigen::VectorXd &displacement, double loadingParameter);
};


#endif //SFEMS_LOGGER_HPP
