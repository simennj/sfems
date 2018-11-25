#ifndef SFEMS_COROTATIONAL_HPP
#define SFEMS_COROTATIONAL_HPP


#include "analyzer.hpp"
#include <fstream>
#include <iomanip>

class Corotational : public Analyzer {
    Eigen::VectorXd v_0;
    double lambda = 0;

    int i = 0;

    std::ofstream predictorPoints;
    std::ofstream correctorPoints;
    std::ofstream finalPoints;
    std::ofstream allPoints;

public:

    explicit Corotational(
            const std::vector<double> &vertices,
            const ElementProperties &properties,
            const std::vector<BoundaryCondition> &boundaryConditions,
            const std::vector<Force> &forces,
            std::unique_ptr<Iterator> iterator
    ) : Analyzer(
            vertices,
            properties,
            boundaryConditions,
            forces,
            std::move(iterator)
    ),
        v_0(Eigen::VectorXd::Zero(degreesOfFreedom)),
        predictorPoints("predictorPoints.dat"),
        correctorPoints("correctorPoints.dat"),
        finalPoints("finalPoints.dat"),
        allPoints("allPoints.dat")
        {
//        printf(fp,"%12.3e  %12.3e\n")
        predictorPoints << std::scientific;
        predictorPoints << std::setprecision(10);
        correctorPoints << std::scientific;
        correctorPoints << std::setprecision(10);
        finalPoints << std::scientific;
        finalPoints << std::setprecision(10);
        finalPoints << "0 0" << std::endl;
        allPoints << std::scientific;
        allPoints << std::setprecision(10);

        globalStiffness = calculateStiffness();
    }

    double update() override;

    bool iterate(double deltaForce) override;
    bool iterateArcLength(double deltaForce);

    Eigen::MatrixXd calculateStiffness() const;

    Eigen::VectorXd calculateResidual(const Eigen::VectorXd &outerForces, const Eigen::VectorXd &innerForces) const;

    bool newtonIterations(const Eigen::VectorXd &outerForces, Eigen::VectorXd &innerForces);

    Eigen::VectorXd calculateInnerForces() ;

    Eigen::VectorXd getOuterForces(double scalingFactor);
};


#endif //SFEMS_COROTATIONAL_HPP
