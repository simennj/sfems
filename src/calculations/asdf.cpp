#include "asdf.hpp"

Eigen::VectorXd solve(Eigen::MatrixXd A, Eigen::VectorXd b) {
//    return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(x);
    return A.colPivHouseholderQr().solve(b);
}
