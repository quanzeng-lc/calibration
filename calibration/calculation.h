#pragma once
#include "Eigen/Eigen"

class calculation
{
public:
    calculation();
    Eigen::Matrix3d calculateRbo(Eigen::MatrixXd nidData, int numOffset);
};
