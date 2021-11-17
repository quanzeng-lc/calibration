#pragma once
#include "Eigen/Eigen"

class calculation
{
public:
    calculation();
    Eigen::Matrix3d calculateRbo(Eigen::MatrixXd ndiData, int numOffset);
    Eigen::Matrix3d calculateOnceReo(Eigen::MatrixXd ndiData, int numOffset);
    Eigen::MatrixXd calculateReo(Eigen::MatrixXd ndiData, int numInit, int numOffset);
};
