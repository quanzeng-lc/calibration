#include "calculation.h"

calculation::calculation()
{

}


Eigen::Matrix3d calculation::calculateRbo(Eigen::MatrixXd ndiData, int numOffset)
{
    Eigen::Vector3d init = ndiData(0, Eigen::seq(4,6));

    Eigen::Matrix3d Rbo = Eigen::Matrix3d::Zero();
    Eigen::Vector3d Rbo_x = Eigen::Vector3d::Zero();
    Eigen::Vector3d Rbo_y = Eigen::Vector3d::Zero();
    Eigen::Vector3d Rbo_z = Eigen::Vector3d::Zero();

    for (int offset = 1; offset <= numOffset; offset++) {
        Eigen::Vector3d p = ndiData(offset, Eigen::seq(4, 6));
        Eigen::Vector3d diff = (p - init);
        double norm = std::sqrt(pow(diff(0), 2) + pow(diff(1), 2) + pow(diff(2), 2));
        Rbo_x += (diff / norm);
    }
    Rbo_x = Rbo_x / numOffset;

    for (int offset = 1; offset <= numOffset; offset++) {
        Eigen::Vector3d p = ndiData(numOffset + offset, Eigen::seq(4, 6));
        Eigen::Vector3d diff = p - init;
        double norm = std::sqrt(pow(diff(0), 2) + pow(diff(1), 2) + pow(diff(2), 2));
        Rbo_y += (diff / norm);
    }
    Rbo_y = Rbo_y / numOffset;

    for (int offset = 1; offset <= numOffset; offset++) {
        Eigen::Vector3d p = ndiData(2 * numOffset + offset, Eigen::seq(4,6));
        Eigen::Vector3d diff = p - init;
        double norm = std::sqrt(pow(diff(0), 2) + pow(diff(1), 2) + pow(diff(2), 2));
        Rbo_z += (diff / norm);
    }
    Rbo_z = Rbo_z / numOffset;

    Rbo = { {Rbo_x(0),Rbo_y(0),Rbo_z(0)},
            {Rbo_x(1),Rbo_y(1),Rbo_z(2)},
            {Rbo_x(2),Rbo_y(2),Rbo_z(2)} };

    return Rbo;

}