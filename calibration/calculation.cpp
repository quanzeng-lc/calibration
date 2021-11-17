#include "calculation.h"

calculation::calculation()
{

}


Eigen::Matrix3d calculation::calculateRbo(Eigen::MatrixXd ndiData, int numOffset)
{
    Eigen::Vector3d init = ndiData(0, Eigen::seq(4,6));

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

    Eigen::Matrix3d Rbo({ {Rbo_x(0), Rbo_y(0), Rbo_z(0)},
                          {Rbo_x(1), Rbo_y(1), Rbo_z(1)},
                          {Rbo_x(2), Rbo_y(2), Rbo_z(2)} });

    return Rbo;
}

Eigen::Matrix3d calculation::calculateOnceReo(Eigen::MatrixXd ndiData, int numOffset)
{
    Eigen::Vector3d init = ndiData(0, Eigen::seq(4, 6));

    Eigen::Vector3d Reo_x = Eigen::Vector3d::Zero();
    Eigen::Vector3d Reo_y = Eigen::Vector3d::Zero();
    Eigen::Vector3d Reo_z = Eigen::Vector3d::Zero();

    for (int offset = 1; offset <= numOffset; offset++) {
        Eigen::Vector3d p = ndiData(offset, Eigen::seq(4, 6));
        Eigen::Vector3d diff = (p - init);
        double norm = std::sqrt(pow(diff(0), 2) + pow(diff(1), 2) + pow(diff(2), 2));
        Reo_x += (diff / norm);
    }
    Reo_x = Reo_x / numOffset;

    for (int offset = 1; offset <= numOffset; offset++) {
        Eigen::Vector3d p = ndiData(numOffset + offset, Eigen::seq(4, 6));
        Eigen::Vector3d diff = p - init;
        double norm = std::sqrt(pow(diff(0), 2) + pow(diff(1), 2) + pow(diff(2), 2));
        Reo_y += (diff / norm);
    }
    Reo_y = Reo_y / numOffset;

    for (int offset = 1; offset <= numOffset; offset++) {
        Eigen::Vector3d p = ndiData(2 * numOffset + offset, Eigen::seq(4, 6));
        Eigen::Vector3d diff = p - init;
        double norm = std::sqrt(pow(diff(0), 2) + pow(diff(1), 2) + pow(diff(2), 2));
        Reo_z += (diff / norm);
    }
    Reo_z = Reo_z / numOffset;

    Eigen::Matrix3d Reo({ {Reo_x(0), Reo_y(0), Reo_z(0)},
                          {Reo_x(1), Reo_y(1), Reo_z(1)},
                          {Reo_x(2), Reo_y(2), Reo_z(2)} });

    return Reo;
}
//
Eigen::MatrixXd calculation::calculateReo(Eigen::MatrixXd ndiData, int numInit, int numOffset)
{
    Eigen::MatrixXd Reo(0, 3);
    Eigen::Matrix3d ReoInitPos = Eigen::Matrix3d::Zero();

    for (int i = 0; i < numInit; i++)
    {
        ReoInitPos = this->calculateOnceReo(    \
            ndiData(Eigen::seq(i * 3 * numOffset, (i + 1) * 3 * numOffset), Eigen::all), \
            numOffset = numOffset);

        Reo.conservativeResize(Reo.rows() + 3, Reo.cols());
        Reo.block(3 * i, 0, 3, 3) = ReoInitPos;
    }

    return Reo;
}
