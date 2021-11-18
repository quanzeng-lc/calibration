#include "calculation.h"

calculation::calculation(int numInit)
{
    this->numInit_ = numInit;
}


void calculation::calculateRbo(Eigen::MatrixXd ndiData, int numOffset)
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

    Rbo_ = Rbo;
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

void calculation::calculateReo(Eigen::MatrixXd ndiData, int numOffset)
{
    Eigen::MatrixXd Reo(0, 3);
    Eigen::Matrix3d ReoInitPos = Eigen::Matrix3d::Zero();

    for (int i = 0; i < this->numInit_; i++)
    {
        ReoInitPos = this->calculateOnceReo(    \
            ndiData(Eigen::seq(i * 3 * numOffset, (i + 1) * 3 * numOffset), Eigen::all), \
            numOffset = numOffset);

        Reo.conservativeResize(Reo.rows() + 3, Reo.cols());
        Reo.block(3 * i, 0, 3, 3) = ReoInitPos;
    }

    Reo_ = Reo;
}

void calculation::setRtpTtp(Eigen::Matrix3d Rtp, Eigen::Vector3d Ttp)
{
    this->Rtp_ = Rtp;
    this->Ttp_ = Ttp;
}

void calculation::setRpoTpo(Eigen::MatrixXd Rpo, Eigen::MatrixXd Tpo)
{
    this->Rpo_ = Rpo;
    this->Tpo_ = Tpo;
}

void calculation::setRtoTto(Eigen::MatrixXd Rto, Eigen::MatrixXd Tto)
{
    this->Rto_ = Rto;
    this->Tto_ = Tto;
}

void calculation::calculateOnceRpoTpo(int initPos, Eigen::Matrix3d& Rpo, Eigen::Vector3d& Tpo)
{
    Eigen::Matrix3d Rto = this->Rto_.block(initPos * 3, 0, 3, 3);
    Rpo = Rto * this->Rtp_.inverse();

    Eigen::Vector3d Tto = this->Tto_.block(initPos, 0, 1, 3);
    Tpo = -Rpo * this->Ttp_ + Tto;
}

void calculation::calculateRpoTpo()
{
    Eigen::MatrixXd Rpo(0, 3);
    Eigen::MatrixXd Tpo(0, 3);
    Eigen::Matrix3d RpoInitPos = Eigen::Matrix3d::Zero();
    Eigen::Vector3d TpoInitPos = Eigen::Vector3d::Zero();

    for (int i = 0; i < this->numInit_; i++)
    {
        this->calculateOnceRpoTpo(i, RpoInitPos, TpoInitPos);

        Rpo.conservativeResize(Rpo.rows() + 3, Rpo.cols());
        Rpo.block(3 * i, 0, 1, 3) = RpoInitPos;

        Tpo.conservativeResize(Tpo.rows() + 1, Tpo.cols());
        Tpo.block(i, 0, 1, 3) = TpoInitPos;
    }
    
    Rpo_ = Rpo;
    Tpo_ = Tpo;
}

void calculation::calculateRet()
{
    Eigen::Matrix3d Ret = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Rpo = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Reo = Eigen::Matrix3d::Zero();
    for (int i = 0; i < this->numInit_; i++)
    {
        Rpo = Rpo_.block(3 * i, 0, 3, 3);
        Reo = Reo_.block(3 * i, 0, 3, 3);

        Ret += Rtp_.inverse() * Rpo.inverse() * Reo;
    }

    Ret_ = Ret / this->numInit_;
}

void calculation::calculateRbe()
{
    Eigen::Matrix3d Rbe = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d RbeInitPos = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Reo = Eigen::Matrix3d::Zero();
    for (int i = 0; i < this->numInit_; i++)
    {
        Reo = Reo_.block(3 * i, 0, 3, 3);
        RbeInitPos = Rbo_ * Reo.inverse();

        Rbe.conservativeResize(Rbe.rows() + 3, Rbe.cols());
        Rbe.block(3 * i, 0, 1, 3) = RbeInitPos;
    }
    Rbe_ = Rbe;
}

