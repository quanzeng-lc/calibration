#pragma once
#include "Eigen/Eigen"

class calculation
{
public:
    calculation(int numInit);
    void calculateRbo(Eigen::MatrixXd ndiData, int numOffset);
    Eigen::Matrix3d calculateOnceReo(Eigen::MatrixXd ndiData, int numOffset);
    void calculateReo(Eigen::MatrixXd ndiData, int numOffset);
    void setRtpTtp(Eigen::Matrix3d Rtp, Eigen::Vector3d Ttp);
    void setRpoTpo(Eigen::MatrixXd Rpo, Eigen::MatrixXd Tpo);
    void setRtoTto(Eigen::MatrixXd Rto, Eigen::MatrixXd Tto);
    void calculateOnceRpoTpo(int initPos, Eigen::Matrix3d& Rpo, Eigen::Vector3d& Tpo);
    void calculateRpoTpo();
    void calculateRet();
    void calculateRbe();

private:
    int numInit_ = 0;
    Eigen::Matrix3d Rbo_;
    Eigen::Matrix3d Reo_;
    Eigen::Matrix3d Rtp_;
    Eigen::Vector3d Ttp_;
    Eigen::MatrixXd Rpo_;
    Eigen::MatrixXd Tpo_;
    Eigen::MatrixXd Rto_;
    Eigen::MatrixXd Tto_;
    Eigen::Matrix3d Ret_;
    Eigen::MatrixXd Rbe_;
};
