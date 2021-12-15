#pragma once
#include "Eigen/Eigen"

class calculation
{
public:
    calculation(int numInit, int numOffset);
	Eigen::Matrix3d calculateOneceRbo(Eigen::MatrixXd ndiData);
	void calculateRbo(Eigen::MatrixXd ndiData);
    Eigen::Matrix3d calculateOnceReo(Eigen::MatrixXd ndiData);
    void calculateReo(Eigen::MatrixXd ndiData);
    void setRtoTto(Eigen::MatrixXd matrixE);
	void setRebTeb(Eigen::MatrixXd matrixERobot);
    //void calculateOnceRpoTpo(int initPos, Eigen::Matrix3d& Rpo, Eigen::Vector3d& Tpo);
    //void calculateRpoTpo();
    void calculateRet();
    void calculateTetAndTbo();
	Eigen::MatrixXd RobotToTransformationMatrix(Eigen::MatrixXd robotMaxtrix);
	Eigen::MatrixXd NDIToTransformationMatrix(Eigen::MatrixXd NDIMaxtrix);

	Eigen::Matrix3d getRbo();
	Eigen::MatrixXd getReo();
	Eigen::MatrixXd getRto();
	Eigen::MatrixXd getTto();
	Eigen::Matrix3d getRet();
	Eigen::MatrixXd getReb();
	Eigen::MatrixXd getTeb();

	Eigen::MatrixXd getTet();
	Eigen::MatrixXd getTbo();

private:
    int numInit_ = 0;
	int numOffset_ = 0;
    Eigen::Matrix3d Rbo_;
    Eigen::MatrixXd Reo_;
    //Eigen::Matrix3d Rtp_;
    //Eigen::Vector3d Ttp_;
    //Eigen::MatrixXd Rpo_;
    //Eigen::MatrixXd Tpo_;
    Eigen::MatrixXd Rto_;
    Eigen::MatrixXd Tto_;
    Eigen::Matrix3d Ret_;
    Eigen::MatrixXd Reb_;
	Eigen::MatrixXd Teb_;
	Eigen::MatrixXd Tet_;
	Eigen::MatrixXd Tbo_;

};
