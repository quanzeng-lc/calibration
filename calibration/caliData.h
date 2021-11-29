#pragma once

#include <fstream>
#include <sstream>
#include <string>

#include "Eigen/Eigen"

class caliData
{
public:
    caliData();
    void readNDIData(std::string filename, Eigen::MatrixXd& matrix);
	void readRobotData(std::string filename, Eigen::MatrixXd& matrix);

    void readMatrixB(std::string filename);
    void readMatrixE(std::string filename);
	void readMatrixERobot(std::string filename);

	void readMatrixTestRobot(std::string filename);
	void readMatrixTestNDI(std::string filename);

    Eigen::MatrixXd getMatrixB();
    Eigen::MatrixXd getMatrixE();
	Eigen::MatrixXd getMatrixERobot();
	Eigen::MatrixXd getMatrixTestRobot();
	Eigen::MatrixXd getMatrixTestNDI();

private:
    
    Eigen::MatrixXd MatrixB_;
    Eigen::MatrixXd MatrixE_;
	Eigen::MatrixXd MatrixERobot_;

	Eigen::MatrixXd MatrixTestRobot_;
	Eigen::MatrixXd MatrixTestNDI_;
};
