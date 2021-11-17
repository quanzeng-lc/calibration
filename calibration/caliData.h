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

    void readMatrixB(std::string filename);
    void readMatrixE(std::string filename);
    Eigen::MatrixXd getMatrixB();
    Eigen::MatrixXd getMatrixE();

private:
    
    Eigen::MatrixXd MatrixB_;
    Eigen::MatrixXd MatrixE_;
};
