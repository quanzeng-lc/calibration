#pragma once

#include <fstream>
#include <sstream>
#include <string>

#include "Eigen/Eigen"

class caliData
{
public:
    caliData();
    void readNDIData(std::string filename);
    Eigen::MatrixXd getNDIData();

private:
    
    int numMarkers_ = 0;
    Eigen::MatrixXd NDIData_;
};
