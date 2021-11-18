#include <iostream>

#include "calculation.h"
#include "caliData.h"

int main()
{
    caliData classData; 
    classData.readMatrixB(".\\NDIData.csv");
    classData.readMatrixE(".\\NDIData.csv");
    Eigen::MatrixXd matrixB = classData.getMatrixB();
    Eigen::MatrixXd matrixE = classData.getMatrixE();

    system("pause");
    return 0;
}
