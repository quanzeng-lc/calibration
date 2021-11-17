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

    calculation Cal;

    auto Rbo = Cal.calculateRbo(matrixB, 5);
    std::cout << "Rbo:" << std::endl;
    std::cout << Rbo << std::endl;
    std::cout << std::endl;

    auto Reo = Cal.calculateReo(matrixE, 1 ,5);
    std::cout << "Reo:" << std::endl;
    std::cout << Reo << std::endl;

    system("pause");
    return 0;
}
