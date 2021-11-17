#include <iostream>

#include "calculation.h"
#include "caliData.h"

int main()
{
    caliData testData; 
    testData.readNDIData(".\\NDIData.csv");
    Eigen::MatrixXd ndiData = testData.getNDIData();

    calculation testCal;
    auto testShow = testCal.calculateReo(ndiData, 1 ,5);
    std::cout << testShow << std::endl;

    system("pause");
    return 0;
}
