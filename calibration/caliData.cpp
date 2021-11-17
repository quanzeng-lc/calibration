#include "caliData.h"

#include <iostream>

caliData::caliData()
{
}

void caliData::readNDIData(std::string filename, Eigen::MatrixXd& matrix)
{
    std::string lineStr;
    std::vector<std::vector<std::string>> table;
    std::ifstream file(filename.c_str());

    while (getline(file, lineStr))
    {
        std::stringstream ss(lineStr);
        std::string elements;
        std::vector<std::string> line;

        while (getline(ss, elements, ','))
        {
            line.push_back(elements);
        }
        table.push_back(line);
    }

    matrix = Eigen::MatrixXd(table.size() - 1, 7);

    for (int i = 1; i < table.size(); i++)
    {
        for (int j = 4; j < 11; j++)
        {
            double string2double = std::stod(table[i][j].c_str());
            matrix(i - 1, j - 4) = string2double;
        }
    }

}

void caliData::readMatrixB(std::string filename)
{
    this->readNDIData(filename, this->MatrixB_);
}

void caliData::readMatrixE(std::string filename)
{
    this->readNDIData(filename, this->MatrixE_);
}

Eigen::MatrixXd caliData::getMatrixB()
{
    return MatrixB_;
}

Eigen::MatrixXd caliData::getMatrixE()
{
    return MatrixE_;
}
