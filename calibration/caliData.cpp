#include "caliData.h"

#include <iostream>

caliData::caliData()
{
}

void caliData::readNDIData(std::string filename)
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

    NDIData_ = Eigen::MatrixXd(table.size() - 1, 7);

    for (int i = 1; i < table.size(); i++)
    {
        for (int j = 4; j < 11; j++)
        {
            double string2double = std::stod(table[i][j].c_str());
            this->NDIData_(i - 1, j - 4) = string2double;
        }
    }

}

Eigen::MatrixXd caliData::getNDIData()
{
    return NDIData_;
}
