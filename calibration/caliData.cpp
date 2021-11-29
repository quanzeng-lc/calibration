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

	matrix = Eigen::MatrixXd(table.size(), 7);

	for (int i = 0; i < table.size(); i++)
	{
		for (int j = 0; j < 7; j++)
		{
			double string2double = std::stod(table[i][j].c_str());
			matrix(i, j) = string2double;
		}
	}

}

void caliData::readRobotData(std::string filename, Eigen::MatrixXd & matrix)
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

	matrix = Eigen::MatrixXd(table.size(), 6);

	for (int i = 0; i < table.size(); i++)
	{
		for (int j = 0; j < 6; j++)
		{
			double string2double = std::stod(table[i][j].c_str());
			matrix(i, j) = string2double;
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

void caliData::readMatrixERobot(std::string filename)
{
	this->readRobotData(filename, this->MatrixERobot_);
}

void caliData::readMatrixTestRobot(std::string filename) {
	this->readRobotData(filename, this->MatrixTestRobot_);
}
void caliData::readMatrixTestNDI(std::string filename) {
	this->readNDIData(filename, this->MatrixTestNDI_);
}

Eigen::MatrixXd caliData::getMatrixB()
{
    return MatrixB_;
}

Eigen::MatrixXd caliData::getMatrixE()
{
    return MatrixE_;
}

Eigen::MatrixXd caliData::getMatrixERobot()
{
    return MatrixERobot_;
}

Eigen::MatrixXd caliData::getMatrixTestRobot() {
	return MatrixTestRobot_;
}

Eigen::MatrixXd caliData::getMatrixTestNDI() {
	return MatrixTestNDI_;
}