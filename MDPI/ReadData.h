#pragma once

#include<string>
#include<Eigen/Eigen>
#include<fstream>

class ReadData
{
public:
	ReadData();

	//读取NDI获得的坐标系参数
	void readNDIData(std::string filename, Eigen::MatrixXd& matrix);
	//读取机器人末端获得的坐标系参数 六个参数
	//也可以用来读取机器人的每个姿态的关节角， 也是六个参数
	void readRobotData(std::string filename, Eigen::MatrixXd& matrix);
	//读取机器人的DH参数
	void readRobotDHParam(std::string filename, Eigen::MatrixXd & matrix);
	//读取机器人的末端和marker之间的转换关系
	void readInitEffectorMarker(std::string filename, Eigen::Matrix4d & matrix);
};

