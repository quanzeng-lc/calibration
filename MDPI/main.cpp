// MDPI.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
// 用来NDI和机器人的标定代码与实验
// 张东文的博士论文实现方法
//

#include <iostream>
#include "ReadData.h"
#include "CalQuaternion.h"

int main()
{
	//我们需要对机器人的6个连杆参数 其中4个是Z轴的偏移参数 两个是X轴的偏移参数
	std::vector<double> link_disturbance = { 0, 0, 0, 0, 0, 0 };
	//6个初始角度的偏移参数
	std::vector<double> init_theta_angle_disturbance = { 0, 0, 0, 0, 0, 0 };

	ReadData data;
	Calibration cal;

	//读取机器人的DH参数表
	std::vector<std::vector<double>> DHParam_list;
	data.readRobotDHParam("..\\x64\\Debug\\calibration_data\\MDPI\\UR5e_DH.csv", DHParam_list);
	//机器人每个关节的角度值，刚开始用来测试DH参数的正确性
	std::vector<double> joint_angle = { -0.1472, -1.5248, 1.5758, -1.6742, -1.6161, 2.9677 };
	//DH参数转化成的机器人末端坐标系,验证DH参数是否正确，选取了一组DH参数
	Eigen::Matrix4d DH_robot_effector_matrix = cal.RobotDHMatrixAndJointAngle(DHParam_list, joint_angle);
	//读取机器人的关节角度 六个关节角度 用于以后计算机器人的末端坐标系
	//To do ...


	//读取NDI获得marker的坐标系 7个参数 前四个参数是Marker的姿态 后三个参数是Marker的空间位置
	Eigen::MatrixXd NDI_data;
	data.readNDIData("..\\x64\\Debug\\calibration_data\\20211209\\BaseNDIData.csv", NDI_data);
	//NDI数据点的个数
	int robot_data_num = NDI_data.rows();
	//NDI参数转换为 n*4*4的矩阵
	Eigen::MatrixXd NDI_matrix = cal.NDIToTransformationMatrix(NDI_data);

	//在测试对偶四元数的优化确定 机器人和NDI的坐标系转换关系 
	//我们直接使用获得的机器人末端坐标系
	Eigen::MatrixXd ROBOT_effector_data;
	data.readRobotData("..\\x64\\Debug\\calibration_data\\20211209\\BaseRobotData.csv", ROBOT_effector_data);
	int NDI_data_num = ROBOT_effector_data.rows();
	//机器人的轴角和位置参数转换为 n*4*4的矩阵
	Eigen::MatrixXd Robot_effector_matrix = cal.RobotToTransformationMatrix(ROBOT_effector_data);

	//保证机器人获得数据点个数和DNI获得的点个数相同
	if (NDI_data_num == robot_data_num) {
		cal.set_point_num(NDI_data_num);
		if (NDI_data_num == 0) {
			return 0;
		}
	}
	else {
		return 0;
	}
	//读取初始的机器人末端关节与marker之间的转换关系
	Eigen::Matrix4d Vet;
	data.readInitEffectorMarker("..\\x64\\Debug\\calibration_data\\MDPI\\Vet.csv", Vet);
	//为获得Marker在机器人基座标下的坐标系表示，还需要乘 Vet e: effector t:marker
	Eigen::MatrixXd robot_marker_matrix = Robot_effector_matrix * Vet;
	//求解转换矩阵机器人的向量差系数
	Eigen::VectorXd vector_coefficient(3 * NDI_data_num);
	vector_coefficient.setOnes();
	//vector_coefficient = 0.5 * vector_coefficient;
	//求解转换矩阵矩阵的偏移项系数
	Eigen::VectorXd points_coefficient(NDI_data_num);
	points_coefficient.setOnes();
	//返回的对偶四元数向量 8 * 1
	Eigen::VectorXd optimal_quaternion(8);
	cal.optimalTransQuaternion(robot_marker_matrix, NDI_matrix, 
		vector_coefficient, points_coefficient, optimal_quaternion);
	//对偶四元数转转换矩阵
	Eigen::Matrix4d Vob = cal.DualQuaternion2Matrix(optimal_quaternion);
	std::cout << "Vob:" << std::endl;
	std::cout << Vob << std::endl;
	std::cout << std::endl;

    std::cout << "Hello World!\n";
}
