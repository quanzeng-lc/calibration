// MDPI.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
// 用来NDI和机器人的标定代码与实验
// 张东文的博士论文实现方法
//

#include <iostream>
#include "ReadData.h"
#include "CalQuaternion.h"

int main()
{
	ReadData data;
	Calibration cal;

	//读取机器人的DH参数表
	Eigen::MatrixXd DHParam_list(6, 4);
	data.readRobotDHParam("..\\x64\\Debug\\calibration_data\\MDPI\\UR5e_DH.csv", DHParam_list);
	cal.setNorminalDHParam(DHParam_list);

	//读取初始的机器人末端关节与marker之间的转换关系
	Eigen::Matrix4d Vet;
	data.readInitEffectorMarker("..\\x64\\Debug\\calibration_data\\MDPI\\20211215\\Vet.csv", Vet);
	//保存读取的Vet
	cal.setVet(Vet);
	std::cout << "Vet:" << std::endl;
	std::cout << Vet << std::endl;
	std::cout << std::endl;

	/*
	//机器人每个关节的角度值，刚开始用来测试DH参数的正确性
	Eigen::VectorXd joint_angle(6);
	joint_angle << -0.1472, -1.5248, 1.5758, -1.6742, -1.6161, 2.9677 ;
	//DH参数转化成的机器人末端坐标系,验证DH参数是否正确，选取了一组DH参数
	Eigen::Matrix4d DH_robot_effector_matrix = cal.RobotDHMatrixAndJointAngle(joint_angle);
	std::cout << "DH_robot_effector_matrix" << std::endl;
	std::cout << DH_robot_effector_matrix << std::endl << std::endl;
	*/

	//读取机器人的关节角度 六个关节角度 用于以后计算机器人的末端坐标系
	//To do ...
	Eigen::MatrixXd ROBOT_joint_angle_data;
	data.readRobotData("..\\x64\\Debug\\calibration_data\\MDPI\\20211215\\BaseRobotAngleData.csv", ROBOT_joint_angle_data);
	int robotAngle_data_num = ROBOT_joint_angle_data.rows();

	/*
	//在测试对偶四元数的优化确定 机器人和NDI的坐标系转换关系 
	//我们直接使用获得的机器人末端坐标系
	Eigen::MatrixXd ROBOT_effector_data;
	data.readRobotData("..\\x64\\Debug\\calibration_data\\MDPI\\20211215\\BaseRobotData.csv", ROBOT_effector_data);
	int robot_data_num = ROBOT_effector_data.rows();
	//机器人的轴角和位置参数转换为 n*4*4的矩阵
	Eigen::MatrixXd Robot_effector_matrix = cal.RobotToTransformationMatrix(ROBOT_effector_data);
	//为获得Marker在机器人基座标下的坐标系表示，还需要乘 Vet e: effector t:marker
	Eigen::MatrixXd robot_marker_matrix = Robot_effector_matrix * Vet;
	*/

	//读取NDI获得marker的坐标系 7个参数 前四个参数是Marker的姿态 后三个参数是Marker的空间位置
	Eigen::MatrixXd NDI_data;
	data.readNDIData("..\\x64\\Debug\\calibration_data\\MDPI\\20211215\\BaseNDIData.csv", NDI_data);
	//NDI数据点的个数
	int NDI_data_num = NDI_data.rows();
	//NDI参数转换为 n*4*4的矩阵
	Eigen::MatrixXd NDI_matrix = cal.NDIToTransformationMatrix(NDI_data);


	//保证机器人获得数据点个数和DNI获得的点个数相同
	if (NDI_data_num == robotAngle_data_num) {
		cal.set_point_num(NDI_data_num);
		if (NDI_data_num == 0) {
			return 0;
		}
	}
	else {
		return 0;
	}

	Eigen::MatrixXd final_Vob;
	for (int i = 0; i < 1; i++) {
		//机器人的关节角转换为 n*4*4 的矩阵
		Eigen::MatrixXd RobotDH_effector_matrix = cal.RobotDHMatrixAndMultiJointAngle(ROBOT_joint_angle_data);
		//为获得Marker在机器人基座标下的坐标系表示，还需要乘 Vet e: effector t:marker
		Eigen::MatrixXd robotDH_marker_matrix = RobotDH_effector_matrix * Vet;


		//设置转换矩阵机器人的向量差系数
		Eigen::VectorXd vector_coefficient(3 * NDI_data_num);
		vector_coefficient.setOnes();
		//vector_coefficient = vector_coefficient;
		//求解转换矩阵矩阵的偏移项系数
		Eigen::VectorXd points_coefficient(NDI_data_num);
		points_coefficient.setOnes();
		points_coefficient = 0.1 * points_coefficient;
		//返回的对偶四元数向量 8 * 1
		Eigen::VectorXd optimal_quaternion(8);
		cal.optimalTransQuaternion(robotDH_marker_matrix, NDI_matrix,
			vector_coefficient, points_coefficient, optimal_quaternion);
		//对偶四元数转转换矩阵
		Eigen::Matrix4d Vob = cal.DualQuaternion2Matrix(optimal_quaternion);
		cal.setVob(Vob);
		std::cout << "Vob:" << std::endl;
		std::cout << Vob << std::endl;
		std::cout << std::endl;
		final_Vob = Vob;
		//更新DH参数，定义参数扰动 第一个是theta的初始角度 第二个是 a 的连杆长度 第三个是 d 连杆长度 一共12个参数
		//以机器人末端坐标系为参考系 Marker的坐标系不变
		//在一个点计算 雅各比矩阵 以后可以取多个点的雅各比矩阵
		//为了减少计算量 左边的差只计算偏移的差 n*6*1 向量
		//计算各个变量的增量
		cal.calculateIncrement(ROBOT_joint_angle_data, robotDH_marker_matrix, NDI_matrix);
	}
	Eigen::VectorXd theta_increment = cal.getTheta_increment();
	std::cout << "theta_increment:" << std::endl;
	std::cout << theta_increment << std::endl;
	std::cout << std::endl;

	Eigen::VectorXd d_increment = cal.getd_increment();
	std::cout << "d_increment:" << std::endl;
	std::cout << d_increment << std::endl;
	std::cout << std::endl;

	Eigen::VectorXd a_increment = cal.geta_increment();
	std::cout << "a_increment:" << std::endl;
	std::cout << a_increment << std::endl;
	std::cout << std::endl;

	//读取测试的数据
	Eigen::MatrixXd ROBOT_joint_angle_test_data;
	data.readRobotData("..\\x64\\Debug\\calibration_data\\MDPI\\20211215\\test\\RobotJoint.csv", ROBOT_joint_angle_test_data);
	int robotAngle_data_num_test = ROBOT_joint_angle_test_data.rows();
	//机器人的关节角转换为 n*4*4 的矩阵
	Eigen::MatrixXd RobotDH_effector_matrix_test = cal.RobotDHMatrixAndMultiJointAngle(ROBOT_joint_angle_test_data);
	//为获得Marker在机器人基座标下的坐标系表示，还需要乘 Vet e: effector t:marker
	Eigen::MatrixXd robotDH_marker_matrix_test = RobotDH_effector_matrix_test * Vet;
	Eigen::MatrixXd calculateTool(robotDH_marker_matrix_test.rows(), robotDH_marker_matrix_test.cols());
	for (int i = 0; i < robotAngle_data_num_test; i++) {
		Eigen::Matrix4d result = final_Vob * robotDH_marker_matrix_test.block(4 * i, 0, 4, 4);
		calculateTool.block(4 * i, 0, 4, 4) = result;
	}
	std::cout << "calculateTool:" << std::endl;
	std::cout << calculateTool << std::endl;
	std::cout << std::endl;

	Eigen::MatrixXd NDI_data_test;
	data.readNDIData("..\\x64\\Debug\\calibration_data\\MDPI\\20211215\\test\\NDIData.csv", NDI_data_test);
	//NDI数据点的个数
	int NDI_data_num_test = NDI_data.rows();
	//NDI参数转换为 n*4*4的矩阵
	Eigen::MatrixXd NDI_matrix_test = cal.NDIToTransformationMatrix(NDI_data_test);
	std::cout << "NDI_matrix_test:" << std::endl;
	std::cout << NDI_matrix_test << std::endl;
	std::cout << std::endl;

	Eigen::MatrixXd loss_matrix = NDI_matrix_test - calculateTool;
	std::cout << "loss_matrix:" << std::endl;
	std::cout << loss_matrix << std::endl;
	std::cout << std::endl;

    std::cout << "Hello World!\n";
}
