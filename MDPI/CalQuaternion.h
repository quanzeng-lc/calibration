#pragma once
#include<Eigen/Eigen>
#include<iostream>

class Calibration
{
private:
	//用来记录点的个数
	int point_num;
	//存放DH参数表
	Eigen::MatrixXd DH_list;
	Eigen::Matrix4d Vet;
	Eigen::Matrix4d Vob;
	//用来求偏导数的增量
	double delta;

	Eigen::VectorXd theta_increment;
	Eigen::VectorXd d_increment;
	Eigen::VectorXd a_increment;

public:
	Calibration();

	void set_point_num(int point_num);
	//将DH参数放到成员变量里面
	void setNorminalDHParam(Eigen::MatrixXd DHParam_list);
	//将在文件中读取的Marker相对于机器人末端的坐标系保存
	void setVet(Eigen::Matrix4d Vet);
	//将计算得到的Vob保存下来
	void setVob(Eigen::Matrix4d Vob);

	//参数是UR机械臂的每个关节角度，相当于第一列加上了关节角度
	//最终形成了机器人末端坐标系的表示
	Eigen::Matrix4d RobotDHMatrixAndJointAngle(Eigen::VectorXd jointAngle);
	//多个位姿对应的关节角得到的末端坐标系
	Eigen::MatrixXd RobotDHMatrixAndMultiJointAngle(Eigen::MatrixXd jointAngle_list);
	//初始姿态的DH参数 对其中的d参数进行修正
	Eigen::Matrix4d RobotDHMatrixAndDParam(Eigen::VectorXd dParam);
	//初始姿态的DH参数 对其中的a参数进行修正
	Eigen::Matrix4d RobotDHMatrixAndAParam(Eigen::VectorXd aParam);
	//
	Eigen::Matrix4d RobotDHMatrixJointAngleAndDParam(Eigen::VectorXd jointAngle, Eigen::VectorXd dParam);
	//
	Eigen::Matrix4d RobotDHMatrixJointAngleAndAParam(Eigen::VectorXd jointAngle, Eigen::VectorXd aParam);

	//根据D-H参数生成相邻关节的转换矩阵
	// param 参数顺序是 theta dis alpha a_dis
	Eigen::Matrix4d DHParam2Matrix(Eigen::VectorXd);
	//多个DH参数形成了机器人的末端坐标系
	Eigen::Matrix4d RobotDHMatrix(Eigen::MatrixXd param_list);

	//计算论文中的Q(r)表示矩阵函数
	//参数表示四元数的向量
	Eigen::Matrix4d calculateQuaternionQr(Eigen::Vector4d quaternionR);

	//计算论文中的W(r)表示矩阵函数
	//参数表示四元数的向量
	Eigen::Matrix4d calculateQuaternionWr(Eigen::Vector4d quaternionR);
	//计算三维向量的反对称矩阵
	Eigen::Matrix3d calculateSkewSymMatrix(Eigen::Vector3d threeDVector);

	//对两个坐标系的统一
	//第一个输入marker相对于机械臂基座的坐标系表示 n*4*4的转换矩阵 n是点的个数
	//第二个输入marker在NDI基座标下的坐标系表示 n*4*4的转换矩阵 n是点的个数
	//第三个参数是 每个对方向向量的参数 
	//第三个参数是 每个点的参数
	//返回值是两个四元数 第一个是R 第二个是S
	Eigen::VectorXd optimalTransQuaternion(Eigen::MatrixXd robotMarkerMatrix, Eigen::MatrixXd NDIMarkerMatrix, 
		Eigen::VectorXd vectorParam, Eigen::VectorXd pointParam, Eigen::VectorXd& optimalQuaternion);
	
	//求解C1的矩阵
	Eigen::Matrix4d calculateC1(Eigen::MatrixXd robotMarkerMatrixRotation, Eigen::MatrixXd robotMarkerMatrixTranslation,
								Eigen::MatrixXd NDIMarkerMatrixRotation, Eigen::MatrixXd NDIMarkerMatrixTranslation,
								Eigen::VectorXd vectorParam, Eigen::VectorXd pointParam);
	//求解C2的矩阵
	Eigen::Matrix4d calculateC2(Eigen::VectorXd pointParam);
	//求解C3的矩阵
	Eigen::Matrix4d calculateC3(Eigen::MatrixXd robotMarkerMatrixTranslation, Eigen::MatrixXd NDIMarkerMatrixTranslation, 
		Eigen::VectorXd pointParam);
	//求解常量
	double calculateConst(Eigen::MatrixXd robotMarkerMatrixTranslation, Eigen::MatrixXd NDIMarkerMatrixTranslation,
		Eigen::VectorXd vectorParam, Eigen::VectorXd pointParam);
	//求解最大的特征值及其特征向量
	void calculateMaxEigenValueAndVector(Eigen::Matrix4d matrix, double& max_value, Eigen::Vector4d& corr_vector);
	//机器人的六个参数表示的坐标系 转换成 4*4 变换矩阵
	Eigen::MatrixXd RobotToTransformationMatrix(Eigen::MatrixXd robotMaxtrix);
	//NDI的七个参数表示的坐标系 转换成 4*4 变换矩阵 前四个是姿态 后三个是空间位置
	Eigen::MatrixXd NDIToTransformationMatrix(Eigen::MatrixXd NDIMaxtrix);
	//对偶四元数转换成4*4的转换矩阵
	Eigen::Matrix4d DualQuaternion2Matrix(Eigen::VectorXd dualQuaternion);
	//转换矩阵变成 x y z alpha beta gama
	Eigen::VectorXd matrix2XYZEulerAngle(Eigen::Matrix4d transform_matrixd);

	//计算单个六轴关节角的雅各比系数矩阵 坐标的误差只计算x y z alpha beta gama 6个误差 参数扰动是12个参数 
	//因此返回值是 6*12 的矩阵
	Eigen::MatrixXd calculateOnePointJacobiMatrix(Eigen::VectorXd joint_angle);
	//计算多个点位的雅各比系数矩阵
	Eigen::MatrixXd calculateMultiPointJacobiMatrix(Eigen::MatrixXd joint_angle);
	//计算多个点位的误差 NDI获得点和通过Vob计算出来的点比较
	Eigen::VectorXd calculateMultiPointDifference(Eigen::MatrixXd robot_marker_matrix, Eigen::MatrixXd NDI_matrix);
	//计算各个增量
	void calculateIncrement(Eigen::MatrixXd joint_angle, Eigen::MatrixXd robot_marker_matrix, Eigen::MatrixXd NDI_matrix);
};

