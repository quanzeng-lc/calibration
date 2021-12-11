#pragma once
#include<Eigen/Eigen>

class Calibration
{
private:
	//用来记录点的个数
	int point_num;
public:
	Calibration();

	void set_point_num(int point_num);
	//第一个参数是DH的参数， 第二个参数是UR机械臂的每个关节角度
	//最终形成了机器人末端坐标系的表示
	Eigen::Matrix4d RobotDHMatrixAndJointAngle(std::vector<std::vector<double>> param_list, std::vector<double> jointAngle);
	//根据D-H参数生成相邻关节的转换矩阵
	// param 参数顺序是 theta dis alpha a_dis
	Eigen::Matrix4d DHParam2Matrix(std::vector<double> param);
	//多个DH参数形成了机器人的末端坐标系
	Eigen::Matrix4d RobotDHMatrix(std::vector<std::vector<double>> param_list);

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
	//机器人的六个
	Eigen::MatrixXd RobotToTransformationMatrix(Eigen::MatrixXd robotMaxtrix);

};

