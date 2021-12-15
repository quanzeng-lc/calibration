#include "CalQuaternion.h"

Calibration::Calibration() {
	point_num = 0;
	delta = 1e-6;
	this->DH_list.conservativeResize(6, 4);
	theta_increment.conservativeResize(6);
	a_increment.conservativeResize(6);
	d_increment.conservativeResize(6);
	theta_increment.setZero();
	a_increment.setZero();
	d_increment.setZero();
}

void Calibration::set_point_num(int point_num)
{
	this->point_num = point_num;
}

void Calibration::setNorminalDHParam(Eigen::MatrixXd DHParam_list) {
	this->DH_list = DHParam_list;
}

void Calibration::setVet(Eigen::Matrix4d Vet)
{
	this->Vet = Vet;
}

void Calibration::setVob(Eigen::Matrix4d Vob)
{
	this->Vob = Vob;
}

Eigen::VectorXd Calibration::getTheta_increment()
{
	return this->theta_increment;
}

Eigen::VectorXd Calibration::getd_increment()
{
	return this->d_increment;
}

Eigen::VectorXd Calibration::geta_increment()
{
	return this->a_increment;
}

Eigen::Matrix4d Calibration::RobotDHMatrixAndJointAngle(Eigen::VectorXd jointAngle) {
	int param_list_num = jointAngle.rows();
	//相当于第一列加上了关节角度
	Eigen::MatrixXd dh_param = DH_list;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	//加上输入的theta角度
	dh_param(Eigen::all, Eigen::seq(0, 0)) = dh_param(Eigen::all, Eigen::seq(0, 0)) + jointAngle;
	//加上每个参数的增量
	dh_param(Eigen::all, Eigen::seq(0, 0)) = dh_param(Eigen::all, Eigen::seq(0, 0)) + theta_increment;
	dh_param(Eigen::all, Eigen::seq(1, 1)) = dh_param(Eigen::all, Eigen::seq(1, 1)) + d_increment;
	dh_param(Eigen::all, Eigen::seq(3, 3)) = dh_param(Eigen::all, Eigen::seq(3, 3)) + a_increment;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	Eigen::Matrix4d trans_matrix;
	trans_matrix = RobotDHMatrix(dh_param);
	return trans_matrix;
}

Eigen::MatrixXd Calibration::RobotDHMatrixAndMultiJointAngle(Eigen::MatrixXd jointAngle_list)
{
	int rows = jointAngle_list.rows();
	Eigen::MatrixXd multi_matrix(4 * rows, 4);
	for (int i = 0; i < rows; i++) {
		Eigen::VectorXd joint_angle(6);
		joint_angle = jointAngle_list(Eigen::seq(i, i), Eigen::all).transpose();
		Eigen::Matrix4d effector_matrix = RobotDHMatrixAndJointAngle(joint_angle);
		//if (i < 10) {
		//	std::cout << "effector_matrix " << i << ":" << std::endl;
		//	std::cout << effector_matrix << std::endl;
		//	std::cout << std::endl;
		//}
		multi_matrix(Eigen::seq(4 * i, 4 * i + 3), Eigen::all) = effector_matrix;
	}
	return multi_matrix;
}


Eigen::Matrix4d Calibration::RobotDHMatrixAndDParam(Eigen::VectorXd dParam) {
	int param_list_num = dParam.rows();
	//相当于第二列加上了参数d
	Eigen::MatrixXd dh_param = DH_list;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	dh_param(Eigen::all, Eigen::seq(1, 1)) = dh_param(Eigen::all, Eigen::seq(1, 1)) + dParam;
	//加上每个参数的增量
	dh_param(Eigen::all, Eigen::seq(0, 0)) = dh_param(Eigen::all, Eigen::seq(0, 0)) + theta_increment;
	dh_param(Eigen::all, Eigen::seq(1, 1)) = dh_param(Eigen::all, Eigen::seq(1, 1)) + d_increment;
	dh_param(Eigen::all, Eigen::seq(3, 3)) = dh_param(Eigen::all, Eigen::seq(3, 3)) + a_increment;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	Eigen::Matrix4d trans_matrix;
	trans_matrix = RobotDHMatrix(dh_param);
	return trans_matrix;
}

Eigen::Matrix4d Calibration::RobotDHMatrixAndAParam(Eigen::VectorXd aParam) {
	int param_list_num = aParam.rows();
	//相当于第二列加上了参数d
	Eigen::MatrixXd dh_param = DH_list;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	dh_param(Eigen::all, Eigen::seq(3, 3)) = dh_param(Eigen::all, Eigen::seq(3, 3)) + aParam;
	//加上每个参数的增量
	dh_param(Eigen::all, Eigen::seq(0, 0)) = dh_param(Eigen::all, Eigen::seq(0, 0)) + theta_increment;
	dh_param(Eigen::all, Eigen::seq(1, 1)) = dh_param(Eigen::all, Eigen::seq(1, 1)) + d_increment;
	dh_param(Eigen::all, Eigen::seq(3, 3)) = dh_param(Eigen::all, Eigen::seq(3, 3)) + a_increment;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	Eigen::Matrix4d trans_matrix;
	trans_matrix = RobotDHMatrix(dh_param);
	return trans_matrix;
}

Eigen::Matrix4d Calibration::RobotDHMatrixJointAngleAndDParam(Eigen::VectorXd jointAngle, Eigen::VectorXd dParam) {
	int param_list_num = jointAngle.rows();
	//相当于第一列加上了关节角度
	Eigen::MatrixXd dh_param = DH_list;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	//加上输入的theta角度
	dh_param(Eigen::all, Eigen::seq(0, 0)) = dh_param(Eigen::all, Eigen::seq(0, 0)) + jointAngle;
	//加上每个参数的增量
	dh_param(Eigen::all, Eigen::seq(0, 0)) = dh_param(Eigen::all, Eigen::seq(0, 0)) + theta_increment;
	//加上d参数的增量
	dh_param(Eigen::all, Eigen::seq(1, 1)) = dh_param(Eigen::all, Eigen::seq(1, 1)) + d_increment + dParam;
	dh_param(Eigen::all, Eigen::seq(3, 3)) = dh_param(Eigen::all, Eigen::seq(3, 3)) + a_increment;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	Eigen::Matrix4d trans_matrix;
	trans_matrix = RobotDHMatrix(dh_param);
	return trans_matrix;
}

Eigen::Matrix4d Calibration::RobotDHMatrixJointAngleAndAParam(Eigen::VectorXd jointAngle, Eigen::VectorXd aParam) {
	int param_list_num = jointAngle.rows();
	//相当于第一列加上了关节角度
	Eigen::MatrixXd dh_param = DH_list;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	//加上输入的theta角度
	dh_param(Eigen::all, Eigen::seq(0, 0)) = dh_param(Eigen::all, Eigen::seq(0, 0)) + jointAngle;
	//加上每个参数的增量
	dh_param(Eigen::all, Eigen::seq(0, 0)) = dh_param(Eigen::all, Eigen::seq(0, 0)) + theta_increment;
	dh_param(Eigen::all, Eigen::seq(1, 1)) = dh_param(Eigen::all, Eigen::seq(1, 1)) + d_increment;
	//加上a参数的增量
	dh_param(Eigen::all, Eigen::seq(3, 3)) = dh_param(Eigen::all, Eigen::seq(3, 3)) + a_increment + aParam;
	//std::cout << "dh_param" << std::endl;
	//std::cout << dh_param << std::endl << std::endl;
	Eigen::Matrix4d trans_matrix;
	trans_matrix = RobotDHMatrix(dh_param);
	return trans_matrix;
}

Eigen::Matrix4d Calibration::RobotDHMatrix(Eigen::MatrixXd param_list) {
	Eigen::Matrix4d Robot_trans;
	Robot_trans.setIdentity();
	size_t param_list_num = param_list.rows();
	for (int i = 0; i < param_list_num; i++) {
		//取出其中的一个DH参数，转成转换矩阵
		Eigen::Vector4d param = param_list(Eigen::seq(i, i), Eigen::all).transpose();
		Eigen::Matrix4d DHMatrix;
		DHMatrix.setIdentity();
		int param_num = param.rows();
		if (param_num == 4) {
			DHMatrix = DHParam2Matrix(param);
			Robot_trans = Robot_trans * DHMatrix;
		}
	}
	return Robot_trans;
}

Eigen::Matrix4d Calibration::DHParam2Matrix(Eigen::VectorXd param) {
	double theta = param[0];
	double dis = param[1];
	double alpha = param[2];
	double a_dis = param[3];
	Eigen::Matrix4d trans_matrix;
	//绕着Z轴旋转的矩阵
	Eigen::Matrix4d Rot_z_theta;
	Rot_z_theta.setIdentity();
	Rot_z_theta(0, 0) = cos(theta);
	Rot_z_theta(1, 0) = sin(theta);
	Rot_z_theta(0, 1) = -sin(theta);
	Rot_z_theta(1, 1) = cos(theta);
	//沿着Z轴平移的矩阵
	Eigen::Matrix4d trans_z_d;
	trans_z_d.setIdentity();
	trans_z_d(2, 3) = dis;
	//沿着X轴旋转的矩阵
	Eigen::Matrix4d Rot_x_alpha;
	Rot_x_alpha.setIdentity();
	Rot_x_alpha(1, 1) = cos(alpha);
	Rot_x_alpha(2, 1) = sin(alpha);
	Rot_x_alpha(1, 2) = -sin(alpha);
	Rot_x_alpha(2, 2) = cos(alpha);
	//沿着X轴平移的矩阵
	Eigen::Matrix4d trans_x_dis;
	trans_x_dis.setIdentity();
	trans_x_dis(0, 3) = a_dis;
	trans_matrix = Rot_z_theta * trans_z_d * Rot_x_alpha * trans_x_dis;
	return trans_matrix;
}

Eigen::Matrix4d Calibration::calculateQuaternionQr(Eigen::Vector4d quaternionR) {
	Eigen::Matrix4d matrix;
	Eigen::Vector3d threeVector = quaternionR.block(0, 0, 3, 1);
	matrix.block(0, 0, 3, 3) = quaternionR[3] * Eigen::Matrix3d::Identity() + calculateSkewSymMatrix(threeVector);
	matrix.block(0, 3, 3, 1) = threeVector;
	matrix.block(3, 0, 1, 3) = -1 * threeVector.transpose();
	matrix(3, 3) = quaternionR[3];
	return matrix;
}

Eigen::Matrix4d Calibration::calculateQuaternionWr(Eigen::Vector4d quaternionR) {
	Eigen::Matrix4d matrix;
	Eigen::Vector3d threeVector = quaternionR.block(0, 0, 3, 1);
	matrix.block(0, 0, 3, 3) = quaternionR[3] * Eigen::Matrix3d::Identity() - calculateSkewSymMatrix(threeVector);
	matrix.block(0, 3, 3, 1) = threeVector;
	matrix.block(3, 0, 1, 3) = -1 * threeVector.transpose();
	matrix(3, 3) = quaternionR[3];
	return matrix;
}

Eigen::Matrix3d Calibration::calculateSkewSymMatrix(Eigen::Vector3d threeDVector) {
	Eigen::Matrix3d skew_symmetric_matrix;
	skew_symmetric_matrix.setZero();
	skew_symmetric_matrix(0, 1) = -1 * threeDVector[2];
	skew_symmetric_matrix(0, 2) = threeDVector[1];
	skew_symmetric_matrix(1, 0) = threeDVector[2];
	skew_symmetric_matrix(1, 2) = -1 * threeDVector[0];
	skew_symmetric_matrix(2, 0) = -1 * threeDVector[1];
	skew_symmetric_matrix(2, 1) = threeDVector[0];
	return skew_symmetric_matrix;
}

Eigen::VectorXd Calibration::optimalTransQuaternion(Eigen::MatrixXd robotMarkerMatrix, Eigen::MatrixXd NDIMarkerMatrix, 
	Eigen::VectorXd vectorParam, Eigen::VectorXd pointParam, Eigen::VectorXd & optimalQuaternion)
{
	// robotMarkerMatrix NDIMarkerMatrix n*4*4
	//首先将旋转矩阵和平移向量分离
	//距离的四元数是平移平移向量的1/2 
	Eigen::MatrixXd robotMarkerMatrixRotation = robotMarkerMatrix(Eigen::all, Eigen::seq(0, 2));
	Eigen::MatrixXd robotMarkerMatrixTranslation = 0.5 * robotMarkerMatrix(Eigen::all, Eigen::seq(3, 3));
	//在计算中，偏移向量的最后一个系数为 0
	robotMarkerMatrixTranslation(Eigen::seqN(3, robotMarkerMatrixTranslation.rows()/4, 4), Eigen::all).setZero();
	//std::cout << "translation :"<<std::endl <<robotMarkerMatrixTranslation << std::endl;

	Eigen::MatrixXd NDIMarkerMatrixRotation = NDIMarkerMatrix(Eigen::all, Eigen::seq(0, 2));
	Eigen::MatrixXd NDIMarkerMatrixTraTranslation = 0.5 * NDIMarkerMatrix(Eigen::all, Eigen::seq(3, 3));
	//在计算中，偏移向量的最后一个系数为 0
	NDIMarkerMatrixTraTranslation(Eigen::seqN(3, NDIMarkerMatrixTraTranslation.rows()/4, 4), Eigen::all).setZero();
	//求解C1
	Eigen::Matrix4d C1 = calculateC1(robotMarkerMatrixRotation, robotMarkerMatrixTranslation,
		NDIMarkerMatrixRotation, NDIMarkerMatrixTraTranslation,
		vectorParam, pointParam);
	//求解C2
	Eigen::Matrix4d C2 = calculateC2(pointParam);
	//求解C3
	Eigen::Matrix4d C3 = calculateC3(robotMarkerMatrixTranslation, NDIMarkerMatrixTraTranslation,
		pointParam);
	//求解常量
	double constant = calculateConst(robotMarkerMatrixTranslation, NDIMarkerMatrixTraTranslation,
		vectorParam, pointParam);
	//计算矩阵A 以及最大的特征值和特征向量 最大的特征向量就是需要求的r
	Eigen::Matrix4d A = 0.5*(C3.transpose()*(C2 + C2.transpose()).inverse()*C3 - C1 - C1.transpose());
	double max_eigen_value;
	Eigen::Vector4d max_eigen_vector;
	calculateMaxEigenValueAndVector(A, max_eigen_value, max_eigen_vector);
	Eigen::Vector4d calculate_s = -(C2 + C2.transpose()).inverse()*C3*max_eigen_vector;
	//返回求得的对偶四元数
	optimalQuaternion(Eigen::seq(0, 3), Eigen::all) = max_eigen_vector;
	optimalQuaternion(Eigen::seq(4, 7), Eigen::all) = calculate_s;
	return optimalQuaternion;
}

Eigen::Matrix4d Calibration::calculateC1(Eigen::MatrixXd robotMarkerMatrixRotation, Eigen::MatrixXd robotMarkerMatrixTranslation,
	Eigen::MatrixXd NDIMarkerMatrixRotation, Eigen::MatrixXd NDIMarkerMatrixTranslation,
	Eigen::VectorXd vectorParam, Eigen::VectorXd pointParam)
{
	Eigen::Matrix4d C1;
	Eigen::Matrix4d C1_previous;
	C1_previous.setZero();
	Eigen::Matrix4d C1_next;
	C1_next.setZero();
	for (int i = 0; i < point_num; i++) {
		//取出机器人基座标对应的向量
		Eigen::Vector4d robotX_vector = robotMarkerMatrixRotation.block(4 * i, 0, 4, 1);
		Eigen::Vector4d robotY_vector = robotMarkerMatrixRotation.block(4 * i, 1, 4, 1);
		Eigen::Vector4d robotZ_vector = robotMarkerMatrixRotation.block(4 * i, 2, 4, 1);
		//取出机器人对应的偏移量
		Eigen::Vector4d robot_translation_vector = robotMarkerMatrixTranslation.block(4 * i, 0, 4, 1);
		//取出NDI坐标系下对应的向量
		Eigen::Vector4d NDIX_vector = NDIMarkerMatrixRotation.block(4 * i, 0, 4, 1);
		Eigen::Vector4d NDIY_vector = NDIMarkerMatrixRotation.block(4 * i, 1, 4, 1);
		Eigen::Vector4d NDIZ_vector = NDIMarkerMatrixRotation.block(4 * i, 2, 4, 1);
		//取出NDI坐标系下对应的偏移量
		Eigen::Vector4d NDI_translation_vector = NDIMarkerMatrixTranslation.block(4 * i, 0, 4, 1);
		C1_previous -=  vectorParam[3 * i]*calculateQuaternionQr(NDIX_vector).transpose() * calculateQuaternionWr(robotX_vector);
		C1_previous -=  vectorParam[3 * i + 1]*calculateQuaternionQr(NDIY_vector).transpose() * calculateQuaternionWr(robotY_vector);
		C1_previous -=  vectorParam[3 * i + 2] * calculateQuaternionQr(NDIZ_vector).transpose() * calculateQuaternionWr(robotZ_vector);
		C1_next = C1_next - pointParam[i] * calculateQuaternionQr(NDI_translation_vector).transpose() * calculateQuaternionWr(robot_translation_vector);
	}
	C1 = 2 * (C1_previous + C1_next);
	return C1;
}

Eigen::Matrix4d Calibration::calculateC2(Eigen::VectorXd pointParam)
{
	Eigen::Matrix4d C2;
	C2.setZero();
	double sum = 0;
	for (int i = 0; i < point_num; i++) {
		sum += pointParam[i];
	}
	C2 = sum * Eigen::Matrix4d::Identity();
	return C2;
}

Eigen::Matrix4d Calibration::calculateC3(Eigen::MatrixXd robotMarkerMatrixTranslation, Eigen::MatrixXd NDIMarkerMatrixTranslation, Eigen::VectorXd pointParam)
{
	Eigen::Matrix4d C3;
	C3.setZero();
	for (int i = 0; i < point_num; i++) {
		//取出机器人对应的偏移量
		Eigen::Vector4d robot_translation_vector = robotMarkerMatrixTranslation.block(4 * i, 0, 4, 1);
		//取出NDI坐标对应的偏移量
		Eigen::Vector4d NDI_translation_vector = NDIMarkerMatrixTranslation.block(4 * i, 0, 4, 1);
		C3 = C3 + pointParam[i] * (calculateQuaternionWr(robot_translation_vector) - calculateQuaternionQr(NDI_translation_vector));
	}
	C3 = 2 * C3;
	return C3;
}

double Calibration::calculateConst(Eigen::MatrixXd robotMarkerMatrixTranslation, Eigen::MatrixXd NDIMarkerMatrixTranslation, Eigen::VectorXd vectorParam, Eigen::VectorXd pointParam)
{
	double constant = 0;
	for (int i = 0; i < point_num; i++) {
		constant += 2 * (vectorParam[3 * i] + vectorParam[3 * i + 1] + vectorParam[3 * i + 2]);
		//取出机器人对应的偏移量
		Eigen::Vector4d robot_translation_vector = robotMarkerMatrixTranslation.block(4 * i, 0, 4, 1);
		//取出NDI坐标对应的偏移量
		Eigen::Vector4d NDI_translation_vector = NDIMarkerMatrixTranslation.block(4 * i, 0, 4, 1);
		constant += pointParam[i] * (robot_translation_vector.dot(robot_translation_vector) + NDI_translation_vector.dot(NDI_translation_vector));
	}
	return constant;
}

void Calibration::calculateMaxEigenValueAndVector(Eigen::Matrix4d matrix, double& max_value, Eigen::Vector4d& corr_vector) {
	//求解A的特征值
	Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(matrix);
	//特征值 包含实部和虚部 4*1
	Eigen::MatrixXcd eigen_values = eigen_solver.eigenvalues();
	//特征向量 包含实部和虚部 4*4
	Eigen::MatrixXcd eigen_vector = eigen_solver.eigenvectors();
	//特征向量的实部
	Eigen::VectorXd eigen_values_real = eigen_values.real();
	//特征值最大的位置
	Eigen::MatrixXf::Index eigen_value_max_index;
	eigen_values_real.rowwise().sum().maxCoeff(&eigen_value_max_index);
	max_value = eigen_values_real[eigen_value_max_index];
	//最大的特征向量
	Eigen::Vector4d q_vector;
	//得到最大的特征值
	q_vector << eigen_vector.real()(0, eigen_value_max_index), eigen_vector.real()(1, eigen_value_max_index),
		eigen_vector.real()(2, eigen_value_max_index), eigen_vector.real()(3, eigen_value_max_index);
	corr_vector = q_vector;
}

Eigen::MatrixXd Calibration::RobotToTransformationMatrix(Eigen::MatrixXd robotMaxtrix) {
	int rows = robotMaxtrix.rows();
	int cols = robotMaxtrix.cols();
	Eigen::MatrixXd translationMaxtrix(rows, 3);
	translationMaxtrix = robotMaxtrix.block(0, 0, rows, 3);

	Eigen::MatrixXd tranaformationMatrix(4 * rows, 4);
	tranaformationMatrix.setZero();
	for (int i = 0; i < rows; i++) {
		Eigen::Vector3d vector_tmp = robotMaxtrix.block(i, 3, 1, 3).transpose();
		double angle = sqrt(vector_tmp.transpose() * vector_tmp);
		Eigen::AngleAxisd angleaxis;
		if (angle == 0) {
			angleaxis = Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1));
		}
		else
		{
			angleaxis = Eigen::AngleAxisd(angle, vector_tmp / angle);
		}
		Eigen::Matrix3d rotation = angleaxis.matrix();
		Eigen::Matrix4d effector_matrix;
		effector_matrix.setZero();
		effector_matrix.block(0, 0, 3, 3) = rotation;
		effector_matrix.block(0, 3, 3, 1) = translationMaxtrix.block(i, 0, 1, 3).transpose();
		effector_matrix(3, 3) = 1.0;
		tranaformationMatrix.block(4 * i, 0, 4, 4) = effector_matrix;
		//if (i < 10) {
		//	std::cout << "effector_matrix "<< i <<":" << std::endl;
		//	std::cout << effector_matrix << std::endl;
		//	std::cout << std::endl;
		//}
	}
	return tranaformationMatrix;
}

Eigen::MatrixXd Calibration::NDIToTransformationMatrix(Eigen::MatrixXd NDIMaxtrix) {
	int rows = NDIMaxtrix.rows();
	int cols = NDIMaxtrix.cols();
	Eigen::MatrixXd translationMaxtrix(rows, 3);
	translationMaxtrix = NDIMaxtrix.block(0, 4, rows, 3);
	//最终得到的 4*n * 4的矩阵
	Eigen::MatrixXd tranaformationMatrix(4 * rows, 4);
	tranaformationMatrix.setZero();
	for (int i = 0; i < rows; i++) {
		//从第 0 列开始到 第 3 列 是四元数 
		//从NDI获得数据四元数 第一个是实部 后面三个是虚部
		//Eigen四元数实例化时，如果直接用向量初始化 前三个数是虚部 最后一个数是实部
		//可以用四个数初始化 第一个是实部 后面的是虚部 x y z
		Eigen::Vector4d quaternion_vector = NDIMaxtrix.block(i, 0, 1, 4).transpose();
		Eigen::Quaternion<double> quaternion_tmp = Eigen::Quaternion<double>(quaternion_vector[0], quaternion_vector[1],
			quaternion_vector[2], quaternion_vector[3]);
		//double w = quaternion_tmp.w();
		//double x = quaternion_tmp.x();
		//double y = quaternion_tmp.y();
		//double z = quaternion_tmp.z();
		Eigen::Matrix3d rotation = quaternion_tmp.matrix();
		tranaformationMatrix.block(4 * i, 0, 3, 3) = rotation;
		tranaformationMatrix.block(4 * i, 3, 3, 1) = translationMaxtrix.block(i, 0, 1, 3).transpose();
		tranaformationMatrix(4 * i + 3, 3) = 1.0;
	}
	return tranaformationMatrix;
}

Eigen::Matrix4d Calibration::DualQuaternion2Matrix(Eigen::VectorXd dualQuaternion)
{
	Eigen::Matrix4d matrix;
	//取出四元数的r部分 实部 
	Eigen::Vector4d dual_quaternion_r = dualQuaternion(Eigen::seq(0, 3), Eigen::all);
	Eigen::Matrix4d rotation_matrix = calculateQuaternionWr(dual_quaternion_r).transpose()
		* calculateQuaternionQr(dual_quaternion_r);
	//取出四元数的s部分 对偶部
	Eigen::Vector4d dual_quaternion_s = dualQuaternion(Eigen::seq(4, 7), Eigen::all);
	Eigen::Vector4d t_italic_vector = calculateQuaternionWr(dual_quaternion_r).transpose()
		* dual_quaternion_s;
	Eigen::Vector3d t_vector = 2 * t_italic_vector(Eigen::seq(0, 2), Eigen::all);
	matrix = rotation_matrix;
	matrix.block(0, 3, 3, 1) = t_vector;
	return matrix;
}

Eigen::VectorXd Calibration::matrix2XYZEulerAngle(Eigen::Matrix4d transform_matrixd) {
	Eigen::VectorXd XYZ_EulerAngle_vector(6);
	XYZ_EulerAngle_vector.setZero();
	Eigen::Matrix3d rotation_matrix = transform_matrixd.block(0, 0, 3, 3);
	//先是 yaw 航向角 pitch俯仰角 roll滚向角
	Eigen::Vector3d Eulaer_angle = rotation_matrix.eulerAngles(2, 1, 0);
	XYZ_EulerAngle_vector[0] = transform_matrixd(0, 3);
	XYZ_EulerAngle_vector[1] = transform_matrixd(1, 3);
	XYZ_EulerAngle_vector[2] = transform_matrixd(2, 3);
	XYZ_EulerAngle_vector(Eigen::seq(3, 5), Eigen::all) = Eulaer_angle;
	return XYZ_EulerAngle_vector;
}

Eigen::MatrixXd Calibration::calculateOnePointJacobiMatrix(Eigen::VectorXd joint_angle) {
	Eigen::MatrixXd matrix(6, 12);
	//对theta1求偏导数
	//不加增量得到Marker坐标系在NDI上的表示
	Eigen::Matrix4d effector_matrix = this->Vob*this->RobotDHMatrixAndJointAngle(joint_angle)*this->Vet;
	Eigen::VectorXd effectot_coefficient = matrix2XYZEulerAngle(effector_matrix);
	for (int i = 0; i < 6; i++) {
		Eigen::VectorXd joint_angle_and_delta = joint_angle;
		joint_angle_and_delta[i] += delta;
		Eigen::Matrix4d effector_matrix_and_delta = this->Vob*this->RobotDHMatrixAndJointAngle(joint_angle_and_delta)*this->Vet;
		Eigen::VectorXd effectot_coefficient_and_delta = matrix2XYZEulerAngle(effector_matrix_and_delta);
		//计算x y z alpha beta gama 的偏差系数
		Eigen::VectorXd delta_vector = (effectot_coefficient_and_delta - effectot_coefficient) / delta;
		matrix(Eigen::all, Eigen::seq(i, i)) = delta_vector;
	}
	
	int d_index[4] = {0, 3, 4, 5};
	Eigen::VectorXd d_delta(6);
	d_delta.setZero();
	for (int i = 0; i < 4; i++) {
		d_delta[d_index[i]] += delta;
		Eigen::Matrix4d effector_matrix_and_delta = this->Vob*this->RobotDHMatrixJointAngleAndDParam(joint_angle, d_delta)*this->Vet;
		Eigen::VectorXd effectot_coefficient_and_delta = matrix2XYZEulerAngle(effector_matrix_and_delta);
		//计算x y z alpha beta gama 的偏差系数
		Eigen::VectorXd delta_vector = (effectot_coefficient_and_delta - effectot_coefficient) / delta;
		matrix(Eigen::all, Eigen::seq(i+6, i+6)) = delta_vector;
	}

	int a_index[2] = {1, 2};
	Eigen::VectorXd a_delta(6);
	a_delta.setZero();
	for (int i = 0; i < 2; i++) {
		a_delta[a_index[i]] += delta;
		Eigen::Matrix4d effector_matrix_and_delta = this->Vob*this->RobotDHMatrixJointAngleAndAParam(joint_angle, a_delta)*this->Vet;
		Eigen::VectorXd effectot_coefficient_and_delta = matrix2XYZEulerAngle(effector_matrix_and_delta);
		//std::cout << "effectot_coefficient_and_delta:" << std::endl;
		//std::cout << effectot_coefficient_and_delta << std::endl;
		//std::cout << std::endl;
		//计算x y z alpha beta gama 的偏差系数
		Eigen::VectorXd delta_vector = (effectot_coefficient_and_delta - effectot_coefficient) / delta;
		//std::cout << "delta_vector:" << std::endl;
		//std::cout << delta_vector << std::endl;
		//std::cout << std::endl;
		matrix(Eigen::all, Eigen::seq(i + 10, i + 10)) = delta_vector;
	}
	return matrix;
}

Eigen::MatrixXd Calibration::calculateMultiPointJacobiMatrix(Eigen::MatrixXd joint_angle) {
	int rows = joint_angle.rows();
	Eigen::MatrixXd Jagobi_matrix(6 * rows, 12);
	for (int i = 0; i < rows; i++) {
		Eigen::VectorXd one_point_joint_angle(6);
		one_point_joint_angle = joint_angle(Eigen::seq(i, i), Eigen::all).transpose();
		Eigen::MatrixXd onePointJagobi = calculateOnePointJacobiMatrix(one_point_joint_angle);
		//std::cout << "onePointJagobi:" << std::endl;
		//std::cout << onePointJagobi << std::endl;
		//std::cout << std::endl;
		Jagobi_matrix(Eigen::seq(6 * i, 6 * i + 5), Eigen::all) = onePointJagobi;
	}
	return Jagobi_matrix;
}

Eigen::VectorXd Calibration::calculateMultiPointDifference(Eigen::MatrixXd robot_marker_matrix, Eigen::MatrixXd NDI_matrix)
{
	//误差向量 n*6
	Eigen::VectorXd difference_vector(point_num * 6);
	for (size_t i = 0; i < point_num; i++)
	{
		//取出Marker在机器人基座标下的坐标系
		Eigen::Matrix4d robot_matrix = robot_marker_matrix(Eigen::seq(4 * i, 4 * i + 3), Eigen::all);
		//转换到OTS坐标系下
		Eigen::Matrix4d cal_NDI_marker = this->Vob*robot_matrix;
		//转换成 xyz alpha beta gama
		Eigen::VectorXd cal_xyz_alpha_beta_gama = this->matrix2XYZEulerAngle(cal_NDI_marker);
		//std::cout << "cal_xyz_alpha_beta_gama:" << std::endl;
		//std::cout << cal_xyz_alpha_beta_gama << std::endl;
		//std::cout << std::endl;
		//取出Marker在NDI的坐标系
		Eigen::Matrix4d NDI_marker = NDI_matrix(Eigen::seq(4 * i, 4 * i + 3), Eigen::all);
		Eigen::VectorXd ndi_maker_alpha_beta_gama = this->matrix2XYZEulerAngle(NDI_marker);
		//std::cout << "ndi_maker_alpha_beta_gama:" << std::endl;
		//std::cout << ndi_maker_alpha_beta_gama << std::endl;
		//std::cout << std::endl;
		//保存他们之间的差
		difference_vector(Eigen::seq(6 * i, 6 * i + 5), Eigen::all) = ndi_maker_alpha_beta_gama - cal_xyz_alpha_beta_gama;
	}
	return difference_vector;
}

void Calibration::calculateIncrement(Eigen::MatrixXd joint_angle, Eigen::MatrixXd robot_marker_matrix, Eigen::MatrixXd NDI_matrix)
{
	Eigen::MatrixXd Jagobi_matrix = calculateMultiPointJacobiMatrix(joint_angle);
	//std::cout << "Jagobi_matrix:" << std::endl;
	//std::cout << Jagobi_matrix << std::endl;
	//std::cout << std::endl;

	//计算右边的偏差
	Eigen::VectorXd marker_loss_vector = calculateMultiPointDifference(robot_marker_matrix, NDI_matrix);
	//std::cout << "marker_loss_vector:" << std::endl;
	//std::cout << marker_loss_vector << std::endl;
	//std::cout << std::endl;
	Eigen::VectorXd increment_vector = (Jagobi_matrix.transpose()*Jagobi_matrix).inverse()*Jagobi_matrix.transpose()*marker_loss_vector;
	theta_increment += increment_vector(Eigen::seq(0, 5), Eigen::all);
	std::cout << "increment_vector:" << std::endl;
	std::cout << increment_vector << std::endl;
	std::cout << std::endl;
	int d_index[4] = { 0, 3, 4, 5 };
	for (int i = 0; i < 4; i++) {
		d_increment[d_index[i]] += increment_vector[i + 6];
	}
	int a_index[2] = { 1, 2 };
	for (int i = 0; i < 2; i++) {
		a_increment[a_index[i]] += increment_vector[i + 10];
	}
}

