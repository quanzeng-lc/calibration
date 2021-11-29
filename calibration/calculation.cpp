#include "calculation.h"
#include <iostream>

calculation::calculation(int numInit, int numOffset)
{
    this->numInit_ = numInit;
	this->numOffset_ = numOffset;
}


Eigen::Matrix3d calculation::calculateOneceRbo(Eigen::MatrixXd ndiData)
{
	//NDI获得的数据 0-3是四元数的姿态 4-6位是坐标位置
	Eigen::Vector3d init = ndiData(0, Eigen::seq(4, 6));

	Eigen::Vector3d Rbo_x = Eigen::Vector3d::Zero();
	Eigen::Vector3d Rbo_y = Eigen::Vector3d::Zero();
	Eigen::Vector3d Rbo_z = Eigen::Vector3d::Zero();

	for (int offset = 1; offset <= numOffset_; offset++) {
		Eigen::Vector3d p = ndiData(offset, Eigen::seq(4, 6));
		Eigen::Vector3d diff = (p - init);
		double norm = std::sqrt(diff.transpose()*diff);
		Rbo_x += (diff / norm);
	}
	Rbo_x = Rbo_x / numOffset_;

	for (int offset = 1; offset <= numOffset_; offset++) {
		Eigen::Vector3d p = ndiData(numOffset_ + offset, Eigen::seq(4, 6));
		Eigen::Vector3d diff = p - init;
		double norm = std::sqrt(diff.transpose()*diff);
		Rbo_y += (diff / norm);
	}
	Rbo_y = Rbo_y / numOffset_;

	for (int offset = 1; offset <= numOffset_; offset++) {
		Eigen::Vector3d p = ndiData(2 * numOffset_ + offset, Eigen::seq(4, 6));
		Eigen::Vector3d diff = p - init;
		double norm = std::sqrt(diff.transpose()*diff);
		Rbo_z += (diff / norm);
	}
	Rbo_z = Rbo_z / numOffset_;

	Eigen::Matrix3d Rbo({ {Rbo_x(0), Rbo_y(0), Rbo_z(0)},
						  {Rbo_x(1), Rbo_y(1), Rbo_z(1)},
						  {Rbo_x(2), Rbo_y(2), Rbo_z(2)} });
	//std::cout << Rbo_x << std::endl;
	//std::cout << Rbo_y << std::endl;
	//std::cout << Rbo_z << std::endl;
	//std::cout << Rbo << std::endl;
	return Rbo;
}

void calculation::calculateRbo(Eigen::MatrixXd ndiData)
{
	Eigen::Matrix3d Rbo(3, 3);
	Rbo = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d RboInitPos;

	//将每一个初始点移动之后的数据放入calculateOneceReo函数中
	//计算每一个初始点位的Reo，放入Reo_中
	for (int i = 0; i < numInit_; i++)
	{
		RboInitPos = this->calculateOneceRbo(\
			ndiData(Eigen::seq(i * (3 * numOffset_ + 1), (i + 1) * 3 * numOffset_ + i), Eigen::all));

		Rbo += RboInitPos;
	}

	Rbo_ = Rbo / numInit_;
}

Eigen::Matrix3d calculation::calculateOnceReo(Eigen::MatrixXd ndiData)
{
	Eigen::Vector3d init = ndiData(0, Eigen::seq(4, 6));

	Eigen::Vector3d Reo_x = Eigen::Vector3d::Zero();
	Eigen::Vector3d Reo_y = Eigen::Vector3d::Zero();
	Eigen::Vector3d Reo_z = Eigen::Vector3d::Zero();

	for (int offset = 1; offset <= numOffset_; offset++) {
		Eigen::Vector3d p = ndiData(offset, Eigen::seq(4, 6));
		Eigen::Vector3d diff = (p - init);
		double norm = std::sqrt(diff.transpose()*diff);
		Reo_x += (diff / norm);
	}
	Reo_x = Reo_x / numOffset_;

	for (int offset = 1; offset <= numOffset_; offset++) {
		Eigen::Vector3d p = ndiData(numOffset_ + offset, Eigen::seq(4, 6));
		Eigen::Vector3d diff = p - init;
		double norm = std::sqrt(diff.transpose()*diff);
		Reo_y += (diff / norm);
	}
	Reo_y = Reo_y / numOffset_;

	for (int offset = 1; offset <= numOffset_; offset++) {
		Eigen::Vector3d p = ndiData(2 * numOffset_ + offset, Eigen::seq(4, 6));
		Eigen::Vector3d diff = p - init;
		double norm = std::sqrt(diff.transpose()*diff);
		Reo_z += (diff / norm);
	}
	Reo_z = Reo_z / numOffset_;

	Eigen::Matrix3d Reo({ {Reo_x(0), Reo_y(0), Reo_z(0)},
						  {Reo_x(1), Reo_y(1), Reo_z(1)},
						  {Reo_x(2), Reo_y(2), Reo_z(2)} });

	return Reo;
}

void calculation::calculateReo(Eigen::MatrixXd ndiData)
{
	Eigen::MatrixXd Reo(0, 3);
	Eigen::Matrix3d ReoInitPos = Eigen::Matrix3d::Zero();

	//将每一个初始点移动之后的数据放入calculateOneceReo函数中
	//计算每一个初始点位的Reo，放入Reo_中
	for (int i = 0; i < numInit_; i++)
	{
		ReoInitPos = this->calculateOnceReo(\
			ndiData(Eigen::seq(i * (3 * numOffset_ + 1), (i + 1) * 3 * numOffset_ + i), Eigen::all));

		Reo.conservativeResize(Reo.rows() + 3, Reo.cols());
		Reo.block(3 * i, 0, 3, 3) = ReoInitPos;
	}

	Reo_ = Reo;
}

void calculation::setRtoTto(Eigen::MatrixXd matrixE)
{
	//matrixE每一行是一个点位的NDI数据
	Rto_.conservativeResize(3 * numInit_, 3);
	Tto_.conservativeResize(3 * numInit_, 1);
	for (int i = 0; i < numInit_; i++) {
		Eigen::Vector4d vector_quater = matrixE.block(i * 3 * numOffset_ + i, 0, 1, 4).transpose();
		Eigen::Quaterniond quater(vector_quater[0], vector_quater[1], vector_quater[2], vector_quater[3]);
		Rto_.block(3 * i, 0, 3, 3) = quater.matrix();
		Tto_.block(3*i, 0, 3, 1) = matrixE.block(i * 3 * numOffset_ + i, 4, 1, 3).transpose();
	}
}

void calculation::setRebTeb(Eigen::MatrixXd matrixERobot)
{
	Reb_.conservativeResize(3 * numInit_, 3);
	Teb_.conservativeResize(3 * numInit_, 1);
	for (int i = 0; i < numInit_; i++) {
		Eigen::Vector3d p = matrixERobot.block(i * 3 * numOffset_ + i, 3, 1, 3).transpose();
		Eigen::AngleAxisd angleAxis;
		double angle = sqrt(p.transpose()*p);
		if (angle == 0) {
			angleAxis = Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1));
		}
		else
		{
			 angleAxis = Eigen::AngleAxisd(angle, p/angle);
		}
		Reb_.block(3 * i, 0, 3, 3) = angleAxis.matrix();
		Teb_.block(3 * i, 0, 3, 1) = matrixERobot.block(i * 3 * numOffset_ + i, 0, 1, 3).transpose();
	}
}

void calculation::calculateRet()
{
    Eigen::Matrix3d Ret = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Rto = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Reo = Eigen::Matrix3d::Zero();
    for (int i = 0; i < this->numInit_; i++)
    {
        Rto = Rto_.block(3 * i, 0, 3, 3);
        Reo = Reo_.block(3 * i, 0, 3, 3);

        Ret += Rto.inverse()* Reo;
    }

    Ret_ = Ret / this->numInit_;
}

void calculation::calculateTetAndTbo()
{
	Eigen::Vector3d Pe = { 1, 1, 1 };
	Eigen::MatrixXd A(3 * numInit_, 6);
	Eigen::MatrixXd C(3 * numInit_, 1);
	Eigen::MatrixXd U(2 * 3, 1);
	for (int i = 0; i < numInit_; i++) {
		Eigen::MatrixXd Ai(3, 6);
		Eigen::Matrix3d identify;
		identify.setIdentity();
		Ai.block(0, 0, 3, 3) = -1 * identify;
		Ai.block(0, 3, 3, 3) = Rto_.block(3 * i, 0, 3, 3);
		A.block(3 * i, 0, 3, 6) = Ai;

		Eigen::MatrixXd Ci(3, 1);
		Ci = Rbo_ * Reb_.block(3 * i, 0, 3, 3)*Pe + Rbo_ * Teb_.block(3 * i, 0, 3, 1) -
			Rto_.block(3 * i, 0, 3, 3)*Ret_*Pe - Tto_.block(3 * i, 0, 3, 1);
		C.block(3 * i, 0, 3, 1) = Ci;
	};
	Eigen::MatrixXd general_inverse = (A.transpose()*A).inverse()*A.transpose();
	U = general_inverse * C;
	Tbo_ = U.block(0, 0, 3, 1);
	Tet_ = U.block(3, 0, 3, 1);
}


Eigen::Matrix3d calculation::getRbo()
{
	return this->Rbo_;
}

Eigen::MatrixXd calculation::getReo()
{
	return this->Reo_;
}

Eigen::MatrixXd calculation::getRto()
{
	return this->Rto_;
}

Eigen::MatrixXd calculation::getTto()
{
	return this->Tto_;
}

Eigen::Matrix3d calculation::getRet()
{
	return this->Ret_;
}

Eigen::MatrixXd calculation::getReb()
{
	return this->Reb_;
}

Eigen::MatrixXd calculation::getTeb()
{
	return this->Teb_;
}

Eigen::MatrixXd calculation::getTet()
{
	return this->Tet_;
}

Eigen::MatrixXd calculation::getTbo()
{
	return this->Tbo_;
}


Eigen::MatrixXd calculation::RobotToTransformationMatrix(Eigen::MatrixXd robotMaxtrix) {
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
		Eigen::Matrix3d rotation =  angleaxis.matrix();
		tranaformationMatrix.block(4 * i, 0, 3, 3) = rotation;
		tranaformationMatrix.block(4 * i, 3, 3, 1) = translationMaxtrix.block(i, 0, 1, 3).transpose();
		tranaformationMatrix(4 * i + 3, 3) = 1.0;
	}
	return tranaformationMatrix;
}
