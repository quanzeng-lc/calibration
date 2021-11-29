#include <iostream>

#include "calculation.h"
#include "caliData.h"

int main()
{
    caliData classData;
    classData.readMatrixB("..\\x64\\Debug\\calibration_data\\20211125\\BaseNDIData.csv");
	classData.readMatrixE("..\\x64\\Debug\\calibration_data\\20211125\\EffectorNDIData.csv");
	classData.readMatrixERobot("..\\x64\\Debug\\calibration_data\\20211125\\EffectorRobotData.csv");
    Eigen::MatrixXd matrixB = classData.getMatrixB();
    Eigen::MatrixXd matrixE = classData.getMatrixE();
	Eigen::MatrixXd matrixERoboto = classData.getMatrixERobot();

	calculation Cal(6, 5);

	Cal.calculateRbo(matrixB);
	auto Rbo = Cal.getRbo();
	std::cout << "Rbo:" << std::endl;
	std::cout << Rbo << std::endl;
	std::cout << std::endl;

	Cal.calculateReo(matrixE);
	auto Reo = Cal.getReo();
	Cal.setRtoTto(matrixE);
	auto Rto = Cal.getRto();

	Cal.calculateRet();
	auto Ret = Cal.getRet();
	std::cout << "Ret:" << std::endl;
	std::cout << Ret << std::endl;
	std::cout << std::endl;

	//通过机器人获得数据得到 Reb 和 Teb
	Cal.setRebTeb(matrixERoboto);

	Cal.calculateTetAndTbo();
	auto Tet = Cal.getTet();
	std::cout << "Tet:" << std::endl;
	std::cout << Tet << std::endl;
	std::cout << std::endl;

	auto Tbo = Cal.getTbo();
	std::cout << "Tbo:" << std::endl;
	std::cout << Tbo << std::endl;
	std::cout << std::endl;

	Eigen::Matrix4d Vet;
	Vet.setZero();
	Vet.block(0, 0, 3, 3) = Ret;
	Vet.block(0, 3, 3, 1) = Tet;
	Vet(3, 3) = 1.0;

	Eigen::Matrix4d Vbo;
	Vbo.setZero();
	Vbo.block(0, 0, 3, 3) = Rbo;
	Vbo.block(0, 3, 3, 1) = Tbo;
	Vbo(3, 3) = 1.0;

	classData.readMatrixTestRobot("..\\x64\\Debug\\calibration_data\\20211125\\testRobotData.csv");
	classData.readMatrixTestNDI("..\\x64\\Debug\\calibration_data\\20211125\\testNDIData.csv");
	Eigen::MatrixXd matrixTestRobot = classData.getMatrixTestRobot();
	Eigen::MatrixXd matrixTestNDI = classData.getMatrixTestNDI();
	std::cout << "TestNDI:" << std::endl;
	std::cout << matrixTestNDI << std::endl;
	std::cout << std::endl;


	//末端坐标系的转换矩阵
	Eigen::MatrixXd baseEffector = Cal.RobotToTransformationMatrix(matrixTestRobot);
	//std::cout << "baseEffector:" << std::endl;
	//std::cout << baseEffector << std::endl;
	//std::cout << std::endl;

	Eigen::Matrix4d Vet_inverse = Vet.inverse();
	Eigen::MatrixXd calculateTool(baseEffector.rows(), baseEffector.cols());
	for (int i = 0; i < matrixTestRobot.rows(); i++) {
		Eigen::Matrix4d result = Vbo * baseEffector.block(4*i, 0, 4, 4) * Vet_inverse;
		calculateTool.block(4 * i, 0, 4, 4) = result;
	}
	std::cout << "calculateTool:" << std::endl;
	std::cout << calculateTool << std::endl;
	std::cout << std::endl;

    system("pause");
    return 0;
}
