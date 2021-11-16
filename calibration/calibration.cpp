#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigen>
using namespace Eigen;
using namespace std;

int main()
{
#pragma  region  Addition and subtraction
	Matrix2d a;
	a << 1, 2,
		3, 4;
	MatrixXd b(2, 2);
	b << 2, 3,
		1, 4;
	std::cout << "a + b =\n" << a + b << std::endl;
	std::cout << "a - b =\n" << a - b << std::endl;
	std::cout << "Doing a += b;" << std::endl;
	a += b;
	std::cout << "Now a =\n" << a << std::endl;
	Vector3d v(1, 2, 3);
	Vector3d w(1, 0, 0);
	std::cout << "-v + w - v =\n" << -v + w - v << std::endl;
#pragma endregion

#pragma  region  Scalar multiplication and division
	//	Matrix2d a;  //duplicate definition
	a << 1, 2,
		3, 4;
	//	Vector3d v(1, 2, 3);   //duplicate definition
	std::cout << "a * 2.5 =\n" << a * 2.5 << std::endl;
	std::cout << "0.1 * v =\n" << 0.1 * v << std::endl;
	std::cout << "Doing v *= 2;" << std::endl;
	v *= 2;
	std::cout << "Now v =\n" << v << std::endl;
#pragma endregion

#pragma  region  Transposition and conjugation
	MatrixXcf a_matrix = MatrixXcf::Random(2, 2);
	cout << "Here is the matrix a_matrix\n" << a_matrix << endl;
	cout << "Here is the matrix a_matrix^T\n" << a_matrix.transpose() << endl;
	cout << "Here is the conjugate of a_matrix\n" << a_matrix.conjugate() << endl;
	cout << "Here is the matrix a_matrix^*\n" << a_matrix.adjoint() << endl;

	//This is the so-called aliasing issue
	Matrix2i a_matrix2;
	a_matrix2 << 1, 2, 3, 4;
	cout << "Here is the matrix a_matrix2:\n" << a_matrix2 << endl;
	// a_matrix2 = a_matrix2.transpose(); // !!! do NOT do this !!!
	cout << "and the result of the aliasing effect:\n" << a_matrix2 << endl;
#pragma endregion

#pragma  region  Matrix-matrix and matrix-vector multiplication
	Matrix2d mat;
	mat << 1, 2,
		3, 4;
	Vector2d u_1(-1, 1), v_1(2, 0);
	std::cout << "Here is mat*mat:\n" << mat * mat << std::endl;
	std::cout << "Here is mat*u_1:\n" << mat * u_1 << std::endl;
	std::cout << "Here is u_1^T*mat:\n" << u_1.transpose()*mat << std::endl;
	std::cout << "Here is u_1^T*v:\n" << u_1.transpose()*v_1 << std::endl;
	std::cout << "Here is u_1*v_1^T:\n" << u_1 * v_1.transpose() << std::endl;
	std::cout << "Let's multiply mat by itself" << std::endl;
	mat = mat * mat;
	std::cout << "Now mat is mat:\n" << mat << std::endl;
#pragma endregion  

#pragma  region  Dot product and cross product
	Vector3d v_2(1, 2, 3);
	Vector3d w_2(0, 1, 2);
	cout << "Dot product: " << v_2.dot(w_2) << endl;
	double dp = v_2.adjoint()*w_2; // automatic conversion of the inner product to a scalar
	cout << "Dot product via a matrix product: " << dp << endl;
	cout << "Cross product:\n" << v_2.cross(w_2) << endl;
#pragma endregion

#pragma region   Basic arithmetic reduction operations
	Eigen::Matrix2d mat_3;
	mat_3 << 1, 2,
		3, 4;
	cout << "Here is mat_3.sum():       " << mat_3.sum() << endl;
	cout << "Here is mat_3.prod():      " << mat_3.prod() << endl;
	cout << "Here is mat_3.mean():      " << mat_3.mean() << endl;
	cout << "Here is mat_3.minCoeff():  " << mat_3.minCoeff() << endl;
	cout << "Here is mat_3.maxCoeff():  " << mat_3.maxCoeff() << endl;
	cout << "Here is mat_3.trace():     " << mat_3.trace() << endl;

	Matrix3f m = Matrix3f::Random();
	std::ptrdiff_t i, j;
	float minOfM = m.minCoeff(&i, &j);
	cout << "Here is the matrix m:\n" << m << endl;
	cout << "Its minimum coefficient (" << minOfM
		<< ") is at position (" << i << "," << j << ")\n\n";
	RowVector4i v_4 = RowVector4i::Random();
	int maxOfV = v_4.maxCoeff(&i);
	cout << "Here is the vector v_4: " << v_4 << endl;
	cout << "Its maximum coefficient (" << maxOfV
		<< ") is at position " << i << endl;
#pragma endregion

	/**************This is the end of example codes in Eigen3 online document. **********************/
	system("pause");
}