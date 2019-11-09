#include <iostream>
#include "Eigen/Dense"

int main()
{
	// 矩阵的定义
	Eigen::MatrixXd m(2, 2);
	Eigen::Vector4d vec4d(1.0, 2.0, 3.0, 4.0);

	m(0, 0) = 1;
	m(0, 1) = 2;
	m(1, 0) = 3;
	m(1, 1) = 4;
	std::cout << m << std::endl << std::endl;
	std::cout << vec4d << std::endl << std::endl;

	int row = 4;
	int col = 5;
	Eigen::MatrixXf matrixXf(row, col);
	matrixXf << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20;
	std::cout << matrixXf << std::endl << std::endl;


	Eigen::MatrixXd matrixXd1(3, 3);
	matrixXd1 = Eigen::Matrix3d::Random();
	std::cout << matrixXd1 << std::endl << std::endl;

	//求矩阵的转置、共轭矩阵、伴随矩阵  
	std::cout << m.transpose() << std::endl << std::endl;
	std::cout << m.conjugate() << std::endl << std::endl;
	std::cout << m.adjoint() << std::endl << std::endl;
	std::cout << m << std::endl << std::endl;
	m.transposeInPlace();
	std::cout << m << std::endl << std::endl;


	//求解矩阵的特征值和特征向量  
	Eigen::Matrix2f matrix2f;
	matrix2f << 1, 2, 3, 4;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigenSolver(matrix2f);
	if (eigenSolver.info() == Eigen::Success) {
		std::cout << eigenSolver.eigenvalues() << std::endl << std::endl;
		std::cout << eigenSolver.eigenvectors() << std::endl << std::endl;
	}



	return 0;

}