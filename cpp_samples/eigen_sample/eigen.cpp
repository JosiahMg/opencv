#include <iostream>
#include <ctime>
// Eigen ����
#include <Eigen/Core>
// ���ܾ���Ĵ������㣨�棬����ֵ�ȣ�
#include <Eigen/Dense>

#define MATRIX_SIZE 50

using namespace std;

int main()
{

	// Eigen �����������;�����Eigen::Matrix������һ��ģ���ࡣ����ǰ��������Ϊ���������ͣ��У���
	// ����һ��2*3��float����
	Eigen::Matrix<float, 2, 3> matrix_23;

	// ͬʱ��Eigen ͨ�� typedef �ṩ������������ͣ������ײ�����Eigen::Matrix
	// ���� Vector3d ʵ������ Eigen::Matrix<double, 3, 1>������ά����
	Eigen::Vector3d v_3d;
	// ����һ����
	Eigen::Matrix<float, 3, 1> vd_3d;

	// Matrix3d ʵ������ Eigen::Matrix<double, 3, 3>
	Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); //��ʼ��Ϊ��
														 // �����ȷ�������С������ʹ�ö�̬��С�ľ���
	Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > matrix_dynamic;
	// ���򵥵�
	Eigen::MatrixXd matrix_x;
	// �������ͻ��кܶ࣬���ǲ�һһ�о�

	// �����Ƕ�Eigen��Ĳ���
	// �������ݣ���ʼ����
	matrix_23 << 1, 2, 3, 4, 5, 6;
	// ���
	cout << matrix_23 << endl;

	// ��()���ʾ����е�Ԫ��
	for (int i = 0; i<2; i++) {
		for (int j = 0; j<3; j++)
			cout << matrix_23(i, j) << "\t";
		cout << endl;
	}

	// �����������ˣ�ʵ�������Ǿ���;���
	v_3d << 3, 2, 1;
	vd_3d << 4, 5, 6;
	// ������Eigen���㲻�ܻ�����ֲ�ͬ���͵ľ����������Ǵ��
	// Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
	// Ӧ����ʽת��
	Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
	cout << result << endl;

	Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
	cout << result2 << endl;

	// ͬ���㲻�ܸ������ά��
	// ����ȡ�������ע�ͣ�����Eigen�ᱨʲô��
	// Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

	// һЩ��������
	// ��������Ͳ���ʾ�ˣ�ֱ����+-*/���ɡ�
	matrix_33 = Eigen::Matrix3d::Random();      // ���������
	cout << matrix_33 << endl << endl;

	cout << matrix_33.transpose() << endl;      // ת��
	cout << matrix_33.sum() << endl;            // ��Ԫ�غ�
	cout << matrix_33.trace() << endl;          // ��
	cout << 10 * matrix_33 << endl;               // ����
	cout << matrix_33.inverse() << endl;        // ��
	cout << matrix_33.determinant() << endl;    // ����ʽ

												// ����ֵ
												// ʵ�Գƾ�����Ա�֤�Խǻ��ɹ�
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose()*matrix_33);
	cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
	cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;
	 
	// �ⷽ��
	// ������� matrix_NN * x = v_Nd �������
	// N�Ĵ�С��ǰ�ߵĺ��ﶨ�壬�������������
	// ֱ��������Ȼ����ֱ�ӵģ�����������������

	Eigen::Matrix< double, MATRIX_SIZE, MATRIX_SIZE > matrix_NN;
	matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
	Eigen::Matrix< double, MATRIX_SIZE, 1> v_Nd;
	v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

	clock_t time_stt = clock(); // ��ʱ
								// ֱ������
	Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse()*v_Nd;
	cout << "time use in normal inverse is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;

	// ͨ���þ���ֽ���������QR�ֽ⣬�ٶȻ��ܶ�
	time_stt = clock();
	x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
	cout << "time use in Qr decomposition is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;



	return 0;

}