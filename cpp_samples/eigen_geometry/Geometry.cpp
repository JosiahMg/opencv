#include <iostream>
#include <cmath>

#include <Eigen/Core>
// Eigen ����ģ��
#include <Eigen/Geometry>
#include <iomanip>

using namespace std;


/****************************
* ��������ʾ�� Eigen ����ģ���ʹ�÷���
****************************/

#define M_PI       3.14159265358979323846

//��ת���� -> other
void AngleAxisdToOther(Eigen::AngleAxisd& AgVec);
//��ת���� -> other
void Matrix3dToOther(Eigen::Matrix3d& rt_mat);
//��֪ŷ����(rx, ry, rz)
void eulerAngleToOther(Eigen::Vector3d& eulerAngle);
//��Ԫ��
void quaterniondToOther(Eigen::Quaterniond& quaternion);
//������������ά�����ռ�任
Eigen::Vector3d Isometry(Eigen::Matrix3d mat, Eigen::Vector3d vector, Eigen::Vector3d& point);


int main(int argc, char** argv)
{
	//��ת����-> other
	Eigen::AngleAxisd rotation_vector(M_PI /4, Eigen::Vector3d(0, 0, 1));     //�� Z ����ת 45 ��
	cout << "axis= \n" << rotation_vector.axis() << "\n angle= " << rotation_vector.angle() << endl;
	AngleAxisdToOther(rotation_vector);


	//��ת���� -> other
	// 3D ��ת����ֱ��ʹ�� Matrix3d �� Matrix3f
	Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	rotation_matrix << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	Matrix3dToOther(rotation_matrix);

	//ŷ���� -> other
	Eigen::Vector3d eular_angle(0.2*M_PI, 0.3*M_PI, 0.4*M_PI);
	eulerAngleToOther(eular_angle);

	//��Ԫ��-> other (w,x,y,z) (1,0,0,0)�൱��û����ת��ƽ��
	Eigen::Quaterniond quaternion(1., 0., 0., 0.);
	quaterniondToOther(quaternion);


	Eigen::Vector3d pt_r;
	pt_r = Isometry(rotation_matrix, Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(5, 3, 1));
	cout << pt_r << endl;

	return 0;
}

//��֪��ת����

void AngleAxisdToOther(Eigen::AngleAxisd& AgVec)
{
	//ת������ת����
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix = AgVec.matrix();
	cout << "rotation matrix = " << rotation_matrix << std::endl;

	//ת����ŷ����   XYZ˳�򣬼�roll pitch yaw˳��
	Eigen::Vector3d eulerAngle = AgVec.matrix().eulerAngles(0, 1, 2);
	cout << "euler angles = " << eulerAngle << std::endl;

	//ת������Ԫ��
	Eigen::Quaterniond quaternion(AgVec);
	cout << "quaternion = " << quaternion.coeffs() << std::endl;

}

//��֪��ת����
void Matrix3dToOther(Eigen::Matrix3d& rt_mat)
{
	//ת������ת����
	Eigen::AngleAxisd rotation_vector(rt_mat);
	cout << "rotation_vector angle= " << rotation_vector.angle() << endl;
	cout << "rotation_vector axis= " << rotation_vector.axis() << endl;

	//ת����ŷ����
	Eigen::Vector3d eulerAngle = rt_mat.eulerAngles(0, 1, 2);
	cout << "eulerAngle=" << eulerAngle << endl;

	//ת������Ԫ��
	Eigen::Quaterniond quaternion(rt_mat);
	cout << "quaternion = " << quaternion.coeffs() << endl;
}

//��֪ŷ����(rx, ry, rz)
void eulerAngleToOther(Eigen::Vector3d& eulerAngle)
{
	//ת������ת����
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));

	Eigen::AngleAxisd rotation_vector;
	rotation_vector = yawAngle*pitchAngle*rollAngle;
	cout << "angle = " << rotation_vector.angle() << endl;
	cout << "axis = " << rotation_vector.axis() << endl;

	//ת������ת����
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix = yawAngle*pitchAngle*rollAngle;
	cout << "rotation_matrix = " << rotation_matrix << endl;

	//ת������Ԫ��
	Eigen::Quaterniond quaternion;
	quaternion = yawAngle*pitchAngle*rollAngle;
	cout << "quaternion = " << quaternion.coeffs() << endl;

}

//��Ԫ��
void quaterniondToOther(Eigen::Quaterniond& quaternion)
{
	//ת������ת����
	Eigen::AngleAxisd rotation_vector;
	rotation_vector = quaternion;
	std::cout << "axis = " << rotation_vector.axis() << std::endl;
	std::cout << "angle = " << rotation_vector.angle() << std::endl;

	//ת������ת����
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix = quaternion.matrix();
	std::cout << "rotation_matrix = " << rotation_matrix << std::endl;

	//ת����ŷ����
	Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(0, 1, 2);
	cout << "eulerAngles=" << eulerAngle << endl;
}

/*�������4*4 ������ά���꣬����ƽ����ת�����ά����
*  mat : ��ת����
*  vector �� ƽ������
*  point �� ��ά�����
*  return : ��ά�����
*/
Eigen::Vector3d Isometry(Eigen::Matrix3d mat, Eigen::Vector3d vector, Eigen::Vector3d& point)
{
	// ŷ�ϱ任����ʹ�� Eigen::Isometry  �����ʽ
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();                // ��Ȼ��Ϊ3d��ʵ������4��4�ľ���
	T.rotate(mat);                                     // ����rotation_vector������ת
	T.pretranslate(vector);                     // ��ƽ���������(1,3,4)
	cout << "Transform matrix = \n" << T.matrix() << endl;

	// �ñ任�����������任
	Eigen::Vector3d v_transformed = T*point;                              // �൱��R*v+t
	cout << "v tranformed = " << v_transformed.transpose() << endl;
	return v_transformed;
}
