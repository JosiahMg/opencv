#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen ����ģ��
#include <Eigen/Geometry>

#define M_PI 3.1415926
/****************************
* ��������ʾ�� Eigen ����ģ���ʹ�÷���
****************************/

int main(int argc, char** argv)
{
	// Eigen/Geometry ģ���ṩ�˸�����ת��ƽ�Ƶı�ʾ
	// 3D ��ת����ֱ��ʹ�� Matrix3d �� Matrix3f
	Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	// ��ת����ʹ�� AngleAxis, ���ײ㲻ֱ����Matrix����������Ե���������Ϊ�������������
	Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));     //�� Z ����ת 45 ��
	cout.precision(3);
	cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;                //��matrix()ת���ɾ���
																					  // Ҳ����ֱ�Ӹ�ֵ
	rotation_matrix = rotation_vector.toRotationMatrix();
	// �� AngleAxis ���Խ�������任
	Eigen::Vector3d v(1, 0, 0);
	Eigen::Vector3d v_rotated = rotation_vector * v;
	cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
	// ��������ת����
	v_rotated = rotation_matrix * v;
	cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

	// ŷ����: ���Խ���ת����ֱ��ת����ŷ����
	Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX˳�򣬼�roll pitch yaw˳��
	cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

	// ŷ�ϱ任����ʹ�� Eigen::Isometry  �����ʽ
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();                // ��Ȼ��Ϊ3d��ʵ������4��4�ľ���
	T.rotate(rotation_vector);                                     // ����rotation_vector������ת
	T.pretranslate(Eigen::Vector3d(1, 3, 4));                     // ��ƽ���������(1,3,4)
	cout << "Transform matrix = \n" << T.matrix() << endl;

	// �ñ任�����������任
	Eigen::Vector3d v_transformed = T*v;                              // �൱��R*v+t
	cout << "v tranformed = " << v_transformed.transpose() << endl;

	// ���ڷ������Ӱ�任��ʹ�� Eigen::Affine3d �� Eigen::Projective3d ���ɣ���

	// ��Ԫ��
	// ����ֱ�Ӱ�AngleAxis��ֵ����Ԫ������֮��Ȼ
	Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
	cout << "quaternion = \n" << q.coeffs() << endl;   // ��ע��coeffs��˳����(x,y,z,w),wΪʵ����ǰ����Ϊ�鲿
													   // Ҳ���԰���ת���󸳸���
	q = Eigen::Quaterniond(rotation_matrix);
	cout << "quaternion = \n" << q.coeffs() << endl;
	// ʹ����Ԫ����תһ��������ʹ�����صĳ˷�����
	v_rotated = q*v; // ע����ѧ����qvq^{-1}
	cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

	return 0;
}
