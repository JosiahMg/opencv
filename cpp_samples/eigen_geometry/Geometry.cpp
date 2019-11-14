#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen ����ģ��
#include <Eigen/Geometry>
#include <iomanip>

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include<opencv2/core/eigen.hpp>

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif


/****************************
* ��������ʾ�� Eigen ����ģ���ʹ�÷���
****************************/

#define M_PI       3.14159265358979323846

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta);

int main(int argc, char** argv)
{
	// Eigen/Geometry ģ���ṩ�˸�����ת��ƽ�Ƶı�ʾ
	// 3D ��ת����ֱ��ʹ�� Matrix3d �� Matrix3f
	Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	// ��ת����ʹ�� AngleAxis, ���ײ㲻ֱ����Matrix����������Ե���������Ϊ�������������
	Eigen::AngleAxisd rotation_vector(M_PI / 2, Eigen::Vector3d(0, 1, 0));     //�� Z ����ת 45 ��
	cout.precision(3);
	cout << "rotation_vector axis= \n" << rotation_vector.axis() << "\n rotation angle= " << rotation_vector.angle() << endl;


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
	T.rotate(rotation_matrix);                                     // ����rotation_vector������ת
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
	cout << endl << endl;

	//ʹ��Eigenʵ��ŷ���ǵ���ת�����ת��
	Eigen::Matrix3d rotation;
	Eigen::Vector3d eular_angle(0.2*M_PI, 0.3*M_PI, 0.4*M_PI);
	rotation = Eigen::AngleAxisd(eular_angle[2], Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(eular_angle[1], Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(eular_angle[0], Eigen::Vector3d::UnitX());

	cout << rotation << endl;

	cv::Vec3d eular(0.2*M_PI, 0.3*M_PI, 0.4*M_PI);
	cout << setprecision(3) << eulerAnglesToRotationMatrix(eular) << endl;
	//ʹ��opencvʵ��ŷ���ǵ���ת�����ת��
	Eigen::Matrix3d rot;
	cv::Vec3d theta(0.2*M_PI, 0.3*M_PI, 0.4*M_PI);
	cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta[0]), -sin(theta[0]), 0, sin(theta[0]), cos(theta[0]));
	cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(theta[1]), 0, sin(theta[1]), 0, 1, 0, -sin(theta[1]), 0, cos(theta[1]));
	cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(theta[2]), -sin(theta[2]), 0, sin(theta[2]), cos(theta[2]), 0, 0, 0, 1);
	cv::Mat R = R_z * R_y * R_x;
	cv::cv2eigen(R, rot);
	cout << rot << endl;

	//ͨ����ת�����ȡ��z y x ���ϵ���ת���ȣ�����ŵ�ea�С�
	Eigen::Vector3d ea = rot.eulerAngles(2, 1, 0);
	cout << ea << endl;

	cout << eulerAnglesToRotationMatrix(eular);



	return 0;
}

/**
ŷ���Ǽ����Ӧ����ת����
**/
cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta)
{
	// ������ת�����X����
	cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cos(theta[0]), -sin(theta[0]),
		0, sin(theta[0]), cos(theta[0])
		);

	// ������ת�����Y����
	cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);

	// ������ת�����Z����
	cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
		cos(theta[2]), -sin(theta[2]), 0,
		sin(theta[2]), cos(theta[2]), 0,
		0, 0, 1);

	// �ϲ�
	cv::Mat R = R_z * R_y * R_x;
	return R;
}