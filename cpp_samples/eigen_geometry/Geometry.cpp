#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>
#include <iomanip>

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include<opencv2/core/eigen.hpp>

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif


/****************************
* 本程序演示了 Eigen 几何模块的使用方法
****************************/

#define M_PI       3.14159265358979323846

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta);

int main(int argc, char** argv)
{
	// Eigen/Geometry 模块提供了各种旋转和平移的表示
	// 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
	Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	// 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
	Eigen::AngleAxisd rotation_vector(M_PI / 2, Eigen::Vector3d(0, 1, 0));     //沿 Z 轴旋转 45 度
	cout.precision(3);
	cout << "rotation_vector axis= \n" << rotation_vector.axis() << "\n rotation angle= " << rotation_vector.angle() << endl;


	cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;                //用matrix()转换成矩阵
																					  // 也可以直接赋值
	rotation_matrix = rotation_vector.toRotationMatrix();
	// 用 AngleAxis 可以进行坐标变换
	Eigen::Vector3d v(1, 0, 0);
	Eigen::Vector3d v_rotated = rotation_vector * v;
	cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
	// 或者用旋转矩阵
	v_rotated = rotation_matrix * v;
	cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

	// 欧拉角: 可以将旋转矩阵直接转换成欧拉角
	Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序，即roll pitch yaw顺序
	cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

	// 欧氏变换矩阵使用 Eigen::Isometry  齐次形式
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
	T.rotate(rotation_matrix);                                     // 按照rotation_vector进行旋转
	T.pretranslate(Eigen::Vector3d(1, 3, 4));                     // 把平移向量设成(1,3,4)
	cout << "Transform matrix = \n" << T.matrix() << endl;

	// 用变换矩阵进行坐标变换
	Eigen::Vector3d v_transformed = T*v;                              // 相当于R*v+t
	cout << "v tranformed = " << v_transformed.transpose() << endl;

	// 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

	// 四元数
	// 可以直接把AngleAxis赋值给四元数，反之亦然
	Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
	cout << "quaternion = \n" << q.coeffs() << endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
													   // 也可以把旋转矩阵赋给它
	q = Eigen::Quaterniond(rotation_matrix);
	cout << "quaternion = \n" << q.coeffs() << endl;
	// 使用四元数旋转一个向量，使用重载的乘法即可
	v_rotated = q*v; // 注意数学上是qvq^{-1}
	cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
	cout << endl << endl;

	//使用Eigen实现欧拉角到旋转矩阵的转换
	Eigen::Matrix3d rotation;
	Eigen::Vector3d eular_angle(0.2*M_PI, 0.3*M_PI, 0.4*M_PI);
	rotation = Eigen::AngleAxisd(eular_angle[2], Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(eular_angle[1], Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(eular_angle[0], Eigen::Vector3d::UnitX());

	cout << rotation << endl;

	cv::Vec3d eular(0.2*M_PI, 0.3*M_PI, 0.4*M_PI);
	cout << setprecision(3) << eulerAnglesToRotationMatrix(eular) << endl;
	//使用opencv实现欧拉角到旋转矩阵的转换
	Eigen::Matrix3d rot;
	cv::Vec3d theta(0.2*M_PI, 0.3*M_PI, 0.4*M_PI);
	cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta[0]), -sin(theta[0]), 0, sin(theta[0]), cos(theta[0]));
	cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(theta[1]), 0, sin(theta[1]), 0, 1, 0, -sin(theta[1]), 0, cos(theta[1]));
	cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(theta[2]), -sin(theta[2]), 0, sin(theta[2]), cos(theta[2]), 0, 0, 0, 1);
	cv::Mat R = R_z * R_y * R_x;
	cv::cv2eigen(R, rot);
	cout << rot << endl;

	//通过旋转矩阵获取在z y x 轴上的旋转弧度，并存放到ea中。
	Eigen::Vector3d ea = rot.eulerAngles(2, 1, 0);
	cout << ea << endl;

	cout << eulerAnglesToRotationMatrix(eular);



	return 0;
}

/**
欧拉角计算对应的旋转矩阵
**/
cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta)
{
	// 计算旋转矩阵的X分量
	cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cos(theta[0]), -sin(theta[0]),
		0, sin(theta[0]), cos(theta[0])
		);

	// 计算旋转矩阵的Y分量
	cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);

	// 计算旋转矩阵的Z分量
	cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
		cos(theta[2]), -sin(theta[2]), 0,
		sin(theta[2]), cos(theta[2]), 0,
		0, 0, 1);

	// 合并
	cv::Mat R = R_z * R_y * R_x;
	return R;
}