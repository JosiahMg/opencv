#include <iostream>
#include <cmath>

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>
#include <iomanip>

using namespace std;


/****************************
* 本程序演示了 Eigen 几何模块的使用方法
****************************/

#define M_PI       3.14159265358979323846

//旋转向量 -> other
void AngleAxisdToOther(Eigen::AngleAxisd& AgVec);
//旋转矩阵 -> other
void Matrix3dToOther(Eigen::Matrix3d& rt_mat);
//已知欧拉角(rx, ry, rz)
void eulerAngleToOther(Eigen::Vector3d& eulerAngle);
//四元数
void quaterniondToOther(Eigen::Quaterniond& quaternion);
//齐次坐标计算三维坐标点空间变换
Eigen::Vector3d Isometry(Eigen::Matrix3d mat, Eigen::Vector3d vector, Eigen::Vector3d& point);


int main(int argc, char** argv)
{
	//旋转向量-> other
	Eigen::AngleAxisd rotation_vector(M_PI /4, Eigen::Vector3d(0, 0, 1));     //沿 Z 轴旋转 45 度
	cout << "axis= \n" << rotation_vector.axis() << "\n angle= " << rotation_vector.angle() << endl;
	AngleAxisdToOther(rotation_vector);


	//旋转矩阵 -> other
	// 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
	Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	rotation_matrix << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	Matrix3dToOther(rotation_matrix);

	//欧拉角 -> other
	Eigen::Vector3d eular_angle(0.2*M_PI, 0.3*M_PI, 0.4*M_PI);
	eulerAngleToOther(eular_angle);

	//四元数-> other (w,x,y,z) (1,0,0,0)相当于没有旋转和平移
	Eigen::Quaterniond quaternion(1., 0., 0., 0.);
	quaterniondToOther(quaternion);


	Eigen::Vector3d pt_r;
	pt_r = Isometry(rotation_matrix, Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(5, 3, 1));
	cout << pt_r << endl;

	return 0;
}

//已知旋转向量

void AngleAxisdToOther(Eigen::AngleAxisd& AgVec)
{
	//转换成旋转矩阵
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix = AgVec.matrix();
	cout << "rotation matrix = " << rotation_matrix << std::endl;

	//转换成欧拉角   XYZ顺序，即roll pitch yaw顺序
	Eigen::Vector3d eulerAngle = AgVec.matrix().eulerAngles(0, 1, 2);
	cout << "euler angles = " << eulerAngle << std::endl;

	//转换成四元数
	Eigen::Quaterniond quaternion(AgVec);
	cout << "quaternion = " << quaternion.coeffs() << std::endl;

}

//已知旋转矩阵
void Matrix3dToOther(Eigen::Matrix3d& rt_mat)
{
	//转换成旋转向量
	Eigen::AngleAxisd rotation_vector(rt_mat);
	cout << "rotation_vector angle= " << rotation_vector.angle() << endl;
	cout << "rotation_vector axis= " << rotation_vector.axis() << endl;

	//转换成欧拉角
	Eigen::Vector3d eulerAngle = rt_mat.eulerAngles(0, 1, 2);
	cout << "eulerAngle=" << eulerAngle << endl;

	//转换成四元数
	Eigen::Quaterniond quaternion(rt_mat);
	cout << "quaternion = " << quaternion.coeffs() << endl;
}

//已知欧拉角(rx, ry, rz)
void eulerAngleToOther(Eigen::Vector3d& eulerAngle)
{
	//转换成旋转向量
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));

	Eigen::AngleAxisd rotation_vector;
	rotation_vector = yawAngle*pitchAngle*rollAngle;
	cout << "angle = " << rotation_vector.angle() << endl;
	cout << "axis = " << rotation_vector.axis() << endl;

	//转换成旋转矩阵
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix = yawAngle*pitchAngle*rollAngle;
	cout << "rotation_matrix = " << rotation_matrix << endl;

	//转换成四元数
	Eigen::Quaterniond quaternion;
	quaternion = yawAngle*pitchAngle*rollAngle;
	cout << "quaternion = " << quaternion.coeffs() << endl;

}

//四元数
void quaterniondToOther(Eigen::Quaterniond& quaternion)
{
	//转换成旋转向量
	Eigen::AngleAxisd rotation_vector;
	rotation_vector = quaternion;
	std::cout << "axis = " << rotation_vector.axis() << std::endl;
	std::cout << "angle = " << rotation_vector.angle() << std::endl;

	//转换成旋转矩阵
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix = quaternion.matrix();
	std::cout << "rotation_matrix = " << rotation_matrix << std::endl;

	//转换成欧拉角
	Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(0, 1, 2);
	cout << "eulerAngles=" << eulerAngle << endl;
}

/*齐次坐标4*4 传入三维坐标，返回平移旋转后的三维坐标
*  mat : 旋转矩阵
*  vector ： 平移向量
*  point ： 三维坐标点
*  return : 三维坐标点
*/
Eigen::Vector3d Isometry(Eigen::Matrix3d mat, Eigen::Vector3d vector, Eigen::Vector3d& point)
{
	// 欧氏变换矩阵使用 Eigen::Isometry  齐次形式
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
	T.rotate(mat);                                     // 按照rotation_vector进行旋转
	T.pretranslate(vector);                     // 把平移向量设成(1,3,4)
	cout << "Transform matrix = \n" << T.matrix() << endl;

	// 用变换矩阵进行坐标变换
	Eigen::Vector3d v_transformed = T*point;                              // 相当于R*v+t
	cout << "v tranformed = " << v_transformed.transpose() << endl;
	return v_transformed;
}
