// My_handeye.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "411"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")

#endif


using namespace cv;
using namespace std;

Mat eulerAnglesToRotationMatrix(Vec3f &theta);
Vec3f rotationMatrixToEulerAngles(Mat &R);
float rad2deg(float radian);
float deg2rad(float degree);

int main()
{

	// Camera calibration information

	std::vector<double> distortionCoefficients(5);  // camera distortion
	distortionCoefficients[0] = -0.05905;
	distortionCoefficients[1] = 0.2999;
	distortionCoefficients[2] = -1.35e-03;
	distortionCoefficients[3] = 2.72e-04;
	distortionCoefficients[4] = 0.1578;

	double f_x = 2563.1; // Focal length in x axis
	double f_y = 2578; // Focal length in y axis (usually the same?)
	double c_x = 1453.7; // Camera primary point x
	double c_y = 1069.2; // Camera primary point y

	cv::Mat cameraMatrix(3, 3, CV_32FC1);
	cameraMatrix.at<float>(0, 0) = f_x;
	cameraMatrix.at<float>(0, 1) = 0.0;
	cameraMatrix.at<float>(0, 2) = c_x;
	cameraMatrix.at<float>(1, 0) = 0.0;
	cameraMatrix.at<float>(1, 1) = f_y;
	cameraMatrix.at<float>(1, 2) = c_y;
	cameraMatrix.at<float>(2, 0) = 0.0;
	cameraMatrix.at<float>(2, 1) = 0.0;
	cameraMatrix.at<float>(2, 2) = 1.0;

	Mat rvec(3, 1, CV_32F), tvec(3, 1, CV_32F);
	//

	std::vector<Mat> R_gripper2base;
	std::vector<Mat> t_gripper2base;
	std::vector<Mat> R_target2cam;
	std::vector<Mat> t_target2cam;
	Mat R_cam2gripper = (Mat_<float>(3, 3));
	Mat t_cam2gripper = (Mat_<float>(3, 1));

	vector<String> fn;
	glob("../../images/chessboard/*.bmp", fn, false);

	vector<Mat> images;
	size_t num_images = fn.size(); //number of bmp files in images folder
	Size patternsize(13, 9); //number of centers
	vector<Point2f> centers; //this will be filled by the detected centers
	float cell_size = 30;
	vector<Point3f> obj_points;

	R_gripper2base.reserve(num_images);
	t_gripper2base.reserve(num_images);
	R_target2cam.reserve(num_images);
	t_target2cam.reserve(num_images);

	for (int i = 0; i < patternsize.height; ++i)
		for (int j = 0; j < patternsize.width; ++j)
			obj_points.push_back(Point3f(float(j*cell_size),
				float(i*cell_size), 0.f));

	for (size_t i = 0; i < num_images; i++)
		images.push_back(imread(fn[i]));

	Mat frame;

	for (size_t i = 0; i < num_images; i++)
	{
		frame = imread(fn[i]); //source image
		bool patternfound = findChessboardCorners(frame, patternsize, centers);
		if (patternfound)
		{
			drawChessboardCorners(frame, patternsize, Mat(centers), patternfound);
			//imshow("window", frame);
			//int key = cv::waitKey(0) & 0xff;
			solvePnP(Mat(obj_points), Mat(centers), cameraMatrix, distortionCoefficients, rvec, tvec);

			Mat R;
			Rodrigues(rvec, R); // R is 3x3
			R_target2cam.push_back(R);
			t_target2cam.push_back(tvec);
			Mat T = Mat::eye(4, 4, R.type()); // T is 4x4
			T(Range(0, 3), Range(0, 3)) = R * 1; // copies R into T
			T(Range(0, 3), Range(3, 4)) = tvec * 1; // copies tvec into T

			cout << "T = " << endl << " " << T << endl << endl;

		}
		cout << patternfound << endl;
	}

	Vec3f theta_01{ deg2rad(-129.232), deg2rad(37.761),   deg2rad(-26.565) };
	Vec3f theta_02{ deg2rad(-121.052), deg2rad(29.316),  deg2rad(-35.449) };
	Vec3f theta_03{ deg2rad(-125.306), deg2rad(45.754), deg2rad(-12.951) };
	Vec3f theta_04{ deg2rad(-116.914), deg2rad(36.956), deg2rad(-16.946) };
	Vec3f theta_05{ deg2rad(-139.678), deg2rad(28.644),deg2rad(-27.222) };
	Vec3f theta_06{ deg2rad(-132.853),  deg2rad(27.269), deg2rad(-19.032) };
	Vec3f theta_07{ deg2rad(-135.202),  deg2rad(30.168), deg2rad(-35.972) };
	Vec3f theta_08{ deg2rad(-135.474), deg2rad(25.538), deg2rad(-25.235) };
	Vec3f theta_09{ deg2rad(-121.553), deg2rad(29.574), deg2rad(-18.772) };
	Vec3f theta_10{ deg2rad(-134.766), deg2rad(32.062),deg2rad(-18.357) };

	Mat robot_rot_01 = eulerAnglesToRotationMatrix(theta_01);
	Mat robot_rot_02 = eulerAnglesToRotationMatrix(theta_02);
	Mat robot_rot_03 = eulerAnglesToRotationMatrix(theta_03);
	Mat robot_rot_04 = eulerAnglesToRotationMatrix(theta_04);
	Mat robot_rot_05 = eulerAnglesToRotationMatrix(theta_05);
	Mat robot_rot_06 = eulerAnglesToRotationMatrix(theta_06);
	Mat robot_rot_07 = eulerAnglesToRotationMatrix(theta_07);
	Mat robot_rot_08 = eulerAnglesToRotationMatrix(theta_08);
	Mat robot_rot_09 = eulerAnglesToRotationMatrix(theta_09);
	Mat robot_rot_10 = eulerAnglesToRotationMatrix(theta_10);

	const Mat robot_tr_01 = (Mat_<float>(3, 1) << 370.267, 139.972, 169.509);
	const Mat robot_tr_02 = (Mat_<float>(3, 1) << 317.717, 120.002, 151.249);
	const Mat robot_tr_03 = (Mat_<float>(3, 1) << 426.381, 120.002, 192.19);
	const Mat robot_tr_04 = (Mat_<float>(3, 1) << 365.542, 68.09, 144.67);
	const Mat robot_tr_05 = (Mat_<float>(3, 1) << 352.186, 208.414, 144.67);
	const Mat robot_tr_06 = (Mat_<float>(3, 1) << 346.761, 205.175, 208.39);
	const Mat robot_tr_07 = (Mat_<float>(3, 1) << 361.521, 199.966, 132.271);
	const Mat robot_tr_08 = (Mat_<float>(3, 1) << 361.521, 168.476, 259.512);
	const Mat robot_tr_09 = (Mat_<float>(3, 1) << 321.583, 123.316, 140.022);
	const Mat robot_tr_10 = (Mat_<float>(3, 1) << 364.491, 166.786, 140.022);

	R_gripper2base.push_back(robot_rot_01);
	R_gripper2base.push_back(robot_rot_02);
	R_gripper2base.push_back(robot_rot_03);
	R_gripper2base.push_back(robot_rot_04);
	R_gripper2base.push_back(robot_rot_05);
	R_gripper2base.push_back(robot_rot_06);
	R_gripper2base.push_back(robot_rot_07);
	R_gripper2base.push_back(robot_rot_08);
	R_gripper2base.push_back(robot_rot_09);
	R_gripper2base.push_back(robot_rot_10);

	t_gripper2base.push_back(robot_tr_01);
	t_gripper2base.push_back(robot_tr_02);
	t_gripper2base.push_back(robot_tr_03);
	t_gripper2base.push_back(robot_tr_04);
	t_gripper2base.push_back(robot_tr_05);
	t_gripper2base.push_back(robot_tr_06);
	t_gripper2base.push_back(robot_tr_07);
	t_gripper2base.push_back(robot_tr_08);
	t_gripper2base.push_back(robot_tr_09);
	t_gripper2base.push_back(robot_tr_10);

	calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper, CALIB_HAND_EYE_TSAI);

	Vec3f R_cam2gripper_r = rotationMatrixToEulerAngles(R_cam2gripper);

	cout << "R_cam2gripper = " << endl << " " << R_cam2gripper.inv() << endl << endl;
	cout << "R_cam2gripper_r = " << endl << " " << R_cam2gripper_r << endl << endl;
	cout << "t_cam2gripper = " << endl << " " << t_cam2gripper << endl << endl;
}

Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
	// Calculate rotation about x axis
	Mat R_x = (Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cos(theta[0]), -sin(theta[0]),
		0, sin(theta[0]), cos(theta[0])
		);

	// Calculate rotation about y axis
	Mat R_y = (Mat_<double>(3, 3) <<
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);

	// Calculate rotation about z axis
	Mat R_z = (Mat_<double>(3, 3) <<
		cos(theta[2]), -sin(theta[2]), 0,
		sin(theta[2]), cos(theta[2]), 0,
		0, 0, 1);


	// Combined rotation matrix
	Mat R = R_x * R_y * R_z;

	return R;

}

float rad2deg(float radian) {
	double pi = 3.14159;
	return(radian * (180 / pi));
}

float deg2rad(float degree) {
	double pi = 3.14159;
	return(degree * (pi / 180));
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
	Mat Rt;
	transpose(R, Rt);
	Mat shouldBeIdentity = Rt * R;
	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

	return  norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{

	assert(isRotationMatrix(R));

	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return Vec3f(x, y, z);

}