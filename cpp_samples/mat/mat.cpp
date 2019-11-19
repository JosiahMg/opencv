#include <iostream>
#include "opencv2/opencv.hpp"

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif


int main()
{
	//直接一起赋值
	cv::Mat samp1 = (cv::Mat_<double>(1, 3) << 1.0, 2.0, 3.0);
	
	//对成员依次赋值
	cv::Mat samp2(2, 2, CV_64F);
	samp2.at<double>(0, 0) = -0.989;
	samp2.at<double>(0, 1) = -2;
	samp2.at<double>(1, 0) = 1;
	samp2.at<double>(1, 1) = 2.5;

	//使用zeros eye创建Mat
	cv::Mat samp3 = cv::Mat::zeros(2, 3, CV_64F);
	cv::Mat samp4 = cv::Mat::eye(3, 3, CV_16UC1);

	//使用copyTo进行赋值
	cv::Mat samp5 = cv::Mat::eye(3, 3, CV_64F);
	//拷贝到samp5(0,0)开始位置，结束位置为宽3，高1
	samp1.copyTo(samp5({ 0,0,3,1 }));
	//拷贝到samp5第1行0列位置，结束位置开始位置增加宽2，高2
	samp2.copyTo(samp5({ 0,1,2,2 }));

	//创建一个n*1的向量
	cv::Mat samp6 = cv::Mat{ 445., 34., 1. };

	cv::Mat samp7(1280, 720, CV_32FC1);
	samp7.setTo(0);

}