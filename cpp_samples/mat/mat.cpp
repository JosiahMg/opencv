#include <iostream>
#include "opencv2/opencv.hpp"

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif


int main()
{
	//ֱ��һ��ֵ
	cv::Mat samp1 = (cv::Mat_<double>(1, 3) << 1.0, 2.0, 3.0);
	
	//�Գ�Ա���θ�ֵ
	cv::Mat samp2(2, 2, CV_64F);
	samp2.at<double>(0, 0) = -0.989;
	samp2.at<double>(0, 1) = -2;
	samp2.at<double>(1, 0) = 1;
	samp2.at<double>(1, 1) = 2.5;

	//ʹ��zeros eye����Mat
	cv::Mat samp3 = cv::Mat::zeros(2, 3, CV_64F);
	cv::Mat samp4 = cv::Mat::eye(3, 3, CV_16UC1);

	//ʹ��copyTo���и�ֵ
	cv::Mat samp5 = cv::Mat::eye(3, 3, CV_64F);
	//������samp5(0,0)��ʼλ�ã�����λ��Ϊ��3����1
	samp1.copyTo(samp5({ 0,0,3,1 }));
	//������samp5��1��0��λ�ã�����λ�ÿ�ʼλ�����ӿ�2����2
	samp2.copyTo(samp5({ 0,1,2,2 }));

	//����һ��n*1������
	cv::Mat samp6 = cv::Mat{ 445., 34., 1. };

	cv::Mat samp7(1280, 720, CV_32FC1);
	samp7.setTo(0);

}