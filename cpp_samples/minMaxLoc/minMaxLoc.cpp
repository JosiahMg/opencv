#include "opencv2/opencv.hpp"
#include <iostream>

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif


int main(int argc, char** argv)
{
	cv::Mat image, image_3c;
	//w:320  h:240
	image.create(cv::Size(320, 240), CV_8UC1);
	image_3c.create(cv::Size(320, 240), CV_8UC3);   //3ͨ����ͼ��
	image.setTo(0);
	image_3c.setTo(0);

	//w:200 h:10
	image.at<uchar>(10, 200) = 255;
	//w:100 h:20
	cv::Point point(20, 100);
	image.at<uchar>(point) = 250;

	/*�������ͼ���Ƕ�ͨ���ģ�����˵imageͼ���ͨ����ʱn��
	  ��ʹ��Mat::at(x, y)ʱ����x= 0 �� height��
	  ��y��ȡֵ��Χ����0��image��width����n

	  Mat::at(point)��ֵʱ��������һ�����֣�����һ��nά����
	 */

	
	image_3c.at<uchar>(10, 300) = 255;  //ʵ��������(int(300/3), 10)[300%3]
	image_3c.at<uchar>(10, 302) = 254;	//ʵ��������(int(302/3), 10)[302%3]
	cv::Point point_3c(20, 200);		//ʵ��������(int(20/3), 200)[20%3]
	image_3c.at<uchar>(point_3c) = 250;

	double maxVal = 0; //���ֵһ��Ҫ����ֵ����������ʱ�ᱨ��
	cv::Point maxLoc;
	minMaxLoc(image, NULL, &maxVal, NULL, &maxLoc);
	std::cout << "��ͨ��ͼ�����ֵ�� " << maxVal << std::endl;
	std::cout << "��ͨ��ͼ�����ֵ����w�� " << maxLoc.x << std::endl;
	std::cout << "��ͨ��ͼ�����ֵ����h�� " << maxLoc.y << std::endl;

	double min_3c, max_3c;
	//ע���ͨ����ʹ��minMaxLoc()�����ǲ��ܸ����������Сֵ����ģ���Ϊÿ�����ص���ʵ�ж�����꣬�����ǲ��������
	minMaxLoc(image_3c, &min_3c, &max_3c, NULL, NULL);
	std::cout << "3ͨ��ͼ�����ֵ: " << max_3c << std::endl;


	cv::namedWindow("2D", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("3D", cv::WINDOW_AUTOSIZE);

	cv::imshow("2D", image);
	cv::imshow("3D", image_3c);

	cv::waitKey(0);
	return 0;

}
