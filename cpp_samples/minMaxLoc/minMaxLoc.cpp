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
	image_3c.create(cv::Size(320, 240), CV_8UC3);   //3通道的图像
	image.setTo(0);
	image_3c.setTo(0);

	//w:200 h:10
	image.at<uchar>(10, 200) = 255;
	//w:100 h:20
	cv::Point point(20, 100);
	image.at<uchar>(point) = 250;

	/*如果所画图像是多通道的，比如说image图像的通道数时n，
	  则使用Mat::at(x, y)时，其x= 0 至 height，
	  而y的取值范围则是0到image的width乘以n

	  Mat::at(point)的值时，都不是一个数字，而是一个n维向量
	 */

	
	image_3c.at<uchar>(10, 300) = 255;  //实际坐标是(int(300/3), 10)[300%3]
	image_3c.at<uchar>(10, 302) = 254;	//实际坐标是(int(302/3), 10)[302%3]
	cv::Point point_3c(20, 200);		//实际坐标是(int(20/3), 200)[20%3]
	image_3c.at<uchar>(point_3c) = 250;

	double maxVal = 0; //最大值一定要赋初值，否则运行时会报错
	cv::Point maxLoc;
	minMaxLoc(image, NULL, &maxVal, NULL, &maxLoc);
	std::cout << "单通道图像最大值： " << maxVal << std::endl;
	std::cout << "单通道图像最大值坐标w： " << maxLoc.x << std::endl;
	std::cout << "单通道图像最大值坐标h： " << maxLoc.y << std::endl;

	double min_3c, max_3c;
	//注意多通道在使用minMaxLoc()函数是不能给出其最大最小值坐标的，因为每个像素点其实有多个坐标，所以是不会给出的
	minMaxLoc(image_3c, &min_3c, &max_3c, NULL, NULL);
	std::cout << "3通道图像最大值: " << max_3c << std::endl;


	cv::namedWindow("2D", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("3D", cv::WINDOW_AUTOSIZE);

	cv::imshow("2D", image);
	cv::imshow("3D", image_3c);

	cv::waitKey(0);
	return 0;

}
