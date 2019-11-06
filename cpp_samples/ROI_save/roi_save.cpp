#include <opencv2/opencv.hpp>
#include <iostream>


#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif


int main(int argc, char** argv) {

	cv::Mat srcImage, grayImage;
	srcImage = cv::imread("../../images/r1_Color.png");

	cv::Rect SrcImgROI = cv::Rect(705, 235, 75, 80);

	cv::Mat ROIImg = srcImage(SrcImgROI);
	cv::namedWindow("srcImg", CV_WINDOW_AUTOSIZE);
	cv::imshow("srcImg", ROIImg);

	cv::namedWindow("roiImg", CV_WINDOW_AUTOSIZE);
	cv::imshow("roiImg", srcImage);

	imwrite("../../images/ROI.png", ROIImg);

	cv::waitKey(0);



}