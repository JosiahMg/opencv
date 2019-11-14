#include <iostream>
#include <opencv2/opencv.hpp>

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif


int main(int argc, char** argv)
{
	cv::Mat src, dst;

	src = cv::imread("../../images/template/ROI.png");
	dst = src.clone();

	for (int col = 0; col < dst.cols; col++)
	{
		for (int row = 0; row < dst.rows; row++)
		{
			cv::Vec3b bgr = src.at<cv::Vec3b>(row, col);
			bgr[0] = 255 - bgr[0];
			bgr[1] = 255 - bgr[1];
			bgr[2] = 255 - bgr[2];
			dst.at<cv::Vec3b>(row, col) = bgr;
		}
	}
	cv::imshow("src", src);
	cv::imshow("dst", dst);
	cv::waitKey(0);
	return 0;
}

