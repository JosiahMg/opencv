#include <opencv2/opencv.hpp>
#include <iostream>


#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif



cv::Mat src;
cv::Mat dst;
int threshold_min = 100;
int threshold_max = 255;
void Threshold(int, void*);

int main(int argc, char** argv)
{
	src = cv::imread("../../images/r1_Color.png");
	if (src.empty()) {
		printf("could not load image...\n");
		return -1;
	}

	cv::namedWindow("threshold", cv::WINDOW_AUTOSIZE);
	const char* trackbar_title = "Match Algo Type:";
	cv::createTrackbar(trackbar_title, "threshold", &threshold_min, threshold_max, Threshold);
	Threshold(0, 0);
	cv::waitKey(0);
	return 0;
}

void Threshold(int, void*)
{
	cv::threshold(src, dst, threshold_min, 255, CV_THRESH_BINARY);
	cv::imshow("threshold", dst);
}
