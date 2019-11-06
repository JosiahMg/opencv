#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>


#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif



using namespace std;
using namespace cv;

Mat src, temp, dst;
int match_method = TM_SQDIFF;
int max_track = 5;
const char* INPUT_T = "input image";
const char* OUTPUT_T = "result image";
const char* match_t = "template match-demo";
void Match_Demo(int, void*);
int main(int argc, char** argv) {
	// 待检测图像
	src = imread("../../images/r12_Color.png");
	
	// 模板图像
	temp = imread("../../images/ROI.png");
	if (src.empty() || temp.empty()) {
		printf("could not load image...\n");
		return -1;
	}

//	cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
//	cv::cvtColor(temp, temp, cv::COLOR_BGR2GRAY);
	namedWindow(INPUT_T, CV_WINDOW_AUTOSIZE);
	namedWindow(OUTPUT_T, CV_WINDOW_NORMAL);
	namedWindow(match_t, CV_WINDOW_AUTOSIZE);
	imshow(INPUT_T, temp);
	const char* trackbar_title = "Match Algo Type:";
	createTrackbar(trackbar_title, OUTPUT_T, &match_method, max_track, Match_Demo);

	Match_Demo(0, 0);
	waitKey(0);
	return 0;
}

void Match_Demo(int, void*) {
	//result保存匹配后的结果,而结果大小是原图-模板图+1的值。
	int width = src.cols - temp.cols + 1;
	int height = src.rows - temp.rows + 1;
	Mat result(width, height, CV_32FC1);

	matchTemplate(src, temp, result, match_method, Mat());
	normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

	Point minLoc;
	Point maxLoc;
	double min, max;
	src.copyTo(dst);
	Point temLoc;
	minMaxLoc(result, &min, &max, &minLoc, &maxLoc, Mat());
	if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED) {
		temLoc = minLoc;
	}
	else {
		temLoc = maxLoc;
	}

	// 绘制矩形
	rectangle(dst, Rect(temLoc.x, temLoc.y, temp.cols, temp.rows), Scalar(0, 0, 255), 2, 8);
	rectangle(result, Rect(temLoc.x, temLoc.y, temp.cols, temp.rows), Scalar(0, 0, 255), 2, 8);

	imshow(OUTPUT_T, result);
	imshow(match_t, dst);
}
