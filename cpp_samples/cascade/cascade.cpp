#include "opencv2/opencv.hpp"
#include <iostream>

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_core" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_imgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_highgui" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_imgcodecs" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_objdetect" OPENCV_VERSION ".lib")
#endif



int main(int argc, char** argv)
{
	std::string cascaFilePath = "../../third-party/opencv343/etc/haarcascades/haarcascade_frontalface_alt.xml";
	cv::CascadeClassifier face_cascade;
	if (!face_cascade.load(cascaFilePath))
	{
		printf("could not load harr data..\n");
		return -1;
	}

	cv::Mat src, gray_src;
	src = cv::imread("images/test.jpg");
	cv::cvtColor(src, gray_src, cv::COLOR_BGR2GRAY);
	cv::equalizeHist(gray_src, gray_src);

	std::vector<cv::Rect> faces;
	face_cascade.detectMultiScale(gray_src, faces, 1.1, 2, 0, cv::Size(30, 30));
	printf("faces size is %d\n", faces.size());
	for (size_t t = 0; t < faces.size(); t++)
	{
		cv::rectangle(src, faces[t], cv::Scalar(0, 0, 255), 2, 8, 0);
	}
	cv::namedWindow("output", CV_WINDOW_AUTOSIZE);
	cv::imshow("output", src);
	
	cv::waitKey(0);
	return 0;
}