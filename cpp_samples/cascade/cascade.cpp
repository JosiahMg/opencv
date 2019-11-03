#include "opencv2/opencv.hpp"
#include <iostream>

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif



int main(int argc, char** argv)
{
	// For video to detection, more faster
	std::string lbpcascaFilePath = "../../third-party/opencv343/etc/lbpcascades/lbpcascade_frontalface.xml";
	// For image to detection
	std::string haarcascaFilePath = "../../third-party/opencv343/etc/haarcascades/haarcascade_frontalface_alt.xml";
#if 0
	
	cv::CascadeClassifier face_cascade;
	if (!face_cascade.load(haarcascaFilePath))
	{
		printf("could not load harr data..\n");
		return -1;
	}
	cv::VideoCapture capture(0);
	cv::Mat frame;
	cv::Mat gray;
	cv::namedWindow("output", CV_WINDOW_AUTOSIZE);
	while (capture.read(frame))
	{
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		cv::equalizeHist(gray, gray);
		std::vector<cv::Rect> faces;
		face_cascade.detectMultiScale(gray, faces, 1.1, 2, 0, cv::Size(30, 30));
		printf("faces size is %d\n", faces.size());
		for (size_t t = 0; t < faces.size(); t++)
		{
			cv::rectangle(frame, faces[t], cv::Scalar(0, 0, 255), 2, 8, 0);
		}
		cv::imshow("output", frame);
		char c = cv::waitKey(50);
		if (c == 27)
		{
			break;
		}
	}
	capture.release();
#else

	cv::CascadeClassifier face_cascade;
	if (!face_cascade.load(haarcascaFilePath))
	{
		printf("could not load harr data..\n");
		return -1;
	}
	cv::Mat src;
	cv::Mat gray;
	src = cv::imread("images/test.jpg");
	cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
	cv::equalizeHist(gray, gray);

	std::vector<cv::Rect> faces;
	face_cascade.detectMultiScale(gray, faces, 1.1, 2, 0, cv::Size(30, 30));
	printf("faces size is %d\n", faces.size());
	for (size_t t = 0; t < faces.size(); t++)
	{
		cv::rectangle(src, faces[t], cv::Scalar(0, 0, 255), 2, 8, 0);
	}
	cv::namedWindow("output", CV_WINDOW_AUTOSIZE);
	cv::imshow("output", src);
	
	cv::waitKey(0);
#endif

	return 0;
}