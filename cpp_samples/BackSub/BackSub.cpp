#include <opencv2/opencv.hpp>
#include <iostream>

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")

#endif


using namespace cv;
using namespace std;

int main(int argc, char**) 
{
	VideoCapture capture;
	capture.open(2);
	if (!capture.isOpened()) 
	{
		printf("could not find the video file...\n");
		return -1;
	}
	// create windows
	Mat frame;
	Mat bsmaskMOG2, bsmaskKNN;
	namedWindow("input video", CV_WINDOW_AUTOSIZE);
	namedWindow("MOG2", CV_WINDOW_AUTOSIZE);
	namedWindow("KNN Model", CV_WINDOW_AUTOSIZE);

	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));

	// intialization BS
	Ptr<BackgroundSubtractor> pMOG2 = createBackgroundSubtractorMOG2();
	Ptr<BackgroundSubtractor> pKNN = createBackgroundSubtractorKNN();
	while (capture.read(frame)) 
	{
		imshow("input video", frame);

		// MOG BS
		pMOG2->apply(frame, bsmaskMOG2);
		morphologyEx(bsmaskMOG2, bsmaskMOG2, MORPH_OPEN, kernel, Point(-1, -1));
		imshow("MOG2", bsmaskMOG2);

		// KNN BS mask
		pKNN->apply(frame, bsmaskKNN);
		imshow("KNN Model", bsmaskKNN);
		char c = waitKey(100);
		if (c == 27) 
		{
			break;
		}
	}

	capture.release();
	waitKey(0);
	return 0;
}