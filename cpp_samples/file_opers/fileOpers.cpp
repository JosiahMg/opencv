#include <opencv2/opencv.hpp>
#include <iostream>

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")

#endif

using namespace std;
using namespace cv;


typedef struct
{
	int x;
	int y;
	string s;
}test_t;


int main(int argc, char* argv)
{
	FileStorage fs("test.xml", FileStorage::WRITE);

	string str = "hello sysu!";
	int arr[10] = { 1,2,3,4,5,6,7,8,9,10 };
	test_t t = { 3,4,"hi sysu" };
	map<string, int> m;
	m["kobe"] = 100;
	m["james"] = 99;
	m["curry"] = 98;

	fs << "string_data" << str;
	fs << "array_data" << "["; //数组开始
	for (int i = 0; i < 10; i++)
	{
		fs << arr[i];
	}
	fs << "]"; //数组结束
	
			   //写入结构体
	fs << "struct_data" << "{"; //结构体开始
	fs << "x" << t.x;
	fs << "y" << t.y;
	fs << "s" << t.s;
	fs << "}";  //结构结束
	return 0;

}

