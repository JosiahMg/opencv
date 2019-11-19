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
	fs << "array_data" << "["; //���鿪ʼ
	for (int i = 0; i < 10; i++)
	{
		fs << arr[i];
	}
	fs << "]"; //�������
	
			   //д��ṹ��
	fs << "struct_data" << "{"; //�ṹ�忪ʼ
	fs << "x" << t.x;
	fs << "y" << t.y;
	fs << "s" << t.s;
	fs << "}";  //�ṹ����
	return 0;

}

