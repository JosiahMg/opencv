#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define OPENCV_VERSION "343"
#define REALSENSE_VERSION "2290"
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#pragma comment(lib, "realsense2_" REALSENSE_VERSION ".lib")
#endif


#define COLOR_COLS		1280
#define COLOR_ROWS		720
#define DEPTH_COLS		1280
#define DEPTH_ROWS		720

void get_intr_from_sdk()
{
	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	rs2::config cfg;

	cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_COLS, DEPTH_ROWS, RS2_FORMAT_Z16, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, COLOR_COLS, COLOR_ROWS, RS2_FORMAT_BGR8, 30);

	pipe.start(cfg);


	rs2::frameset data = pipe.wait_for_frames();
	rs2::video_frame color = data.get_color_frame();
	rs2::depth_frame depth = data.get_depth_frame();

	rs2_intrinsics intr_color = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
	rs2_intrinsics intr_depth = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
	std::cout << intr_color.fx << "       " << 0 << "       " << intr_color.ppx << std::endl;
	std::cout << 0 << "       " << intr_color.fy << "       " << intr_color.ppy << std::endl;
	std::cout << 0 << "       " << 0 << "       " << 1 << std::endl;

	std::cout << std::endl;
	std::cout << intr_depth.fx << "       " << 0 << "       " << intr_depth.ppx << std::endl;
	std::cout << 0 << "       " << intr_depth.fy << "       " << intr_depth.ppy << std::endl;
	std::cout << 0 << "       " << 0 << "       " << 1 << std::endl;
	std::cout << "End" << std::endl;
}

/*
	input_file���ļ�·��
	board_size : �궨�彻����*��
*/
void get_intr_from_chess(const std::string &input_file, const cv::Size &board_size, cv::Size &square_size)
{
	std::ofstream fout("caliberation_result.txt");  /* ����궨������ļ� */
	cv::Size image_size;										/* ͼ��ĳߴ� */
	std::vector<cv::String> imgnames;
	std::vector<cv::Point2f> image_points_buf;							/* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	std::vector<std::vector<cv::Point2f>> image_points_seq;				/* �����⵽������ͼƬ�ϵĽǵ� */
	int image_count = 0;												 /* ͼ������ */


	if (input_file[input_file.size() - 1] == '/' || input_file[input_file.size() - 1] == '\\')
	{
		cv::glob(input_file + "*g", imgnames);
	}
	else
	{
		cv::glob(input_file + "/*g", imgnames);
	}
	std::cout << "��ȡͼƬ�Ľ����������ꡭ����������\n";
	for (const auto &it : imgnames)
	{
		auto srcimg = imread(it, 1);
		cv::Mat gray_img;

		if (srcimg.empty())
		{
			std::cout << "imread failed.  fname:" << it << std::endl;
			continue;
		}

		if (image_count == 0)  //�����һ��ͼƬʱ��ȡͼ������Ϣ
		{
			image_size.width = srcimg.cols;
			image_size.height = srcimg.rows;
			std::cout << "image_size.width = " << image_size.width << std::endl;
			std::cout << "image_size.height = " << image_size.height << std::endl;
		}


		/* ��ȡ�ǵ� */
		cv::cvtColor(srcimg, gray_img, CV_RGB2GRAY);
		if (0 == findChessboardCorners(gray_img, board_size, image_points_buf))
		{
			std::cout << "can not find chessboard corners!\n";	//�Ҳ����ǵ�
			exit(1);
		}
		else
		{
			cv::Mat view_gray;
			image_count++;

			cv::cvtColor(srcimg, view_gray, CV_RGB2GRAY);
			/* �����ؾ�ȷ�� */
			find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ��
																				//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
														   /* ��ͼ������ʾ�ǵ�λ�� */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); //������ͼƬ�б�ǽǵ�
			cv::imshow("Camera Calibration", view_gray);//��ʾͼƬ
			cv::waitKey(500);//��ͣ0.5S		
		}

	}
	std::cout << "��������������ȡ��ɣ�\n";


	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* ������ڲ������� */
	std::vector<int> point_counts;										// ÿ��ͼ���нǵ������
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));	 /* �������5������ϵ����k1,k2,p1,p2,k3 */
	std::vector<cv::Mat> tvecsMat;										/* ÿ��ͼ���ƽ�ƾ��� */
	std::vector<cv::Mat> rvecsMat;										/* ÿ��ͼ�����ת���� */
	std::vector<std::vector<cv::Point3f>> object_points;		/* ����궨���Ͻǵ����ά���� */

	for (int i = 0; i<image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}



	std::cout << "����ͼƬ����λ�õ��������ꡭ����������\n";
	/* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (t = 0; t<image_count; t++)
	{
		std::vector<cv::Point3f> tempPointSet;
		for (i = 0; i<board_size.height; i++)
		{
			for (j = 0; j<board_size.width; j++)
			{
				cv::Point3f realPoint;
				/* ����궨�������������ϵ��z=0��ƽ���� */
				realPoint.x = i*square_size.width;
				realPoint.y = j*square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	std::cout << "���������������\n";


	std::cout << "��ʼ�궨������������\n";
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	std::cout << "�궨��ɣ�\n";


	std::cout << "��ʼ���۱궨���������������\n";
	double total_err = 0.0; /* ����ͼ���ƽ�������ܺ� */
	double err = 0.0; /* ÿ��ͼ���ƽ����� */
	std::vector<cv::Point2f> image_points2; /* �������¼���õ���ͶӰ�� */
	std::cout << "\tÿ��ͼ��ı궨��\n";

	for (i = 0; i<image_count; i++)
	{
		std::vector<cv::Point3f> tempPointSet = object_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		std::vector<cv::Point2f> tempImagePoint = image_points_seq[i];
		cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
		cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << std::endl;

	}
	std::cout << "����ƽ����" << total_err / image_count << "����" << std::endl;


	std::cout << "������ɣ�" << std::endl;

	//���涨����  	
	std::cout << "��ʼ���涨����������������" << std::endl;
	cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* ����ÿ��ͼ�����ת���� */
	fout << "����ڲ�������" << std::endl;
	fout << cameraMatrix << std::endl << std::endl;
	fout << "����ϵ����\n";
	fout << distCoeffs << std::endl << std::endl << std::endl;
	for (int i = 0; i<image_count; i++)
	{
		/* ����ת����ת��Ϊ���Ӧ����ת���� */
		Rodrigues(rvecsMat[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת����" << std::endl;
		fout << rotation_matrix << std::endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << std::endl;
		fout << tvecsMat[i] << std::endl << std::endl;
	}
	std::cout << "��ɱ���" << std::endl;
	fout << std::endl;

	std::cout << "��ʾ����������ͼƬ������������" << std::endl;

	cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
	cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);

	for (const auto &it : imgnames)
	{
		initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
		cv::Mat imageSource = imread(it);
		cv::Mat newimage = imageSource.clone();
		//��һ�ֲ���Ҫת������ķ�ʽ
		//undistort(imageSource,newimage,cameraMatrix,distCoeffs);
		remap(imageSource, newimage, mapx, mapy, cv::INTER_LINEAR);
		cv::imshow("Camera Calibration", newimage);//��ʾͼƬ
		cv::imshow("Camera Original", imageSource);//��ʾͼƬ
		cv::waitKey(500);//��ͣ0.5S		
	}
	std::cout << "END" << std::endl;
}

int main(int argc, char * argv[])
{
	const std::string input_file = "30mm11x8/";
	cv::Size board_size = cv::Size(11, 8);								/* �궨����ÿ�С��еĽǵ��� */
	cv::Size square_size = cv::Size(30, 30);					/* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
	get_intr_from_chess(input_file, board_size, square_size);

	return 0;

}

