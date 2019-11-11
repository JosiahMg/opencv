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
	input_file：文件路径
	board_size : 标定板交点行*列
*/
void get_intr_from_chess(const std::string &input_file, const cv::Size &board_size, cv::Size &square_size)
{
	std::ofstream fout("caliberation_result.txt");  /* 保存标定结果的文件 */
	cv::Size image_size;										/* 图像的尺寸 */
	std::vector<cv::String> imgnames;
	std::vector<cv::Point2f> image_points_buf;							/* 缓存每幅图像上检测到的角点 */
	std::vector<std::vector<cv::Point2f>> image_points_seq;				/* 保存检测到的所有图片上的角点 */
	int image_count = 0;												 /* 图像数量 */


	if (input_file[input_file.size() - 1] == '/' || input_file[input_file.size() - 1] == '\\')
	{
		cv::glob(input_file + "*g", imgnames);
	}
	else
	{
		cv::glob(input_file + "/*g", imgnames);
	}
	std::cout << "提取图片的交点像素坐标………………\n";
	for (const auto &it : imgnames)
	{
		auto srcimg = imread(it, 1);
		cv::Mat gray_img;

		if (srcimg.empty())
		{
			std::cout << "imread failed.  fname:" << it << std::endl;
			continue;
		}

		if (image_count == 0)  //读入第一张图片时获取图像宽高信息
		{
			image_size.width = srcimg.cols;
			image_size.height = srcimg.rows;
			std::cout << "image_size.width = " << image_size.width << std::endl;
			std::cout << "image_size.height = " << image_size.height << std::endl;
		}


		/* 提取角点 */
		cv::cvtColor(srcimg, gray_img, CV_RGB2GRAY);
		if (0 == findChessboardCorners(gray_img, board_size, image_points_buf))
		{
			std::cout << "can not find chessboard corners!\n";	//找不到角点
			exit(1);
		}
		else
		{
			cv::Mat view_gray;
			image_count++;

			cv::cvtColor(srcimg, view_gray, CV_RGB2GRAY);
			/* 亚像素精确化 */
			find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(5, 5)); //对粗提取的角点进行精确化
																				//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			image_points_seq.push_back(image_points_buf);  //保存亚像素角点
														   /* 在图像上显示角点位置 */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); //用于在图片中标记角点
			cv::imshow("Camera Calibration", view_gray);//显示图片
			cv::waitKey(500);//暂停0.5S		
		}

	}
	std::cout << "交点像素坐标提取完成！\n";


	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
	std::vector<int> point_counts;										// 每幅图像中角点的数量
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));	 /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	std::vector<cv::Mat> tvecsMat;										/* 每幅图像的平移矩阵 */
	std::vector<cv::Mat> rvecsMat;										/* 每幅图像的旋转向量 */
	std::vector<std::vector<cv::Point3f>> object_points;		/* 保存标定板上角点的三维坐标 */

	for (int i = 0; i<image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}



	std::cout << "生成图片所在位置的世界坐标………………\n";
	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t<image_count; t++)
	{
		std::vector<cv::Point3f> tempPointSet;
		for (i = 0; i<board_size.height; i++)
		{
			for (j = 0; j<board_size.width; j++)
			{
				cv::Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i*square_size.width;
				realPoint.y = j*square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	std::cout << "世界坐标生成完成\n";


	std::cout << "开始标定………………\n";
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	std::cout << "标定完成！\n";


	std::cout << "开始评价标定结果………………\n";
	double total_err = 0.0; /* 所有图像的平均误差的总和 */
	double err = 0.0; /* 每幅图像的平均误差 */
	std::vector<cv::Point2f> image_points2; /* 保存重新计算得到的投影点 */
	std::cout << "\t每幅图像的标定误差：\n";

	for (i = 0; i<image_count; i++)
	{
		std::vector<cv::Point3f> tempPointSet = object_points[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
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
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << std::endl;

	}
	std::cout << "总体平均误差：" << total_err / image_count << "像素" << std::endl;


	std::cout << "评价完成！" << std::endl;

	//保存定标结果  	
	std::cout << "开始保存定标结果………………" << std::endl;
	cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	fout << "相机内参数矩阵：" << std::endl;
	fout << cameraMatrix << std::endl << std::endl;
	fout << "畸变系数：\n";
	fout << distCoeffs << std::endl << std::endl << std::endl;
	for (int i = 0; i<image_count; i++)
	{
		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(rvecsMat[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << std::endl;
		fout << rotation_matrix << std::endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << std::endl;
		fout << tvecsMat[i] << std::endl << std::endl;
	}
	std::cout << "完成保存" << std::endl;
	fout << std::endl;

	std::cout << "显示畸变纠正后的图片………………" << std::endl;

	cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
	cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);

	for (const auto &it : imgnames)
	{
		initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
		cv::Mat imageSource = imread(it);
		cv::Mat newimage = imageSource.clone();
		//另一种不需要转换矩阵的方式
		//undistort(imageSource,newimage,cameraMatrix,distCoeffs);
		remap(imageSource, newimage, mapx, mapy, cv::INTER_LINEAR);
		cv::imshow("Camera Calibration", newimage);//显示图片
		cv::imshow("Camera Original", imageSource);//显示图片
		cv::waitKey(500);//暂停0.5S		
	}
	std::cout << "END" << std::endl;
}

int main(int argc, char * argv[])
{
	const std::string input_file = "30mm11x8/";
	cv::Size board_size = cv::Size(11, 8);								/* 标定板上每行、列的角点数 */
	cv::Size square_size = cv::Size(30, 30);					/* 实际测量得到的标定板上每个棋盘格的大小 */
	get_intr_from_chess(input_file, board_size, square_size);

	return 0;

}

