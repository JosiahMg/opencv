/*
Author : meng hui
Date : 20191112
Describe : Camera calibration
Company : Coman Robot
*/

#include "calibration.h"


Calibration::Calibration(const std::string& imgsDirectory,
	const std::string& outputFilename,
	cv::Size boardSize,
	double squareSize,
	Pattern type
	)
{
	this->imgsDirectory = imgsDirectory;
	this->outputFilename = outputFilename;
	this->boardSize = boardSize;
	this->squareSize = squareSize;
	this->pattern = type;
}


double Calibration::computeReprojectionErrors(
	const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	std::vector<float>& perViewErrors)
{
	std::vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		//根据标定后的内参、外参以及三维坐标重新计算出像素坐标
		projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		//计算L2范数
		err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}


void Calibration::calcChessboardCorners(cv::Size boardSize, float squareSize,
	std::vector<cv::Point3f>& corners, Pattern patternType)
{
	corners.resize(0);

	switch (patternType)
	{
	case CHESSBOARD:
	case CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(cv::Point3f(float(j*squareSize),
					float(i*squareSize), 0));
		break;

	case ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(cv::Point3f(float((2 * j + i % 2)*squareSize),
					float(i*squareSize), 0));
		break;

	default:
		CV_Error(cv::Error::StsBadArg, "Unknown pattern type\n");
	}
}



bool Calibration::runCalibration(std::vector<std::vector<cv::Point2f> > imagePoints,
	cv::Size imageSize, cv::Size boardSize, Pattern patternType,
	float squareSize, float aspectRatio,
	int flags, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
	std::vector<float>& reprojErrs,
	double& totalAvgErr)
{
	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	if (flags & cv::CALIB_FIX_ASPECT_RATIO)
	{
		cameraMatrix.at<double>(0, 0) = aspectRatio;
	}

	distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	std::vector<std::vector<cv::Point3f>> objectPoints(1);
	//获取棋盘格坐标系下的棋盘坐标
	calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);
	//将objectPoints所有值进行初始化同一个坐标值
	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, flags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
		///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;

}


void Calibration::saveCameraParams(const std::string& filename,
	cv::Size imageSize, cv::Size boardSize,
	float squareSize, float aspectRatio, int flags,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const std::vector<float>& reprojErrs,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	double totalAvgErr)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened()) {
		std::cout << "Can not create \"" << filename << "\"." << std::endl;	return;
	}

	if (!cv::utils::fs::exists(filename))
		cv::error(cv::Error::StsBadArg, "file create failed.", __FUNCTION__, __FILE__, __LINE__);

	time_t tt;
	time(&tt);
	struct tm *t2 = new struct tm();
	localtime_s(t2, &tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

	if (flags & cv::CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	if (flags != 0)
	{
		sprintf_s(buf, "flags: %s%s%s%s",
			flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
		//cvWriteComment( *fs, buf, 0 );
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

	//// 增加旋转、平移
	fs << "rvecs" << rvecs;
	fs << "tvecs" << tvecs;

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			// {r, t}
			cv::Mat r = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
			cv::Mat t = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();	// 旋转向量 -> Rodrigues变换 -> 旋转矩阵  
			t = tvecs[i].t();	// 旋转矩阵

			//Mat r_matrix;
			//cv::Rodrigues(r, r_matrix);
		}
		//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
		fs << "extrinsic_parameters" << bigmat;

		std::swap(bigmat, extrinsicsBigMat);
	}

	/////  0731 存储检测棋盘结果  1  成功， 0  失败  ==》 对应1的菜读取机械手姿态数据
	cv::Mat matFoundCheeseBoard(1, foundCheeseBoardVec.size(), CV_32S, foundCheeseBoardVec.data());
	fs << "found_cheese_board" << matFoundCheeseBoard;

	//将所有像素坐标拷贝到imagePtMat中，每行为一张图片上的像素坐标
	if (!imagePoints.empty())
	{
		cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			cv::Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
	fs.release();

	std::cout << "Calibration done, save datas in file " << filename << std::endl << std::endl;
}


bool Calibration::runAndSave(const std::string& outputFilename,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	cv::Size imageSize, cv::Size boardSize, Pattern patternType, float squareSize,
	float aspectRatio, int flags, cv::Mat& cameraMatrix,
	cv::Mat& distCoeffs, bool writeExtrinsics, bool writePoints)
{
	//R t
	std::vector<cv::Mat> rvecs, tvecs;
	//存放标定后结果的误差值
	std::vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
		aspectRatio, flags, cameraMatrix, distCoeffs,
		rvecs, tvecs, reprojErrs, totalAvgErr);

	printf("%s. avg reprojection error = %.2f\n",
		ok ? "Calibration succeeded" : "Calibration failed",
		totalAvgErr);

	if (ok)
	{
		saveCameraParams(outputFilename, imageSize,
			boardSize, squareSize, aspectRatio,
			flags, cameraMatrix, distCoeffs,
			writeExtrinsics ? rvecs : std::vector<cv::Mat>(),
			writeExtrinsics ? tvecs : std::vector<cv::Mat>(),
			writeExtrinsics ? reprojErrs : std::vector<float>(),
			writePoints ? imagePoints : std::vector<std::vector<cv::Point2f> >(),
			totalAvgErr);
	}
	return ok;
}


//相机内参标定
bool Calibration::doCalibration()
{
	std::vector<cv::String> imageList;
	std::string path = this->imgsDirectory;

	/*
		assume no exits sequence: 1,2,..,10,11,..,20,21
		cv::glob will get 1,10,11,12,...,19,2,20, 21
	*/
	cv::glob(path + "/*.png", imageList);
	if (imageList.size() == 0)
	{
		std::cout << "no images." << std::endl;
		return false;
	}
	int nframes = (int)imageList.size();
	foundCheeseBoardVec.resize(nframes);

	for (int i = 0; ; i++)
	{
		cv::Mat view, viewGray;

		std::cout << "\n";

		if (i < (int)imageList.size()) 
		{
			std::cout << "  " << imageList[i];
			view = imread(imageList[i], cv::IMREAD_COLOR);
		}
		if (view.empty())
		{
			if (imagePoints.size() > 0)
			{
				runAndSave(outputFilename, imagePoints, imageSize,
					boardSize, pattern, squareSize, aspectRatio,
					flags, cameraMatrix, distCoeffs,
					writeExtrinsics, writePoints);
			}
			break;
		}
		imageSize = view.size();
		std::vector<cv::Point2f> pointbuf;
		cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
		bool found = false;

		switch (pattern)
		{
		case CHESSBOARD:
			found = findChessboardCorners(view, boardSize, pointbuf,
				cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
			if (found)
			{
				//获取亚像素角点
				cornerSubPix(viewGray, pointbuf, cv::Size(11, 11), cv::Size(-1, -1),
					cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
				imagePoints.push_back(pointbuf);
				foundCheeseBoardVec[i] = 1;
			}
			else
			{
				foundCheeseBoardVec[i] = 0;
			}
			break;

		case CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, pointbuf, cv::CALIB_CB_SYMMETRIC_GRID);
			if (found)
			{
				imagePoints.push_back(pointbuf);
				foundCheeseBoardVec[i] = 1;
			}
			else
			{
				foundCheeseBoardVec[i] = 0;
			}
			break;
			//非对称的圆形棋盘
		case ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID);
			if (found)
			{
				imagePoints.push_back(pointbuf);
				foundCheeseBoardVec[i] = 1;
			}
			else
			{
				foundCheeseBoardVec[i] = 0;
			}
			break;
		}


		if (showCorners)
		{
			if (found)
			{
				drawChessboardCorners(view, boardSize, cv::Mat(pointbuf), found);
				cv::namedWindow("Image View", cv::WINDOW_AUTOSIZE);
				imshow("Image View", view);
				cv::waitKey(300);
			}
		}

	}


	if (showCorners)
	{
		cv::destroyAllWindows();
	}
	return true;
}


// Get Method
cv::Mat Calibration::getExtrinsicsBigMat() const
{
	return this->extrinsicsBigMat;
}

cv::Mat Calibration::getCameraMatrix() const
{
	return this->cameraMatrix;
}

cv::Mat Calibration::getDistCoeffsMatrix() const
{
	return this->distCoeffs;
}

std::vector<int> Calibration::getFoundCheeseBoardVec() const
{
	return this->foundCheeseBoardVec;
}

