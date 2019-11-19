/*
	Author : meng hui
	Date : 20191112
	Describe : Camera calibration
	Company : Coman Robot
*/

#pragma once


#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utils/filesystem.hpp"

#include <stdio.h>
#include <string.h>

#include <time.h>

#include <iostream>
#include <fstream>


class Calibration
{
public:
	enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

	explicit Calibration(
		const std::string& imgsDirectory,
		const std::string& outputFilename,
		cv::Size boardSize,
		double squareSize,
		Pattern type
		);

	//该类的入口函数
	bool doCalibration();

	//保存和存储相机内参和外参
	bool runAndSave(const std::string& outputFilename,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		cv::Size imageSize, cv::Size boardSize, Pattern patternType, float squareSize,
		float aspectRatio, int flags, cv::Mat& cameraMatrix,
		cv::Mat& distCoeffs, bool writeExtrinsics, bool writePoints);

	//相机内参标定
	bool runCalibration(std::vector<std::vector<cv::Point2f> > imagePoints,
		cv::Size imageSize, cv::Size boardSize, Pattern patternType,
		float squareSize, float aspectRatio,
		int flags, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
		std::vector<float>& reprojErrs,
		double& totalAvgErr);

	//保存相机参数
	void saveCameraParams(const std::string& filename,
		cv::Size imageSize, cv::Size boardSize,
		float squareSize, float aspectRatio, int flags,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const std::vector<float>& reprojErrs,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		double totalAvgErr);

	//获取以棋盘为坐标系的世界坐标
	void Calibration::calcChessboardCorners(cv::Size boardSize, float squareSize,
		std::vector<cv::Point3f>& corners, Pattern patternType);

	//计算相机标定后误差率
	static double computeReprojectionErrors(
		const std::vector<std::vector<cv::Point3f> >& objectPoints,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrors);

	//获取相机参数的方法
	cv::Mat getCameraMatrix() const;
	cv::Mat getDistCoeffsMatrix() const;
	cv::Mat getExtrinsicsBigMat() const;
	std::vector<int> getFoundCheeseBoardVec() const;


private:
	std::string imgsDirectory;
	std::string outputFilename;
	cv::Size boardSize;
	float squareSize = 30;	//mm

	std::vector<int> foundCheeseBoardVec;	//表征是否找到角点
	cv::Size imageSize;						//图像尺寸

	Pattern pattern = CHESSBOARD;
	std::vector<std::vector<cv::Point2f> > imagePoints;		//角点像素坐标值
	bool showCorners = true;			//用于显示角点连线后的图片

	cv::Mat cameraMatrix, distCoeffs;	//内参和畸变矩阵
	cv::Mat extrinsicsBigMat;			//n*6的矩阵，每行存放Rt

	/* 标定函数使用的参数 */
	float aspectRatio = 1.0;			
	int flags = 0;


	bool writeExtrinsics = true;	//是否存储外参矩阵
	bool writePoints = true;		//是否存储像素坐标点
};


