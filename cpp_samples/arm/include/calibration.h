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

	//�������ں���
	bool doCalibration();

	//����ʹ洢����ڲκ����
	bool runAndSave(const std::string& outputFilename,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		cv::Size imageSize, cv::Size boardSize, Pattern patternType, float squareSize,
		float aspectRatio, int flags, cv::Mat& cameraMatrix,
		cv::Mat& distCoeffs, bool writeExtrinsics, bool writePoints);

	//����ڲα궨
	bool runCalibration(std::vector<std::vector<cv::Point2f> > imagePoints,
		cv::Size imageSize, cv::Size boardSize, Pattern patternType,
		float squareSize, float aspectRatio,
		int flags, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
		std::vector<float>& reprojErrs,
		double& totalAvgErr);

	//�����������
	void saveCameraParams(const std::string& filename,
		cv::Size imageSize, cv::Size boardSize,
		float squareSize, float aspectRatio, int flags,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const std::vector<float>& reprojErrs,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		double totalAvgErr);

	//��ȡ������Ϊ����ϵ����������
	void Calibration::calcChessboardCorners(cv::Size boardSize, float squareSize,
		std::vector<cv::Point3f>& corners, Pattern patternType);

	//��������궨�������
	static double computeReprojectionErrors(
		const std::vector<std::vector<cv::Point3f> >& objectPoints,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrors);

	//��ȡ��������ķ���
	cv::Mat getCameraMatrix() const;
	cv::Mat getDistCoeffsMatrix() const;
	cv::Mat getExtrinsicsBigMat() const;
	std::vector<int> getFoundCheeseBoardVec() const;


private:
	std::string imgsDirectory;
	std::string outputFilename;
	cv::Size boardSize;
	float squareSize = 30;	//mm

	std::vector<int> foundCheeseBoardVec;	//�����Ƿ��ҵ��ǵ�
	cv::Size imageSize;						//ͼ��ߴ�

	Pattern pattern = CHESSBOARD;
	std::vector<std::vector<cv::Point2f> > imagePoints;		//�ǵ���������ֵ
	bool showCorners = true;			//������ʾ�ǵ����ߺ��ͼƬ

	cv::Mat cameraMatrix, distCoeffs;	//�ڲκͻ������
	cv::Mat extrinsicsBigMat;			//n*6�ľ���ÿ�д��Rt

	/* �궨����ʹ�õĲ��� */
	float aspectRatio = 1.0;			
	int flags = 0;


	bool writeExtrinsics = true;	//�Ƿ�洢��ξ���
	bool writePoints = true;		//�Ƿ�洢���������
};


