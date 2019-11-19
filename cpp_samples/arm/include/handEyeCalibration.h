/*
Author : meng hui
Date : 20191113
Describe : Camera calibration
Company : Coman Robot
*/

#pragma once
#include "calibration.h"


class HandEyeCalibration :
	public Calibration
{
public:

	enum HandEyeCaliMethod {
		HAND_EYE_NAVY = 0,
		HAND_EYE_TSAI = 1,
	};


	HandEyeCalibration(const std::string& imgsDirector,
		const std::string& outputFilename = "camera_data.yml",
		cv::Size boardSize = cv::Size(11, 8),
		double squareSize = 10.,
		Pattern type= CHESSBOARD);  // mm or m

	~HandEyeCalibration();

public:
	// load camera extrinsic and disort coefficients matrix 
	// form file, eg. out_camera_data.yml
	static bool readCameraParameters(const std::string& outputFilename,
		cv::Mat& camMatrix, cv::Mat& distCoefs = cv::Mat());

	// load extrinsic and attitude datas
	[[deprecated("use another override method instead of this one. \
		( Files data format must be specified.")]]
	static bool readDatasFromFile(const std::string& file,
		std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc);

	// load extrinsic datas from yml, eg. out_camera_data.yml, load camera extrinsic datas 
	// one-to-one correspondence 
	static bool readDatasFromFile(
		const std::string& file,
		const std::string& folder,
		std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc, bool useQuaternion = true);

	// calculate A，B
	static void convertVectors2Hij(
		std::vector<cv::Mat> & vecHg, std::vector<cv::Mat> & vecHc,
		std::vector<cv::Mat> & vecHgij, std::vector<cv::Mat> & vecHcij);

	// solve  Ax = xB
	static void computerHandEyeMatrix(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij, HandEyeCaliMethod method,
		const std::string& file);

	static void Tsai_HandEye(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij); // private
	static void Navy_HandEye(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij); // private

																			  // attitude vector => matrix，  10 elems {x,y,z,}
	static cv::Mat attitudeVectorToMatrix(cv::Mat& m,
		bool useQuaternion = true, const std::string& seq = "");

	// check rotatation matrix 3x3 or 4*4{r,t}
	static bool isRotationMatrix(const cv::Mat& R);

	// eulerAngle => rotated matrix
	static cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq = "xyz");

	// quaternion vector to matrix
	static cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q);

	// solve location
	static cv::Mat solveLocation(cv::Vec3d& pos, cv::Mat& Hcg, cv::Mat& Hg,
		cv::Mat& extParamMatrix, cv::Mat& camMatrix) = delete;

	/** @brief 根据图像坐标(校正)求解在基坐标系下的位置
	*
	*	@note  数据类型double；返回值（参考矩阵，若使用）的单位与姿态矩阵单位一致
	*
	*	@param 	imgPt				图像坐标 (u,v)
	*	@param 	z					高度
	*	@param 	Hcg					手眼标定变换矩阵 4*4
	*	@param 	Hg					末端姿态矩阵 4*4
	*	@param 	camMatrix			相机内参矩阵 3*3
	*	@param 	refHg				参考姿态矩阵（保留）
	*	@param 	refExtMatrix		参考外参矩阵（保留）
	*
	*	@return 空间三维点坐标
	*/
	static cv::Point3d getWorldPos(cv::Point2d& imgPt, double z, cv::Mat& Hcg, cv::Mat& Hg,
		cv::Mat& camMatrix);

	static std::vector<cv::Point3d> getWorldPos(
		std::vector<cv::Point2d>& imgPts, double z,
		cv::Mat& Hcg, cv::Mat& Hg, cv::Mat& camMatrix) = delete;

	//计算eye-in-hand中相机与末端手臂的Rt
	static void calibrateEyeInHand(
		cv::Mat& Hcg, std::vector<cv::Mat> & vecHg, std::vector<cv::Mat> & vecHc,
		HandEyeCaliMethod method = HAND_EYE_NAVY, const std::string& file = "EyeInHandMatrix.yml");

};