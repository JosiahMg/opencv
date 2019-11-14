/*
 * Author: Yu-Shiang Yan
 *
 * Note: Image processing interface
 */
 
#ifndef IMAGE_PROCESSING_H_INCLUDED
#define IMAGE_PROCESSING_H_INCLUDED

#include "ImageAlgo_HandEye_Calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/* image */
typedef struct struct_CR_Image
{
    unsigned char	    *raw_data;
    unsigned char           *mask_data;
    int             	    width;
    int             	    height;
    int			    pitch;
    int                     roi_shape;

} CR_Image;

/* eye in hand calibration parameters */
class CR_EyeInHand_Cali_Param {
public:
    Mat oMiMatrix;												//相机的投影矩阵M = 内参矩阵*外参矩阵.inv()
    Mat cMoRmatrix;												//第一张图片的旋转矩阵R
    Mat eMcRmatrix;												//hand到相机的旋转矩阵
    Mat wMeShotRmatrix;											//3*3的矩阵，机械臂第一个pose末端角度(rx, ry, rz)计算后的旋转矩阵
    Mat wMeTouchRmatrix;										//机械臂坐标 R(-180, 0, 110)
    Mat tvecsCamObject;											//第一张图片的平移向量t
    Mat tvecsEndEffectorCam;									//hand到相机的平移向量
    Mat tvecsShotWorldEndEffector;								//3*1的矩阵，机械臂第一个POSE的末端的坐标(x, y, z)
    Mat tvecsTouchWorldEndEffector;								//机械臂坐标 T(0, 0, 110)
    Mat tvecsEndEffectorTool;
    Size imgResolution;											//棋盘图片的width*height
    std::vector <CRHandInfo> calibrationHandInfo;				//存放机械臂(x y z rx ry rz)坐标信息
    std::vector<std::vector<Point2f>> calibrationImgFeature;	//存放棋盘交点像素坐标，二维数组 13*9 个交点
    CRHandInfo shotInitHandInfo;
    CRHandInfo shotRunTimeHandInfo;								//机械臂第一个姿态的(x, y, z, rx, ry, rz)
    CRHandInfo touchHandInfo;									//(0, 0, 110, -180, 0, 0)
    int    gridSize;											//棋盘格的大小(mm)
    double toolWidth;
    double toolHeight;
    double toolTheta;
};

class CR_EyeInHand_Result {
    public:
    std::vector<Point2f> imgPList;	//第一张图片的交点像素坐标值
    std::vector<Point3f> worldPList;
    std::vector<CRAngleInfo> worldAngleList;
};

/**
 * @brief   read image
 * @param	image       [in]    image structure(raw data, width, etc.)
 * @param	img_str     [in]    image file name
 * @return	error code  [out]
 */
int CR_ReadImg(CR_Image*   srcImg,
               char*       str);

/**
 * @brief   save as image
 * @param	image       [in]    image structure(raw data, width, etc.)
 * @param	img_str     [in]    image file name
 * @return	error code  [out]
 */			
int CR_SaveAsImg(CR_Image*  image,
                 char*      img_str);

					  
/**
 * @brief   initialize image resoure(image's memory)
 * @param	image       [in]    image structure(raw data, width, etc.)
 * @return	error code  [out]
 */
int CR_Image_Resource_Init(CR_Image** image);

/**
 * @brief   release image resoure
 * @param	image   [in]    image structure(raw data, width, etc.)
 */
void CR_Image_Resource_Release(CR_Image* image);



void CRHandEyeResultClear(CR_EyeInHand_Result *result);

int CRHandEyeResultInit(CR_EyeInHand_Result **result);

void CRHandEyeResultRelease(CR_EyeInHand_Result *result);

int CRHandEyeCaliParamInit(CR_EyeInHand_Cali_Param **caliParam);

void CRHandEyeCaliParamRelease(CR_EyeInHand_Cali_Param *caliParam);

int CRHandEyeGetCalibrationInfo(unsigned char* rawData,
                                int            width,
                                int            height,
                                double         posX,
                                double         posY,
                                double         posZ,
                                double         posRx,
                                double         posRy,
                                double         posRz,
                                std::string         imgInfoStr,
                                CR_EyeInHand_Cali_Param*  pCaliInfo,
                                bool           *found);

int CRHandEyeGetRunTimeInfo(double     shotX,
                            double     shotY,
                            double     shotZ,
                            double     shotRx,
                            double     shotRy,
                            double     shotRz,
                            double     touchX,
                            double     touchY,
                            double     touchZ,
                            double     touchRx,
                            double     touchRy,
                            double     touchRz,
                            int        gridSize,
                            CR_EyeInHand_Cali_Param*  pCaliInfo);

int CRHandEyeGetToolInfo(double     toolWidth,
                         double     toolHeight,
                         double     toolTheta,
                         CR_EyeInHand_Cali_Param*  pCaliInfo);


int CRHandEyeCalcCalibration(CR_EyeInHand_Cali_Param*  pCaliInfo);


int CRHandEyeCalcImgToWorld(CR_EyeInHand_Cali_Param*        pCaliInfo,
                            std::vector<Point2f>            imgPointList,
                            std::vector<Point3f>*           worldPointList,
                            std::vector<CRAngleInfo>*       worldAngleList);


void CRHandEyeParamsClear(CR_EyeInHand_Cali_Param*        pCaliInfo);


#ifdef __cplusplus
}
#endif

#endif //
