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
    Mat oMiMatrix;												//�����ͶӰ����M = �ڲξ���*��ξ���.inv()
    Mat cMoRmatrix;												//��һ��ͼƬ����ת����R
    Mat eMcRmatrix;												//hand���������ת����
    Mat wMeShotRmatrix;											//3*3�ľ��󣬻�е�۵�һ��poseĩ�˽Ƕ�(rx, ry, rz)��������ת����
    Mat wMeTouchRmatrix;										//��е������ R(-180, 0, 110)
    Mat tvecsCamObject;											//��һ��ͼƬ��ƽ������t
    Mat tvecsEndEffectorCam;									//hand�������ƽ������
    Mat tvecsShotWorldEndEffector;								//3*1�ľ��󣬻�е�۵�һ��POSE��ĩ�˵�����(x, y, z)
    Mat tvecsTouchWorldEndEffector;								//��е������ T(0, 0, 110)
    Mat tvecsEndEffectorTool;
    Size imgResolution;											//����ͼƬ��width*height
    std::vector <CRHandInfo> calibrationHandInfo;				//��Ż�е��(x y z rx ry rz)������Ϣ
    std::vector<std::vector<Point2f>> calibrationImgFeature;	//������̽����������꣬��ά���� 13*9 ������
    CRHandInfo shotInitHandInfo;
    CRHandInfo shotRunTimeHandInfo;								//��е�۵�һ����̬��(x, y, z, rx, ry, rz)
    CRHandInfo touchHandInfo;									//(0, 0, 110, -180, 0, 0)
    int    gridSize;											//���̸�Ĵ�С(mm)
    double toolWidth;
    double toolHeight;
    double toolTheta;
};

class CR_EyeInHand_Result {
    public:
    std::vector<Point2f> imgPList;	//��һ��ͼƬ�Ľ�����������ֵ
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
