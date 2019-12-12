#pragma once

#include <librealsense2/rs.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkAutoInit.h>
#include "poseCommon.h"


VTK_MODULE_INIT(vtkRenderingOpenGL);


class RangePointcloud {
public:
	RangePointcloud(float MinPassZ_, float MaxPassZ_);
	RangePointcloud(const std::string& fileName);
	RangePointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);


	~RangePointcloud() {};

	//�ӵ����ļ��м��ص���
	int loadPointcloud();

	//��������ļ�
	int savePlyPointcloud(const std::string& fileName);

	//��D415�����ͼ��ȡ����
	int capturePointcloud();

	//��realsense�ĵ��Ƹ�ʽת��PCL��ʽ
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsPcl(const rs2::points& points);

	//��ʾ�����ļ�
	int viewPointcloud(const std::string& viewerName);

	//��ȡ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloud();




private:
	//��Ŵ������ץȡ�ĵ�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	//���ļ�����ʱ�����ص��ļ���
	std::string fileName;

	//ֱͨ�˲�����Z���˲���Χ
	float MinPassZ;
	float MaxPassZ;
};
