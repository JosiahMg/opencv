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

	//从点云文件中加载点云
	int loadPointcloud();

	//保存点云文件
	int savePlyPointcloud(const std::string& fileName);

	//从D415的深度图获取点云
	int capturePointcloud();

	//将realsense的点云格式转成PCL格式
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsPcl(const rs2::points& points);

	//显示点云文件
	int viewPointcloud(const std::string& viewerName);

	//获取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloud();




private:
	//存放从摄像机抓取的点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	//从文件加载时，加载的文件名
	std::string fileName;

	//直通滤波器的Z轴滤波范围
	float MinPassZ;
	float MaxPassZ;
};
