#pragma once
#include "rangePointcloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "poseCommon.h"



class PointCloudProcess :
	public RangePointcloud {

public:
	PointCloudProcess(float MinPassZ_, float MaxPassZ_);
	PointCloudProcess(const std::string& fileName);
	PointCloudProcess(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
	~PointCloudProcess(){}

	int filterVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr src_cloud,
						pcl::PointCloud<pcl::PointXYZ> &cloud_filtered,
						float Size_x = 0.005, float Size_y = 0.005, float Size_z = 0.005);

	int extractPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr src,
							pcl::PointCloud<pcl::PointXYZ> &cloud_filtered, int maxInter=1000,
							double distance=0.01, float ration=0.4, bool writePcs=false);

	int extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr src,
							   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
							   double Tolerance=0.02,int minPoints=200, 
							   int maxPoints=25000, bool writePcs=true);

	int cloud_process(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);
};