#pragma once

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include "poseCommon.h"



class PoseEstimationPCA {
public:

	struct PoseInfo 
	{
		//�������ϵ�������������
		pcl::PointCloud<PointType>::Ptr cloud;
		//�������ϵ����������
		Eigen::Vector3f cam_centroid;
		//�������ϵ����������ֵ����Сֵ��
		PointType cam_min_p1;
		PointType cam_max_p1;
		//���������ϵΪ�����꣺���������ϵ����
		Eigen::Matrix3f eigenVectorsPCA;
		//��������ϵ�������Rt
		Eigen::Matrix4f cam_Rt;
	};


	PoseEstimationPCA(std::vector<pcl::PointCloud<PointType>::Ptr>& clusters);
	~PoseEstimationPCA() {};


	int calcCloudInfo(struct PoseInfo& info);
	void viewBbox();
	int doPoseEstimation();

private:
	std::vector<struct PoseInfo> EveryPoseInfo;
};
