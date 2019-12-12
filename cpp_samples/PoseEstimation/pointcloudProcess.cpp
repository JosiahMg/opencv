/*
Author : meng hui
Date : 20191209
Describe : The depth images produces point clouds.
Company : Coman Robot
*/

#include "pointcloudProcess.h"


PointCloudProcess::PointCloudProcess(float MinPassZ_, float MaxPassZ_) :
	RangePointcloud(MinPassZ_, MaxPassZ_)
{
}

PointCloudProcess::PointCloudProcess(const std::string& fileName) :
	RangePointcloud(fileName)
{
}

PointCloudProcess::PointCloudProcess(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) :
	RangePointcloud(cloud_ptr)
{
}

/** @brief Downsampling a PointCloud using a VoxelGrid filter
*
*	@note
*
*	@param	 cloud_filtered 存放滤波后的点云数据
*
*	@return  0 Success   -1 failed
*/

int PointCloudProcess::filterVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr src_cloud,
									   pcl::PointCloud<pcl::PointXYZ> &cloud_filtered,
									   float Size_x, float Size_y, float Size_z)
{
	// Create the filtering object
	if (src_cloud == nullptr)
	{
		std::cout << "In filterVoxelGrid pointcloud was null!" << std::endl;
		return -1;
	}
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(src_cloud);
	
	sor.setLeafSize(Size_x, Size_y, Size_z);
	sor.filter(cloud_filtered);
	std::cout << "After VoxelGrid filtering: " << cloud_filtered.width * cloud_filtered.height << " data points." << std::endl;
	return 0;
}



/** @brief 分离点云中所有的平面，保留最后的非平面点云
*
*	@note
*
*	@param	 cloud_ptr 输入待分割的将点云
*	@param	 maxInter 设置分割最大迭代次数
*	@param	 distance 设置距离平面多少距离的点被划分成平面内的点
*	@param	 ratio 分离平面后，剩余100*ratio%比例不再分割，主要用于分离含有多个平面的点云
*
*	@return  0 Success   -1 failed
*/

int PointCloudProcess::extractPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr src,
									pcl::PointCloud<pcl::PointXYZ> &cloud_filtered,
									int maxInter, double distance, float ratio, bool writePcs)
{
	if (src == nullptr)
	{
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxInter);
	seg.setDistanceThreshold(distance);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int i = 0, nr_points = (int)src->points.size();
	// While 100*ratio% of the original cloud is still there
	while (src->points.size() > ratio * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(src);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud(src);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud_p);

		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
		if (writePcs)
		{
			pcl::PLYWriter writer;
			std::stringstream ss;
			ss << "seg_" << i << ".ply";
			writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);
			
			i++;
		}

		src.swap(cloud_p);
	}
	cloud_filtered = *src;
	return 0;
}


/** @brief 聚类所有目标到vector中
*
*	@note
*
*	@param	 src 输入待聚类的点云
*
*	@return  0 Success   -1 failed
*/
int PointCloudProcess::extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr src,
					std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
					double Tolerance, int minPoints, int maxPoints, bool writePcs)
{
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(src);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(Tolerance); //设置近邻搜索的搜索半径2cm
	ec.setMinClusterSize(minPoints);    //设置一个聚类需要的最少点数目
	ec.setMaxClusterSize(maxPoints);  //设置一个聚类需要的最大数目
	ec.setSearchMethod(tree);     //设置点云的搜索方法
	ec.setInputCloud(src);
	ec.extract(cluster_indices);        //从点云中提取聚类并保存到cluster_indices中

										
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_cluster->points.push_back(src->points[*pit]);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);

		//保存聚类后的点到磁盘
		if (writePcs)
		{
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
			std::stringstream ss;
			ss << "cluster_" << j << ".ply";
			pcl::PLYWriter writer;
			writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
			j++;
		}
	}
	return 0;
}





/** @brief 点云的处理流程：获取 - 分离平面 - 聚类 -  svd
*
*	@note
*
*	@param	 src 输入待聚类的点云
*
*	@return  0 Success   -1 failed
*/

int PointCloudProcess::cloud_process(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters)
{
	int ret = -1;
	//从深度相机获取点云数据
	ret = capturePointcloud();
	if (ret == 0)
	{
		pcl::PointCloud<pcl::PointXYZ> voxel;
		pcl::PointCloud<pcl::PointXYZ> ex_plane;
		//cloud.viewPointcloud("cloud");
		//返回点云数据
		auto src_cloud = getPointcloud();
		if (src_cloud)
		{
			//降采样
			filterVoxelGrid(src_cloud, voxel);
			//分离平面
			extractPlane((pcl::PointCloud<pcl::PointXYZ>::Ptr) &voxel, ex_plane);
			//分割目标
			extractClusters((pcl::PointCloud<pcl::PointXYZ>::Ptr) &ex_plane, clusters);
		}
	}
	else
	{
		std::cout << "Captured failed！" << std::endl;
	}

	return 0;
}