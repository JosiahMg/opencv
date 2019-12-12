/*
Author : meng hui
Date : 20191210
Describe : PCA pose estimation.
Company : Coman Robot
*/

#include "poseEstimation.h"


/** @brief ���캯������ʼ��Ŀ���������
*
*	@note
*
*	@param	 clusters ���Ŀ�걻�ָ��ĵ�������
*
*	@return  None
*/
PoseEstimationPCA::PoseEstimationPCA(std::vector<pcl::PointCloud<PointType>::Ptr>& clusters)
{
	for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator cloud = clusters.begin();
		 cloud != clusters.end(); ++cloud)
	{
		struct PoseInfo info;
		memset(&info, 0x00, sizeof(struct PoseInfo));
		info.cloud = *cloud;
		EveryPoseInfo.push_back(info);
	}
}

/** @brief ����ÿ��Ŀ��������Ƶ���Ϣ
*
*	@note
*
*	@param	 info ����Ŀ�����Ϣ
*
*	@return  0
*/
int PoseEstimationPCA::calcCloudInfo(struct PoseInfo& info)
{
	//��ȡ����
	pcl::PointCloud<PointType>::Ptr cloud = info.cloud;
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	info.cam_centroid = pcaCentroid.head<3>();

	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

	//У��������䴹ֱ
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));




	info.eigenVectorsPCA = eigenVectorsPCA;

	//std::cout << "����ֵva(3x1):\n" << eigenValuesPCA << std::endl;
	//std::cout << "vector(3x3):\n" << eigenVectorsPCA << std::endl;
	//std::cout << "centroid(4x1):\n" << pcaCentroid << std::endl;


	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();

	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	//���ĵ�任�������Ϊԭ�㣬���(0, 0, 0) = R*(x, y, z) + t -> t = (0, 0, 0) - R*(x, y, z)
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());
	tm_inv = tm.inverse();

	info.cam_Rt = tm_inv;

	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));


	//auto euler = bboxQ1.toRotationMatrix().eulerAngles(0, 1, 2);
	auto euler = bboxQ.toRotationMatrix().eulerAngles(0, 1, 2);
	std::cout << "Euler from quaternion in roll, pitch, yaw" << std::endl << euler / 3.14 * 180 << std::endl << std::endl;


	/*
	* ���Ƚ����Ʊ任��������Ϊԭ��PCA���ɷ�����ϵ��
	* Ȼ���ڸ�����ϵ�»�ȡ���Ƶ�������Сֵ��
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);

	pcl::getMinMax3D(*transformedCloud, info.cam_min_p1, info.cam_max_p1);

	return 0;
}

/** @brief �ദ����
*
*	@note
*
*	@param	 None
*
*	@return  0
*/
int PoseEstimationPCA::doPoseEstimation()
{
	//�����������ݳ�Ա
	for (std::vector<struct PoseInfo>::iterator info = EveryPoseInfo.begin();
		 info != EveryPoseInfo.end(); ++info)
	{
		calcCloudInfo(*info);
	}
	return 0;
}


/** @brief ���ӻ�Ŀ��ķ���
*
*	@note
*
*	@param	 None
*
*	@return  void
*/
void PoseEstimationPCA::viewBbox()
{
	pcl::visualization::PCLVisualizer viewer;
	int j = 0;
	//����ÿ����������
	for (std::vector<struct PoseInfo>::iterator info = EveryPoseInfo.begin();
	info != EveryPoseInfo.end(); ++info)
	{
		pcl::PointCloud<PointType>::Ptr cloud = info->cloud;

		const Eigen::Quaternionf bboxQ((info->cam_Rt).block<3, 3>(0, 0));

		//��ȡ���Ƶ�����
		const Eigen::Vector3f bboxC = info->cam_centroid;

		//������Ƴ����
		Eigen::Vector3f whd;
		whd = info->cam_max_p1.getVector3fMap() - info->cam_min_p1.getVector3fMap();


		std::stringstream ss_cloud, ss_bbox;
		ss_cloud << "Cloud_" << j;
		ss_bbox << "bbox_" << j;
		//������Ƶ�����
		//Eigen::Vector3f centerp;
		//centerp = 0.5f*(info->cam_min_p1.getVector3fMap() + info->cam_max_p1.getVector3fMap());
	
		//���õ�����ɫ
		pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(cloud, 255, 0, 0);
		
		viewer.addPointCloud(cloud, tc_handler, ss_cloud.str());
		viewer.addCube(bboxC, bboxQ, whd(0), whd(1), whd(2), ss_bbox.str());
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, ss_bbox.str());
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, ss_bbox.str());


		//���Ƶ�����
		PointType cp;
		cp.x = bboxC(0);
		cp.y = bboxC(1);
		cp.z = bboxC(2);

		float sc1 = (whd(0) + whd(1) + whd(2)) / 3;  //����ƽ���߶ȣ����������������ͷ��С

		PointType pcX;
		pcX.x = sc1 * info->eigenVectorsPCA(0, 0) + cp.x;
		pcX.y = sc1 * info->eigenVectorsPCA(1, 0) + cp.y;
		pcX.z = sc1 * info->eigenVectorsPCA(2, 0) + cp.z;
		PointType pcY;
		pcY.x = sc1 * info->eigenVectorsPCA(0, 1) + cp.x;
		pcY.y = sc1 * info->eigenVectorsPCA(1, 1) + cp.y;
		pcY.z = sc1 * info->eigenVectorsPCA(2, 1) + cp.z;
		PointType pcZ;
		pcZ.x = sc1 * info->eigenVectorsPCA(0, 2) + cp.x;
		pcZ.y = sc1 * info->eigenVectorsPCA(1, 2) + cp.y;
		pcZ.z = sc1 * info->eigenVectorsPCA(2, 2) + cp.z;

		std::stringstream arrow_X;
		std::stringstream arrow_Y;
		std::stringstream arrow_Z;
		arrow_X << "arrow_X" << j;
		arrow_Y << "arrow_Y" << j;
		arrow_Z << "arrow_Z" << j;

		viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, arrow_X.str());
		viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, arrow_Y.str());
		viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, arrow_Z.str());

		j++;

	}

	viewer.addCoordinateSystem();
	viewer.setBackgroundColor(0.0, 0.0, 0.0);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}


