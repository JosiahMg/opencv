#include "RangePointCloud.h"
#include "pointcloudProcess.h"
#include "poseEstimation.h"

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define REALSENSE_VERSION "2290"
#pragma comment(lib, "realsense2_" REALSENSE_VERSION ".lib")
#endif



int main()
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	int ret = -1;

	PointCloudProcess cloud(0.4, 1.0);
	ret = cloud.cloud_process(clusters);
	if (ret == 0)
	{
		cloud.savePlyPointcloud("original.ply");

		PoseEstimationPCA pose(clusters);
		pose.doPoseEstimation();
		pose.viewBbox();
	}
	return ret;
}

