// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>


#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#define REALSENSE_VERSION "2290"
#pragma comment(lib, "realsense2_" REALSENSE_VERSION ".lib")
#endif


/*
* 从深度相机或者bag文件读取一帧深度图片，将其转换成点云并显示。
*/
// Struct for managing rotation of pointcloud view
struct state {
	state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
		ml(false), offset_x(0.0f), offset_y(0.0f) {}
	double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


pcl_ptr points_to_pcl(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //设置背景颜色
}

int main(int argc, char * argv[]) try
{
	rs2::config cfg;
	cfg.enable_device_from_file("D:/realsense/bag/20191104_205917.bag");

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start(cfg);

	for (int i = 0; i < 50; i++)
	{
		pipe.wait_for_frames();
	}
	// Wait for the next set of frames from the camera
	auto frames = pipe.wait_for_frames();

	auto depth = frames.get_depth_frame();

	// Generate the pointcloud and texture mappings
	points = pc.calculate(depth);

	auto pcl_points = points_to_pcl(points);

	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pcl_points);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*cloud_filtered);

	std::vector<pcl_ptr> layers;
	layers.push_back(pcl_points);
	layers.push_back(cloud_filtered);
	std::cout << layers.size() << std::endl;

	std::cout << pcl_points->points.size() << std::endl;											//点云里点规模
	pcl::visualization::CloudViewer viewer("Cloud Viewer");							  //创建可视化viewer对象

	viewer.showCloud(pcl_points);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");


	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

