/*
Author : meng hui
Date : 20191209
Describe : The depth images produces point clouds.
Company : Coman Robot
*/



#include "rangePointcloud.h"

/** @brief ���캯����ͨ�������ļ�������
*
*	@note
*
*	@param	 fileName ָ�������ļ���·�����ļ���
*
*	@return  None
*/
RangePointcloud::RangePointcloud(const std::string& fileName)
{
	this->fileName = fileName;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = tmp;
}

/** @brief ���캯����ͨ��������ץȡ���ͼ��������
*
*	@note
*
*	@param	 MinPassZ_ �� ֱͨ�˲�������Сzֵ
*	@param	 MaxPassZ_ �� ֱͨ�˲��������zֵ
*
*	@return  None
*/

RangePointcloud::RangePointcloud(float MinPassZ_, float MaxPassZ_)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = tmp;
	MinPassZ = MinPassZ_;
	MaxPassZ = MaxPassZ_;
}

/** @brief ���캯����ͨ��pcl::PointCloud��������
*
*	@note
*
*	@param	 cloud_ptr ָ��pcl::PointCloud��������ָ��
*
*	@return  None
*/
RangePointcloud::RangePointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
	if (cloud_ptr == nullptr)
	{
		std::cerr << "Create RangePointcloud error" << std::endl;
	}
	cloud = cloud_ptr;
}



/** @brief ���ص����ļ�
*
*	@note  ͨ��ָ���ļ����������ʱʹ�ø÷���
*
*	@param	 None
*
*	@return  EXIT_FAILURE or EXIT_SUCCESS
*/

int RangePointcloud::loadPointcloud()
{
	char tmpStr[100];
	memset(tmpStr, 0x00, 100);
	strcpy(tmpStr, fileName.c_str());
	char* pext = std::strrchr(tmpStr, '.');
	std::string extply("ply");
	std::string extpcd("pcd");
	if (pext) 
	{
		*pext = '\0';
		pext++;
	}
	std::string ext(pext);
	//�����֧���ļ���ʽ���˳�����
	if (!((ext == extply) || (ext == extpcd))) 
	{
		std::cout << "Input��*.pcd or *.ply��" << std::endl;
		return EXIT_FAILURE;
	}
	if (ext == extply)
	{
		return pcl::io::loadPLYFile(fileName, *cloud);
	}
	else
	{
		return pcl::io::loadPCDFile(fileName, *cloud);
	}
	
}

/** @brief ����������ݵ��ļ�
*
*	@note
*
*	@param	 fileName ָ��������ļ���
*
*	@return  Ptr PCL����ָ��
*/

int RangePointcloud::savePlyPointcloud(const std::string& fileName)
{
	int ret = -1;
	ret = pcl::io::savePLYFile(fileName, *cloud);
	if (ret == 0)
	{
		std::cout << "Saved " << fileName << " successed!" << std::endl;
	}
	else 
	{
		std::cout << "Saved " << fileName << " failed!" << std::endl;
	}
	return ret;
}

/** @brief D415��ȡ�ĵ�������ת����pcl::PointCloud����
*
*	@note
*
*	@param	 points realsense��������
*
*	@return  EXIT_FAILURE or EXIT_SUCCESS
*/

pcl::PointCloud<pcl::PointXYZ>::Ptr RangePointcloud::pointsPcl(const rs2::points& points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud_tmp->width = sp.width();
	cloud_tmp->height = sp.height();
	cloud_tmp->is_dense = false;
	cloud_tmp->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud_tmp->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud_tmp;
}

/** @brief ��������л�ȡ��������,��ʹ��ֱͨ�˲�������ָ����Χ��Z�������
*
*	@note ͨ������ֱͨ�˲�����Zֵ�������ʱʹ�ø÷���
*
*	@param	 None
*
*	@return  EXIT_FAILURE or EXIT_SUCCESS
*/

int RangePointcloud::capturePointcloud() try
{
	rs2::pointcloud rs2_pc;
	rs2::pipeline pipe;
	rs2::config cfg;

	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

	rs2::pipeline_profile selection = pipe.start(cfg);

	for (int i = 0; i < 30; i++) 
	{
		auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
	}

	auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();

	//auto RGB = frames.get_color_frame();

	auto points = rs2_pc.calculate(depth);

	auto cloud_tmp = pointsPcl(points);

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_tmp);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(MinPassZ, MaxPassZ);
	if (cloud == nullptr)
	{
		std::cerr << "In capturePointcloud pointcloud was nullptr " << std::endl;
		return EXIT_FAILURE;
	}
	pass.filter(*cloud);
	cout << "successfully captured. " << endl;
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

/** @brief ���ӻ�����
*
*	@note  
*
*	@param viewerName ָ�����ӻ����ڵ�����
*
*	@return  EXIT_FAILURE or EXIT_SUCCESS
*/

int RangePointcloud::viewPointcloud(const std::string& viewerName)
{
	if (cloud == nullptr)
	{
		std::cout << "In viewPointcloud pointcloud was null!" << std::endl;
		return EXIT_FAILURE;
	}
#if 0
	pcl::visualization::PCLVisualizer viewer("cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 230, 20, 20);
	viewer.addPointCloud(cloud, color_handler, "cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");

	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped())
	{
		// Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}
#endif
	pcl::visualization::CloudViewer viewer(viewerName);
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
	return EXIT_SUCCESS;

}

/** @brief ��ȡ����ָ��
*
*	@note 
*
*	@return ����ָ��
*/

pcl::PointCloud<pcl::PointXYZ>::Ptr RangePointcloud::getPointcloud()
{
	return cloud;
}