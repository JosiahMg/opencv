
// ply点云文件->pcd点云文件并显示
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>

#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>

#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。


int user_data;
using namespace std;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //设置背景颜色
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());//创建点云对象
	pcl::PolygonMesh mesh;
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::loadPolygonFilePLY("../../images/test.ply", mesh);									//PCL利用VTK的IO接口，可以直接读取stl,ply,obj等格式的三维点云数据,传给PolygonMesh对象
	pcl::io::mesh2vtk(mesh, polydata);												//将PolygonMesh对象转化为vtkPolyData对象
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);								//获取点云
	pcl::io::savePCDFileASCII("test.pcd", *cloud);									//存储为pcb文件



	char strfilepath[256] = "test.pcd";
	if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud)) {							//读取pcb格式文件
		cout << "error input!" << endl;
		return -1;
	}

	cout << cloud->points.size() << endl;											//点云里点规模
	pcl::visualization::CloudViewer viewer("Cloud Viewer");							  //创建可视化viewer对象

	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");

	return 0;
}





