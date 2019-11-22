
// ply�����ļ�->pcd�����ļ�����ʾ
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>

#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>

#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���


int user_data;
using namespace std;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //���ñ�����ɫ
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());//�������ƶ���
	pcl::PolygonMesh mesh;
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::loadPolygonFilePLY("../../images/test.ply", mesh);									//PCL����VTK��IO�ӿڣ�����ֱ�Ӷ�ȡstl,ply,obj�ȸ�ʽ����ά��������,����PolygonMesh����
	pcl::io::mesh2vtk(mesh, polydata);												//��PolygonMesh����ת��ΪvtkPolyData����
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);								//��ȡ����
	pcl::io::savePCDFileASCII("test.pcd", *cloud);									//�洢Ϊpcb�ļ�



	char strfilepath[256] = "test.pcd";
	if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud)) {							//��ȡpcb��ʽ�ļ�
		cout << "error input!" << endl;
		return -1;
	}

	cout << cloud->points.size() << endl;											//��������ģ
	pcl::visualization::CloudViewer viewer("Cloud Viewer");							  //�������ӻ�viewer����

	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");

	return 0;
}





