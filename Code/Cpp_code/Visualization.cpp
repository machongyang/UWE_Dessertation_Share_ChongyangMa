#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

using namespace std;

int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//*�򿪵����ļ�
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("5_H_B.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
		return(-1);
	}

	// ��ʼ�����ƿ��ӻ�����
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("��ʾ����"));
	
	//���ñ�����ɫΪ��ɫ
	viewer->setBackgroundColor(255, 255, 255); 
	
	// �Ե�����ɫ���ӻ� (red).
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>target_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	
	// �ȴ�ֱ�����ӻ����ڹر�
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}


	system("pause");
	return 0;
}
