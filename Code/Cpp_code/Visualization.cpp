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

	//*打开点云文件
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("5_H_B.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
		return(-1);
	}

	// 初始化点云可视化对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	
	//设置背景颜色为白色
	viewer->setBackgroundColor(255, 255, 255); 
	
	// 对点云着色可视化 (red).
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>target_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	
	// 等待直到可视化窗口关闭
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}


	system("pause");
	return 0;
}
