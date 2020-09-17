#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/io/ascii_io.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_remove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int meank, int std)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	//附近邻近点数
	sor.setMeanK(meank);
	//判断是否有离群点
	sor.setStddevMulThresh(std);
	sor.filter(*cloud_filter);

	return cloud_filter;
}

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	//*加载点云文件
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("5_H_B.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
		return(-1);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr body_filtered = outlier_remove(cloud, 250, 1);

	//// 初始化点云可视化对象
	//boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	////设置背景颜色为白色
	//viewer->setBackgroundColor(255, 255, 255);
	////设置背景颜色为黑色
	////viewer->setBackgroundColor(0, 0, 0);
	//// 对点云着色可视化 (red).
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>target_color(cloud, 255, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZRGB>(body_filtered);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	//// 等待直到可视化窗口关闭
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	//}

	//保存PCD和PLY文件
	pcl::io::savePCDFileASCII("5_H_B.pcd", *cloud);
	pcl::io::savePLYFile("5_H_B.ply", *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to .pcd and .ply." << std::endl;


	return (0);
}