#include<iostream>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/principal_curvatures.h>
#include <boost/thread/thread.hpp>
#include <math.h>


using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr holder_extraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr holder(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *cloud_HSV);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (int i = 0; i < cloud->points.size(); i++)                         //holder
	{
		if (cloud_HSV->points[i].h < 250 && cloud_HSV->points[i].h > 180 && cloud_HSV->points[i].v > 0.1)            // increase the first parameter to have better visualisation of holder
		{
			inliers->indices.push_back(i);
		}
	}
	pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);
	eifilter.setInputCloud(cloud);
	eifilter.setIndices(inliers);
	eifilter.filter(*holder);

	return holder;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr body_extraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr body(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *cloud_HSV);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (int i = 0; i < cloud->points.size(); i++)                         //body
	{
		if ((cloud_HSV->points[i].h < 80 && cloud_HSV->points[i].h > 0 && cloud_HSV->points[i].v > 0.1) || (cloud_HSV->points[i].h > 350 && cloud_HSV->points[i].v > 0.1))
		{
			inliers->indices.push_back(i);
			//std::cerr << cloud_HSV->points[i].h << std::endl;
		}
	}
	pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);
	eifilter.setInputCloud(cloud);
	eifilter.setIndices(inliers);
	eifilter.filter(*body);

	return body;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr holder_body_extraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_str(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *cloud_HSV);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (int i = 0; i < cloud->points.size(); i++)                        //body with ackene
	{
		if ((cloud_HSV->points[i].h < 250 && cloud_HSV->points[i].h > 180 && cloud_HSV->points[i].v > 0.1) || ((cloud_HSV->points[i].h < 80 && cloud_HSV->points[i].v > 0.1) || (cloud_HSV->points[i].h > 350 && cloud_HSV->points[i].v > 0.1)))
		{
			inliers->indices.push_back(i);
		}
	}
	pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);
	eifilter.setInputCloud(cloud);
	eifilter.setIndices(inliers);
	eifilter.filter(*cloud_str);

	return cloud_str;
}

int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//* load the pcl file

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("1.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
		return(-1);
	}

	// segment holder
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr holder = holder_extraction(cloud);
	// segment body
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr body = body_extraction(cloud);
	// segement holder and body
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr holder_body = holder_body_extraction(cloud);

	//// initialize point cloud visualization
	//boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	////设置背景颜色为白色
	//viewer->setBackgroundColor(255, 255, 255); 
	//// 对点云着色可视化 (red).
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>target_color(cloud, 255, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZRGB>(holder_body);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	//// 等待直到可视化窗口关闭
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	//}


	//// point cloud visualization
	//pcl::visualization::CloudViewer viewer("cloud viewer");
	//viewer.showCloud(holder1);
	//while (!viewer.wasStopped()) 
	//{

	//}

	//Save PCD and PLY files
	pcl::io::savePCDFileASCII("1_H.pcd", *holder);
	pcl::io::savePLYFile("1_H.ply", *holder);
	std::cerr << "Saved " << cloud->points.size() << " data points to .pcd and .ply." << std::endl;

		
	system("pause");
	return 0;
}