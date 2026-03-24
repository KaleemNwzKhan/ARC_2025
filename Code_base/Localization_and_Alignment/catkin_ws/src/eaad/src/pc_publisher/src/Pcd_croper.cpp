#include <pcl/filters/crop_box.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <time.h>
//#include <pcl/visualization/cloud_viewer.h>
int main (int argc, char** argv)
{
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::visualization::CloudViewer viewer("Cloud Viewer");
if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/SOR_filtered_Map2.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
clock_t tStart = clock();
pcl::CropBox<pcl::PointXYZ> boxFilter;
boxFilter.setMin(Eigen::Vector4f(-72, -87, -2.5, 1.0));
boxFilter.setMax(Eigen::Vector4f(86, 128, 25, 1.0));
boxFilter.setInputCloud(cloud);
boxFilter.filter(*target_cloud);
printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
//viewer.showCloud(target_cloud);
//viewer.spin();
 std::cerr << "Point cloud data: " << target_cloud->size () << " points" << std::endl;
pcl::io::savePCDFileASCII ("src/eaad/src/pc_publisher/src/Cropped_Map_2_Plane.pcd", *target_cloud);
return (0);
}
