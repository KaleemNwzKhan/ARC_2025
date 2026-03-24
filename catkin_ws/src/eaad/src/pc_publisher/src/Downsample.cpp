#include <iostream>
#include <thread>
  
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
  
#include <pcl/registration/ndt.h>
//#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;
 int main (int argc, char** argv)
 {
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.7f, 0.7f, 0.7f);

  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;
  pcl::io::savePCDFileASCII ("Downsampled_Map.pcd",  *target_cloud);
  return (0);
 }

 ///map 0.6 
 //v 0.6