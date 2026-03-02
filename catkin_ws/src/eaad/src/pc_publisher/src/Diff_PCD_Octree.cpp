#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <time.h>
#include <pcl/io/pcd_io.h>
int main ()
{
  // Octree resolution - side length of octree voxels
  float resolution = 5;
  std::vector<int> newPointIdxVector;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/SOR_transformed.pcd", *cloudA);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/I2.pcd", *cloudB);

  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();
  octree.switchBuffers ();
  clock_t tStart = clock();
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);
  printf("Time taken: %.2fms\n", ((double)(clock() - tStart)/CLOCKS_PER_SEC)*1000);
  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (std::size_t i = 0; i < newPointIdxVector.size (); ++i){
             cloud.push_back (pcl::PointXYZ ((*cloudB)[newPointIdxVector[i]].x, (*cloudB)[newPointIdxVector[i]].y, (*cloudB)[newPointIdxVector[i]].z));
  }
  pcl::io::savePCDFileASCII ("Result_Diff.pcd", cloud);
}