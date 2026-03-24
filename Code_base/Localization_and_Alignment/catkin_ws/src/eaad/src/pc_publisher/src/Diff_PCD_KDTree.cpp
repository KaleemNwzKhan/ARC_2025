#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
int main ()
{
  // Octree resolution - side length of octree voxels
  float resolution = 0.5f;
  std::vector<int> newPointIdxVector;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );
  std::vector<int> static_indices;
 pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/I1.pcd", *cloudA);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/I2.pcd", *cloudB);
 clock_t tStart = clock();
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud (cloudA);
 pcl::ExtractIndices<pcl::PointXYZ> extract;
  int K =2;
        std::vector<int> indices_to_retain;

        for (std::size_t point_i = 0; point_i < cloudB->size(); ++ point_i)
        {
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            if ( tree.nearestKSearch ((*cloudB)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
                for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                {
                    if ( pointNKNSquaredDistance[i] <= 0.01 )
                        static_indices.push_back(pointIdxNKNSearch[i]);
                    else 
                        indices_to_retain.push_back(point_i);
                }   
            }
        }

        inliers->indices = indices_to_retain;
        extract.setInputCloud (cloudB);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud);
  printf("Time taken: %.2fms\n", ((double)(clock() - tStart)/CLOCKS_PER_SEC));   
 pcl::io::savePCDFileASCII ("Result_Diff.pcd", *cloud);

}