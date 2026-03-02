#include<iostream>
#include<thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/file_io.h>	
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/registration/ia_ransac.h>
using namespace std::chrono_literals;
 int main ()
 {
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/0.pcd", *source_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file source_cloud.pcd \n");
    return (-1);
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/Map.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file target_cloud.pcd \n");
    return (-1);
  }
   
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_source;
  ne_source.setInputCloud (source_cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr source_tree_norm (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne_source.setSearchMethod (source_tree_norm);


  pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
  ne_source.setRadiusSearch (0.03);

  ne_source.compute (*source_normals);
 
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh_source;
  vfh_source.setInputCloud (source_cloud);
  vfh_source.setInputNormals (source_normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr source_tree_fea (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh_source.setSearchMethod (source_tree_fea);

  
  pcl::PointCloud<pcl::VFHSignature308>::Ptr source_features (new pcl::PointCloud<pcl::VFHSignature308> ());
  vfh_source.compute (*source_features);


 std::cout<<source_features->size()<<std::endl;

  
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (target_cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr target_tree_norm (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (target_tree_norm);


  pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.03);

  ne.compute (*target_normals);
 
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (target_cloud);
  vfh.setInputNormals (target_normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr target_tree_fea (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (target_tree_fea);

  
  pcl::PointCloud<pcl::VFHSignature308>::Ptr target_features (new pcl::PointCloud<pcl::VFHSignature308> ());
  vfh.compute (*target_features);

 std::cout<<target_features->size()<<std::endl;

 std::cout<<"Kaleem Nawaz"<<std::endl;

/* pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::VFHSignature308> reg;
reg.setMinSampleDistance (0.05f);
reg.setMaxCorrespondenceDistance (0.1);
reg.setMaximumIterations (1000);

reg.setInputSource (source_cloud);
reg.setInputTarget (target_cloud);
reg.setSourceFeatures (source_features);
reg.setTargetFeatures (target_features);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reg (new pcl::PointCloud<pcl::PointXYZ>);
reg.align (cloud_reg); */

  return (0);

}
