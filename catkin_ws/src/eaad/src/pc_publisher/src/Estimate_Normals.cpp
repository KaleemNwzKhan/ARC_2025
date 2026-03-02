#include <iostream>
#include <fstream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <time.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
int main (int argc, char** argv)
{
std::ofstream myfile;
  myfile.open ("src/eaad/src/pc_publisher/src/Plane_1_45.txt",std::ios_base::app);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/Plane_1_45.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  std::cerr << "Point cloud data: " << cloud->points[0] << " points" << std::endl;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_source;
  ne_source.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr source_tree_norm (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne_source.setSearchMethod (source_tree_norm);

  pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
  ne_source.setRadiusSearch (300000);
  ne_source.setViewPoint (0, 0, 1);
  ne_source.compute (*source_normals);
  //float curvature;
  //Eigen::Vector4f plane_parameters;
  //int a=cloud->points[0].x;
  //int b=cloud->points[0].y;
  //int c=cloud->points[0].z;
  //td::vector<int> indices{a, b, c};
  //ne_source.computePointNormal (cloud->points[0],indices,plane_parameters, curvature);
  std::cerr << "Point cloud data: " << source_normals->size () << " points" << std::endl;
  std::cerr << "Point cloud data: " << source_normals->points[0] << " points" << std::endl;
  //myfile << source_normals->points[0];
  myfile.close();
  return (0);
}