#include <iostream>
#include <fstream>
#include <pcl/search/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>
#include <time.h>
#include <pcl/PCLPointCloud2.h>
int main (int argc, char** argv)
{
std::ofstream myfile;
myfile.open ("src/eaad/src/pc_publisher/src/Plane_3.txt",std::ios_base::app);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/Infra_Plane.pcd", *cloud);
std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
 
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setInputCloud (cloud);
//pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
ne.setSearchMethod (tree);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
ne.setRadiusSearch (0.03);
ne.compute (*normals);

//pcl::search::KdTree<pcl::PointXYZ>::Ptr source_tree_norm (new pcl::search::KdTree<pcl::PointXYZ> ());
//ne_source.setSearchMethod (source_tree_norm);

pcl::PointCloud<pcl::Boundary> boundaries;
pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
est.setInputCloud (cloud);
est.setInputNormals(normals);
est.setRadiusSearch (0.08);   // 2cm radius
est.setAngleThreshold(static_cast<float> (M_PI) / 2.0f);
est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); 
//est.setSearchMethod (source_tree_norm);
est.compute (boundaries);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>);
for(int i = 0; i < cloud->points.size(); i++) 
{ 
   // std::cout<<boundaries[i].boundary_point<<std::endl;
    //std::cout<<"Line End"<<std::endl;
    if(boundaries[i].boundary_point > 0) 
    { 
	cloud_boundary->emplace_back(cloud->points[i]); //
	std::cout<<cloud_boundary->size();
	std::cout<<"Yes"<<std::endl;
	//std::cout<<boundaries[i].boundary_point<<std::endl;
    } 

}  
//myfile << boundaries.points;
//pcl::PCLPointCloud2 output_boundaries;
//toPCLPointCloud2 (boundaries, output_boundaries);
//myfile << output_boundaries;
//std::cerr << "Point cloud data: " << output_boundaries.width << " points" << std::endl;
//std::cerr << "Point cloud data: " << cloud_boundary->size () << " points" << std::endl;
pcl::io::savePCDFile ("test_pcd.pcd", *cloud_boundary);
//pcl::io::savePCDFileASCII ("test_pcd.pcd", output_boundaries);
return 0;
}
