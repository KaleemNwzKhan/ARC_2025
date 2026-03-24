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
int main (int argc, char** argv)
{
std::ofstream myfile;
  myfile.open ("src/eaad/src/pc_publisher/src/Plane_1_45.txt",std::ios_base::app);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/199.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  clock_t tStart = clock();
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  //seg.setAxis(Eigen::Vector3f::UnitZ());   // Set the axis along which we need to search for a model perpendicular to
  //seg.setEpsAngle((10.*M_PI)/180.);           // Set maximum allowed difference between the model normal and the given axis in radians
  //seg.setOptimizeCoefficients(true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100000);                // 1000
  seg.setDistanceThreshold (0.05f);            //0.01
  
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  myfile << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3];
myfile.close();
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::ExtractIndices<pcl::PointXYZ> extract;
extract.setInputCloud (cloud);
extract.setIndices (inliers);
extract.setNegative (false);
extract.filter (*target_cloud);
std::cerr << "PointCloud representing the planar component: " << target_cloud->width * target_cloud->height << " data points." << std::endl;
//writer.write<pcl::PointXYZ> ("Plane_infrastructure.pcd", *target_cloud, false);
pcl::io::savePCDFileASCII ("src/eaad/src/pc_publisher/src/Plane_1_45.pcd", *target_cloud);
  return (0);
}

