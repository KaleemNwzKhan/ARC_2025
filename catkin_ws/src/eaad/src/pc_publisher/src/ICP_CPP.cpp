
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <iostream>

int main ()
{
/*    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI>);
   pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZI>);
   pcl::PointCloud<pcl::PointXYZI> finalCloud ;
   pcl::PointCloud<pcl::PointXYZI> finalCloud1 ;
   pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_new (new pcl::PointCloud<pcl::PointXYZI>) ;


   if(pcl::io::loadPCDFile ("src/eaad/src/pc_publisher/src/0.pcd", *source_cloud)==-1)
   {
     PCL_ERROR ("Couldn't read first file! \n");
     return (-1);
   }

   if(pcl::io::loadPCDFile ("src/eaad/src/pc_publisher/src/Map_Colored.pcd", *target_cloud)==-1)
   {
     PCL_ERROR ("Couldn't read second input file! \n");
     return (-1);
   }

	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	icp.setInputSource(source_cloud);
	icp.setInputTarget(target_cloud);
	icp.setMaximumIterations (500);
	icp.setTransformationEpsilon (1e-9);
	icp.setMaxCorrespondenceDistance (0.05);
	icp.setEuclideanFitnessEpsilon (1);
	icp.setRANSACOutlierRejectionThreshold (1.5);

	icp.align(finalCloud); */


   pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/0.pcd", *source_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file 0.pcd \n");
    return (-1);
  }
  std::cout << "width: " << source_cloud->width << " height: " << source_cloud->height << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/Map.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file Map_Colored.pcd \n");
    return (-1);
  }
  std::cout << "width: " << target_cloud->width << " height: " << target_cloud->height << std::endl; 


 Eigen::Matrix4f trafo;
  trafo <<9.44715285e-02,  9.95091427e-01,  2.94649349e-02, -0.91253279e+02,
          6.36651760e-02, -3.55756423e-02,  9.97337014e-01,  1.00470499e+02,
          9.93489746e-01,-9.23440619e-02,-6.67135561e-02,3.23023979e+00,
          0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00; 

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_init (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*source_cloud, *cloud1_init, trafo);
  std::cout << "width: " << cloud1_init->width << " height: " << cloud1_init->height << std::endl;

  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud1_init);
  icp.setInputTarget(target_cloud);
  icp.setMaximumIterations (4000000);
  icp.setMaxCorrespondenceDistance (15);

  
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZ>);
  icp.align (*aligned);
  (*aligned) += *(target_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr init (new pcl::PointCloud<pcl::PointXYZ>);
  (*init) = (*cloud1_init) + (*target_cloud);
  pcl::io::savePCDFile ("src/eaad/src/pc_publisher/src/pcl_icp_aligned.pcd", *aligned);
  pcl::io::savePCDFile ("src/eaad/src/pc_publisher/src/pcl_icp_init.pcd", *init);
  std::cout << "Converged: " << (icp.hasConverged() ? "True" : "False") << " Score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << "Transformation matrix:" << std::endl << icp.getFinalTransformation() << std::endl;
   std::cout << "Kaleem:" <<  std::endl;
  return 0;
}
