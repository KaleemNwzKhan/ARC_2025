#include <iostream>
#include <thread>
  
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
  
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
  
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;
 int main ()
 {
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/output.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from Map_Colored.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("src/eaad/src/pc_publisher/src/V_1.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from 0.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()<< " data points from room_scan2.pcd" << std::endl;
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.0001);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.0002);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (5);
  // Setting max number of registration iterations.
  ndt.setMaximumIterations (4000000);

  // Setting point cloud to be aligned.
  ndt.setInputSource (input_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);
 
  // Set initial alignment estimate found using robot odometry.
  //Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  //Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  //Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
  //Eigen::Matrix4f init_guess = {{1.00, -0.00, 0.07, -100.70}, {0.00, 1.00, 0.00, 135.40}, {-0.07, -0.00, 1.00, 4.00}, {0,0,0,1}};

 Eigen::Matrix4f init_guess;
  init_guess<<1, 0.00, 0.00, 1,
          0.00, 1.00, 0.00, 1,
          0.00, 0.00, 1.00, 2,
          0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00; 
  //Eigen::Matrix4f init_guess;
  //init_guess<<0.862, 0.011, -0.507, 0.5,
          //-0.139, 0.967, -0.215, 0.7,
          //0.487, 0.255, 0.835, -1.4,
          //0,0,0,1; 

  //std::cout << "Filtered cloud contains " << init_guess<< " data points from room_scan2.pcd" << std::endl;
 
  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //ndt.align (*output_cloud);
  ndt.align (*output_cloud,init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
  std::cout << "Normal Distributions Transform has converged:" << ndt.getFinalTransformation ();

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  //pcl::visualization::PCLVisualizer::Ptr
  //viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  //target_color (target_cloud, 255, 0, 0);
  //viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  //viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  //output_color (output_cloud, 0, 255, 0);
  //viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  //viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "output cloud");

  // Starting visualizer
  //viewer_final->addCoordinateSystem (1.0, "global");
  //viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  //while (!viewer_final->wasStopped ())
  //{
    //viewer_final->spinOnce (100);
    //std::this_thread::sleep_for(100ms);
  //}
  
  return (0);
}