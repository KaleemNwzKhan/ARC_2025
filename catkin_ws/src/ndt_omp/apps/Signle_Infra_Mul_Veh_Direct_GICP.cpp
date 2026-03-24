#include <ros/ros.h>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <queue>
#include <cfloat>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include <iomanip>
#include <cfloat>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h> // C++17 standard header file name
#include <experimental/filesystem>
Eigen::Matrix4f init_guess;
std::queue<Eigen::Matrix4f> q;
std::queue<float> lat;
int count_pcds=0;
float avg_time=0;
std::string map_path = "";
std::string point_cloud_path = "";
int Sanity_Check(ros::NodeHandle nh);
void Initialize_Init_Guess();
void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform);
Eigen::Matrix4f Get_Init_Guess();
std::ofstream vehicle_pose_file;
std::ofstream loc_latency;
pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud) {
  // Set input and output clouds
  //registration->setInputTarget(target_cloud);
  //registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  // Registration
  auto t1 = ros::WallTime::now();
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  registration->align(*aligned, init_guess);
  auto t2 = ros::WallTime::now();
  auto duration = (t2 - t1).toSec();
  // Save transformation
  Eigen::Matrix4f current_frame_pose = registration->getFinalTransformation();
  std::cout << "Initial Guess: "<<init_guess<<std::endl;
  std::cout << "Vehicle Pose: "<<current_frame_pose<<std::endl;
  q.push(current_frame_pose);
  std::cout << "Fitness: " << registration->getFitnessScore() << std::endl;
  lat.push(duration * 1000);
  avg_time=avg_time+(duration * 1000);
  count_pcds=count_pcds+1;
  std::cout << "Latency : " << duration * 1000 << " msec" << std::endl;

  Update_Init_Guess(current_frame_pose);
  return aligned;
}

int main(int argc, char **argv) {
 struct
  {
    bool operator()(std::filesystem::path a, std::filesystem::path b) const
    {
      std::string a1 = a;
      std::string b1 = b;
      int a2 = stoi(a1.substr(a1.find_last_of("/\\") + 1));
      int b2 = stoi(b1.substr(b1.find_last_of("/\\") + 1));
      std::cout << a2 << " " << b2 << std::endl;
      return a2 < b2;
    }
  } customLess;
  ros::init(argc, argv, "Kaleem");
  vehicle_pose_file.open("example.txt");
  loc_latency.open("latency.txt");
  ros::NodeHandle n_h;
  if(Sanity_Check(n_h) == -1) {
    std::cout << "Exitting early" << std::endl;
    return -1;
  }
  std::cout << "Reading map file ..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> (map_path, *map);
   pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_map(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(1.5f, 1.5f, 1.5f);
    voxelgrid.setInputCloud(map);
    voxelgrid.filter(*downsampled_map);

  std::filesystem::path path_to_cloud=point_cloud_path;
  Initialize_Init_Guess();
  Eigen::Matrix4f transformation_matrix = Get_Init_Guess ();
  std::vector<std::filesystem::path> files_in_directory;
  std::copy(std::filesystem::directory_iterator(path_to_cloud), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
  std::sort(files_in_directory.begin(), files_in_directory.end(), customLess);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (const std::string &filename1 : files_in_directory)
  {
   /*  std::string filename2=filename1;
    std::cout << filename2<<std::endl;
    std::cout << "Reading map file ..." << path_to_cloud<<std::endl; */

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename1, *point_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }	

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid1;
    voxelgrid1.setLeafSize(1.5f, 1.5f, 1.5f);
    voxelgrid1.setInputCloud(point_cloud);
    voxelgrid1.filter(*downsampled_cloud);
   // downsampled_cloud=point_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  std::cout << "--- pcl::GICP ---" << std::endl;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
  gicp->setTransformationEpsilon(1e-9);
  gicp->setMaxCorrespondenceDistance(2.5);
  gicp->setMaximumIterations(5);
  aligned = align(gicp, downsampled_map, downsampled_cloud);
  //gicp->setTransformationEpsilon(2e-2);
  //gicp->setMaxCorrespondenceDistance(0.01);
   //gicp->setMaximumIterations(100);

  std::cout << "--- pclomp::GICP ---" << std::endl;
  pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp_omp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
  //aligned = align(gicp_omp, downsampled_map, downsampled_cloud);
  //gicp_omp->setTransformationEpsilon(1e-5);
  //gicp_omp->setEuclideanFitnessEpsilon (1);
  //gicp_omp->setMaxCorrespondenceDistance(0.6);
  //gicp_omp->setMaximumIterations (10);
  //gicp_omp.setRANSACOutlierRejectionThreshold (1.5);
  //gicp_omp->setMaximumIterations(100);
  //aligned = align(gicp_omp, downsampled_map, downsampled_cloud);

   while (not q.empty ())
    {
      vehicle_pose_file<<q.front () <<std::endl;
      loc_latency<<lat.front () <<std::endl;
      lat.pop ();
      q.pop ();
    }
    float temp_time=avg_time/count_pcds;
    std::cout << "Average Latency : " <<temp_time << " msec" << std::endl;
 
  return 0;
}
}

int Sanity_Check(ros::NodeHandle n_h) {
  if(n_h.getParam("/map_path", map_path))
    std::cout << "Map path: " << map_path << std::endl;
  else {
    std::cout << "Map path: " << map_path << std::endl;
    std::cout << "map_path parameter not set" << std::endl;
    return -1;
  }
  if(n_h.getParam("/pc_path_1", point_cloud_path))
    std::cout << "Map path: " << point_cloud_path << std::endl;
  else {
    std::cout << "Map path: " << point_cloud_path<< std::endl;
    std::cout << "map_path parameter not set" << std::endl;
    return -1;
  }
  
  return 0;
}

void Initialize_Init_Guess() {
  std::cout << "Initializing init_guess" << std::endl;
/*   init_guess << 9.99999842e-01, -3.50288325e-04, -1.89105178e-04, -1.33695036e+02,
                3.50211293e-04,  9.99999712e-01, -4.06936777e-04,  1.37406246e+02,
                1.89247693e-04,  4.06870558e-04 , 9.99999799e-01,  1.08245653e+00,
                0.00, 0.00, 0.00, 1.00; */

                init_guess << 8.599999999999999867e-01, 0.000000000000000000e+00, 5.100000000000000089e-01, -3.74000000000000213e+00,
0.000000000000000000e+00, 1.000000000000000000e+00, -0.000000000000000000e+00, -2.060000000000000053e+00,
-5.100000000000000089e-01, 0.000000000000000000e+00, 8.599999999999999867e-01, 3.97000000000000195e+00,
0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00;
  //init_guess = Eigen::Matrix4f::Identity();
  return;
}

Eigen::Matrix4f Get_Init_Guess() {
  return init_guess;
}

void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform) {
  init_guess = prev_frame_transform;
  return;
}
