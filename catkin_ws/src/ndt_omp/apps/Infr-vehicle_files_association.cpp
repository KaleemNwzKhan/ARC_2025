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
std::string vehicle_point_cloud_path = "";
std::string infra_point_cloud_path = "";
int Sanity_Check(ros::NodeHandle nh);
std::ofstream vehicle_files_list;
std::ofstream infra_files_list;

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
  vehicle_files_list.open("vehicle_files_list.txt");
  infra_files_list.open("infra_files_list.txt");
  ros::NodeHandle n_h;
  if(Sanity_Check(n_h) == -1) {
    std::cout << "Exitting early" << std::endl;
    return -1;
  }

  std::filesystem::path path_to_vehicle=vehicle_point_cloud_path;
  std::vector<std::filesystem::path> files_in_directory_v;
  std::copy(std::filesystem::directory_iterator(path_to_vehicle), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory_v));
  std::sort(files_in_directory_v.begin(), files_in_directory_v.end(), customLess);
  for (const std::string &filename1 : files_in_directory_v)
  {
     vehicle_files_list<<filename1 <<std::endl;
  }

  std::filesystem::path path_to_infra=infra_point_cloud_path;
  std::vector<std::filesystem::path> files_in_directory;
  std::copy(std::filesystem::directory_iterator(path_to_infra), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
  std::sort(files_in_directory.begin(), files_in_directory.end(), customLess);
  for (const std::string &filename2 : files_in_directory)
  {
    infra_files_list<<filename2 <<std::endl;
  }

}

int Sanity_Check(ros::NodeHandle n_h)
{
  if(n_h.getParam("/pc_path_1", vehicle_point_cloud_path))
    std::cout << "Vehicle path: " << vehicle_point_cloud_path << std::endl;
  else {
    std::cout << "Vehicle path: " << vehicle_point_cloud_path<< std::endl;
    std::cout << "Vehicle_path parameter not set" << std::endl;
    return -1;
  }
  if(n_h.getParam("/pc_path_2", infra_point_cloud_path))
    std::cout << "Infrastructure path: " << infra_point_cloud_path << std::endl;
  else {
    std::cout << "Infrastructure path: " << infra_point_cloud_path<< std::endl;
    std::cout << "Infrastructure_path parameter not set" << std::endl;
    return -1;
  }
  
  return 0;
}


