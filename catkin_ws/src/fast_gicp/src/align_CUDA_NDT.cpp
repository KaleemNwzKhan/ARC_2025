// #include <ros/ros.h>

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem>
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
//#include <pcl_ros/point_cloud.h>
//#include <pclomp/ndt_omp.h>
//#include <pclomp/gicp_omp.h>
#include <iomanip>
#include <cfloat>
#include <algorithm>
//#include <pcl_conversions/pcl_conversions.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h> // C++17 standard header file name
#include <experimental/filesystem>

//#include <ros/ros.h>
#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#ifdef USE_VGICP_CUDA
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

Eigen::Matrix4f init_guess;
std::queue<Eigen::Matrix4f> q;
std::queue<float> lat;
int count_pcds=0;
float avg_time=0;
std::string map_path = "";
std::string point_cloud_path = "";
void Initialize_Init_Guess();
void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform);
Eigen::Matrix4f Get_Init_Guess();
std::ofstream vehicle_pose_file;
std::ofstream loc_latency;



template <typename Registration>
void test(Registration& reg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
  double fitness_score = 0.0;
  auto t1 = std::chrono::high_resolution_clock::now();
  reg.setInputSource(source);
  reg.align(*aligned,init_guess);
  auto t2 = std::chrono::high_resolution_clock::now();
  fitness_score = reg.getFitnessScore();
  Eigen::Matrix4f current_frame_pose = reg.getFinalTransformation();
  std::cout << current_frame_pose << std::endl;
  q.push(current_frame_pose);
  double single = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
  lat.push(single);
  avg_time=avg_time + (single);
  count_pcds=count_pcds+1;
  Update_Init_Guess(current_frame_pose);
  }


int main(int argc, char **argv) {
  map_path="Downsampled_Map_0.9.pcd";
  point_cloud_path="Vehicle/";

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

  vehicle_pose_file.open("Vehicle_poses.txt");
  loc_latency.open("localization_latency.txt");

  std::cout << "Reading map file ..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_map (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> (map_path, *downsampled_map);

  std::filesystem::path path_to_cloud=point_cloud_path;
  Initialize_Init_Guess();
  Eigen::Matrix4f transformation_matrix = Get_Init_Guess ();
  std::vector<std::filesystem::path> files_in_directory;
  std::copy(std::filesystem::directory_iterator(path_to_cloud), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
  std::sort(files_in_directory.begin(), files_in_directory.end(), customLess);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (const std::string &filename1 : files_in_directory)
  {
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename1, *point_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }	
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid1;
    voxelgrid1.setLeafSize(0.6f, 0.6f, 0.6f);
    voxelgrid1.setInputCloud(point_cloud);
    voxelgrid1.filter(*downsampled_cloud);

    
 #ifdef USE_VGICP_CUDA
  fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ> ndt_cuda;
  ndt_cuda.setResolution(5.0);
  ndt_cuda.setInputTarget(downsampled_map);
  std::cout << "--- ndt_cuda (D2D) ---" << std::endl;
  ndt_cuda.setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
  test(ndt_cuda, downsampled_cloud); 
 
#endif
  }
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
void Initialize_Init_Guess() {
  std::cout << "Initializing init_guess" << std::endl;
   init_guess << 1.00, 0.00, -0.00, -133.70,
                0.00, 1.00, 0.00, 137.40,
                0.00, -0.00, 1.00, 0.99,
                0.00, 0.00, 0.00, 1.00; 
  return;
}
Eigen::Matrix4f Get_Init_Guess() {
  return init_guess;
}

void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform) {
  init_guess = prev_frame_transform;
  return;
}