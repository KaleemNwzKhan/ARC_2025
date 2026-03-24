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
int count_break=0;
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
int strict_count=0;
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_Map(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);

template <typename Registration>
void test(Registration& reg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
  double fitness_score = 0.0;
  auto t1 = std::chrono::high_resolution_clock::now();
  reg.setInputSource(source);
  //reg.setInputTarget(downsampled_Map);
  //if (strict_count>70)
  //{
  reg.align(*aligned,init_guess);
  auto t2 = std::chrono::high_resolution_clock::now();
  fitness_score = reg.getFitnessScore();
  std::cout <<"Fitness Score: " <<fitness_score<< std::endl;
  Eigen::Matrix4f current_frame_pose = reg.getFinalTransformation();
  std::cout << "This is current frame pose: "<<current_frame_pose << std::endl;
  q.push(current_frame_pose);
  double single = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
  std::cout <<"Time: " <<single << std::endl;
  lat.push(single);
  avg_time=avg_time + (single);
  count_pcds=count_pcds+1;
  Update_Init_Guess(current_frame_pose);
  //}
   //strict_count=strict_count+1;
   //std:cout<<"File Count: "<< strict_count<<std::endl;
  }
std::string initial_guess_file,Vehicle_PCDs_path,Vehicle_Localization_Results_file,Vehicle_Localization_Latencies_file;
int main(int argc, char **argv) {

  initial_guess_file=argv[1];
  std::cout<<"First argument: "<< initial_guess_file<<std::endl;
  Vehicle_PCDs_path=argv[2];
  Vehicle_Localization_Results_file=argv[3];
  Vehicle_Localization_Results_file=Vehicle_Localization_Results_file+"/Poses.txt";
  Vehicle_Localization_Latencies_file=argv[4];
  Vehicle_Localization_Latencies_file=Vehicle_Localization_Latencies_file+"/Latencies.txt";
  map_path="Intersection_32_Beam.pcd";

  std::cout<<"Welcome"<<std::endl;

 struct
  {
    bool operator()(std::filesystem::path a, std::filesystem::path b) const
    {
      std::string a1 = a;
      std::string b1 = b;
      int a2 = stoi(a1.substr(a1.find_last_of("/\\") + 1));
      int b2 = stoi(b1.substr(b1.find_last_of("/\\") + 1));
      return a2 < b2;
    }
  } customLess;

  vehicle_pose_file.open(Vehicle_Localization_Results_file);
  loc_latency.open(Vehicle_Localization_Latencies_file);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> (map_path, *map);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_Map(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.6f, 0.6f, 0.6f);
  voxelgrid.setInputCloud(map);
  voxelgrid.filter(*downsampled_Map);

  std::filesystem::path path_to_cloud=Vehicle_PCDs_path;
  Initialize_Init_Guess();

  Eigen::Matrix4f transformation_matrix = Get_Init_Guess ();
  std::vector<std::filesystem::path> files_in_directory;
  std::copy(std::filesystem::directory_iterator(path_to_cloud), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
  std::sort(files_in_directory.begin(), files_in_directory.end(), customLess);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  #ifdef USE_VGICP_CUDA
  fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ> ndt_cuda;
  ndt_cuda.setMaximumIterations (100);
  ndt_cuda.setTransformationEpsilon (0.00001);
  ndt_cuda.setResolution(3.0);
  // ndt_cuda.setTransformationEpsilon (0.01);
  // ndt_cuda.setResolution(0.45);
  ndt_cuda.setInputTarget(downsampled_Map);
  // ndt_cuda.setInputTarget(map);
  ndt_cuda.setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
  ndt_cuda.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);

  for (const std::string &filename1 : files_in_directory)
  {
    std::cout<<"File names: "<<filename1<<std::endl;
  pcl::io::loadPCDFile<pcl::PointXYZ> (filename1, *point_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid1;
    voxelgrid1.setLeafSize(0.2f, 0.2f, 0.2f);
    voxelgrid1.setInputCloud(point_cloud);
    voxelgrid1.filter(*downsampled_cloud);
    test(ndt_cuda, downsampled_cloud);
    /* if (count_break>500)
    {
      break;
    } 
    count_break=count_break+1;  */
 
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

   std::ifstream file(initial_guess_file); // Replace "matrix.txt" with your file name
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file!" << std::endl;
        return;
    }

    double value;

    // Read the matrix values from the file
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!(file >> value)) {
                std::cerr << "Error: Insufficient data in the file!" << std::endl;
                return;
            }
            init_guess(i, j) = value;
        }
    }

    file.close();

// init_guess <<1.947071837093972135e-07, 1.000000000000000000e+00, 0.000000000000000000e+00, -4.700000000000000000e+01,
// -1.000000000000000000e+00, 1.947071837093972135e-07, 0.000000000000000000e+00, 1.725000000000000000e+01,
// 0.000000000000000000e+00, -0.000000000000000000e+00, 1.000000000000000000e+00, 7.189166069030761719e+00,
// 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00;

return;
}
Eigen::Matrix4f Get_Init_Guess() {
  return init_guess;
}

void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform) {
  init_guess = prev_frame_transform;
  return;
}
