#include <iostream>
#include <Eigen/Dense>
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
#include <iomanip>
#include <cfloat>
#include <algorithm>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h> // C++17 standard header file name
#include <experimental/filesystem>

#include <omp.h>
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
int stop_count=0;
std::ofstream vehicle_pose_file;
std::ifstream initial_guess_list,Cells_count_file;
std::ofstream latency_results;
double latency_avg=0;
int pcd_count=0;

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


std::vector<std::string> splitString(std::string str, char splitter){
    std::vector<std::string> result;
    std::string current = "";
    for(int i = 0; i < str.size(); i++){
        if(str[i] == splitter){
            if(current != ""){
                result.push_back(current);
                current = "";
            }
            continue;
        }
        current += str[i];
    }
    if(current.size() != 0)
        result.push_back(current);
    return result;
}

Eigen::Matrix4f Get_Init_Guess()
{
  std::string line="";
  Eigen::Matrix4f init_guess;
  Eigen::Matrix4f temp;
  int k=0;
  for (int i=0;i<4;i++)
  {
    getline(initial_guess_list,line);
    std::vector<std::string> result = splitString(line, ' ');
    std::vector<std::string> result_last = splitString(result[3],'\n');
    for (int j=0;j<3;j++)
    {
      init_guess(k)=std::stof(result[j]);
      k++;
    }
    init_guess(k)=std::stof(result_last[0]);
    k++;
  }
  temp=init_guess.transpose();
  init_guess=temp;
  return init_guess;
}

Eigen::Matrix4f init_guess;

template <typename Registration>
void test(Registration& reg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
  double fitness_score = 0.0;
  auto t1 = std::chrono::high_resolution_clock::now();
  reg.setInputTarget(target);
  reg.setInputSource(source);
  reg.align(*aligned,init_guess);
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout<<"Fitness Score : "<<reg.getFitnessScore()<< std::endl;
  vehicle_pose_file<<reg.getFinalTransformation()<<std::endl;
  double single = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
  latency_results<<single<<std::endl;
  latency_avg= latency_avg+single;
  pcd_count=pcd_count+1;  
  }
  
int main(int argc, char** argv) {
  std::string Infra_path = std::string(argv[1])+"/Vehicle";
  std::string Vehicle_path = std::string(argv[1])+"/Leader";
  initial_guess_list.open(argv[2]);
  vehicle_pose_file.open(std::string(argv[1])+"/Direct_Alignment_Poses.txt");
  latency_results.open(std::string(argv[1])+"/Latencies.txt");
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr I_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr V_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  std::filesystem::path path_to_cloud_I=Infra_path;
  std::vector<std::filesystem::path> files_in_directory_I;
  std::copy(std::filesystem::directory_iterator(path_to_cloud_I), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory_I));
  std::sort(files_in_directory_I.begin(), files_in_directory_I.end(), customLess);

std::cout<<Infra_path<<std::endl;
  std::cout<<Vehicle_path<<std::endl;
  std::filesystem::path path_to_cloud_V=Vehicle_path;
  std::vector<std::filesystem::path> files_in_directory_V;
  std::copy(std::filesystem::directory_iterator(path_to_cloud_V), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory_V));
  std::sort(files_in_directory_V.begin(), files_in_directory_V.end(), customLess);

for (const std::string &filename : files_in_directory_V)
  {
    init_guess = Get_Init_Guess ();
    pcl::io::loadPCDFile<pcl::PointXYZ> (files_in_directory_V[pcd_count], *V_point_cloud);
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_V_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(0.3f, 0.3f, 0.3f);
    voxelgrid.setInputCloud(V_point_cloud);
    voxelgrid.filter(*downsampled_V_point_cloud);

   pcl::io::loadPCDFile<pcl::PointXYZ> (files_in_directory_I[pcd_count], *I_point_cloud);
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_I_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid1;
    voxelgrid1.setLeafSize(0.3f, 0.3f, 0.3f);
    voxelgrid1.setInputCloud(I_point_cloud);
    voxelgrid1.filter(*downsampled_I_point_cloud);
  fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp;
  
  #ifdef USE_VGICP_CUDA
  fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp_cuda;
  vgicp_cuda.setResolution(0.5);
  vgicp_cuda.setTransformationEpsilon(0.01);
  vgicp_cuda.setMaximumIterations(200);
  vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
  vgicp_cuda.setKernelWidth(2.0);
  test(vgicp_cuda,downsampled_V_point_cloud,downsampled_I_point_cloud);
  #endif
      } 
 std::cout<<"PCD count: "<<pcd_count<<std::endl;
 std::cout<<"Sum latency: "<<latency_avg<<std::endl;
 std::cout<<"Average Latency: "<<latency_avg/pcd_count<<std::endl;
 
  return 0;
}
