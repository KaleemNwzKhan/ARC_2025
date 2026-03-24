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
#include <pcl/visualization/cloud_viewer.h> 
#include <experimental/filesystem>

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

Eigen::Matrix4f init_guess;
std::queue<Eigen::Matrix4f> q;
std::queue<float> lat;
int count_pcds=0;
float avg_time=0;
std::string vehicle_point_cloud_path = "";
std::string infra_point_cloud_path = "";
std::ifstream vehicle_files_list;
std::ifstream infra_files_list;
std::ifstream initial_guess_list;
std::ofstream vehicle_pose_file;
std::ofstream loc_latency;
 pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_I_point_cloud_cloud(new pcl::PointCloud<pcl::PointXYZ>());

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
  Eigen::Matrix4f I_W;
  Eigen::Matrix4f Guess;
  Eigen::Matrix4f temp,temp1;
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

  I_W <<0.87, -0.0,0, 0.50, -91.70,
0.00, 1.00, 0.00, 135.40,
-0.50, -0.00, 0.87, 5.00,
0.00, 0.00, 0.00, 1.00;

  temp=init_guess.transpose();
  init_guess=temp;
  temp1=init_guess.inverse();
  Guess=temp1*I_W;
  return init_guess;
}

template <typename Registration>
void test(Registration& reg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);

  double fitness_score = 0.0;
  reg.setInputSource(downsampled_I_point_cloud_cloud);
  auto t1 = std::chrono::high_resolution_clock::now();
  reg.clearTarget();
  reg.setInputTarget(target);
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
  std::cout << "Latency : " << single << " msec" << std::endl;
  }

int main(int argc, char** argv) {
  initial_guess_list.open("example_3.txt");
  vehicle_files_list.open("vehicle_files_list.txt");
  infra_files_list.open("infra_files_list.txt");
  vehicle_pose_file.open("example.txt");
  loc_latency.open("latency.txt");
  pcl::PointCloud<pcl::PointXYZ>::Ptr V_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr I_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("/dataset/Infra-100-closest/351.pcd", *I_point_cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid1;
  voxelgrid1.setLeafSize(1.0f, 1.0f, 1.0f);
  voxelgrid1.setInputCloud(I_point_cloud);
  voxelgrid1.filter(*downsampled_I_point_cloud_cloud);
  for (int frame_number=0;frame_number<100;frame_number++)
  { 
	  init_guess = Get_Init_Guess ();
    std::cout << "Hello3 : " << std::endl;
    std::cout << "Initial Guess : " << init_guess << std::endl;
    getline (vehicle_files_list,vehicle_point_cloud_path,'\n');
    std::cout<<vehicle_point_cloud_path<<endl;
     if ( pcl::io::loadPCDFile<pcl::PointXYZ> (vehicle_point_cloud_path, *V_point_cloud)==-1)
     {
             PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
             return (-1);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_V_point_cloud_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(1.0, 1.0f, 1.0f);
    voxelgrid.setInputCloud(V_point_cloud);
    voxelgrid.filter(*downsampled_V_point_cloud_cloud);

  fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp;
  vgicp.setResolution(1.0);
  vgicp.setNumThreads(1);

  std::cout << "--- vgicp_mt ---" << std::endl;
  vgicp.setNumThreads(omp_get_max_threads());
  test(vgicp, downsampled_V_point_cloud_cloud);
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