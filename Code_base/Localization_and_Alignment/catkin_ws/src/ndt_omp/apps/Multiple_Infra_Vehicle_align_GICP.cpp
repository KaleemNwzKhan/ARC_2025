#include <ros/ros.h>
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
std::string vehicle_point_cloud_path = "";
std::string infra_point_cloud_path = "";
int Sanity_Check(ros::NodeHandle nh);
std::ifstream vehicle_files_list;
std::ifstream infra_files_list;
std::ifstream initial_guess_list;
std::ofstream vehicle_pose_file;
std::ofstream loc_latency;

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
  Eigen::Matrix4f init_guess1;
  Eigen::Matrix4f temp;
  int k=0;
  for (int i=0;i<4;i++)
  {
    getline(initial_guess_list,line);
    std::vector<std::string> result = splitString(line, ' ');
    std::vector<std::string> result_last = splitString(result[3],'\n');
    for (int j=0;j<3;j++)
    {
      init_guess1(k)=std::stof(result[j]);
      k++;
    }
    init_guess1(k)=std::stof(result_last[0]);
    k++;
  }
  temp=init_guess1.transpose();
  //init_guess1=temp;
   init_guess1 << 8.599999999999999867e-01, 0.000000000000000000e+00, 5.100000000000000089e-01, -3.74000000000000213e+00,
0.000000000000000000e+00, 1.000000000000000000e+00, -0.000000000000000000e+00, -2.060000000000000053e+00,
-5.100000000000000089e-01, 0.000000000000000000e+00, 8.599999999999999867e-01, 3.97000000000000195e+00,
0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00;
  return init_guess1;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud) {
pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
auto t1 = ros::WallTime::now();
registration->setInputTarget(target_cloud);
registration->setInputSource(source_cloud);
registration->align(*aligned, init_guess);
auto t2 = ros::WallTime::now();
auto duration = (t2 - t1).toSec();
  Eigen::Matrix4f current_frame_pose = registration->getFinalTransformation();
  std::cout << "Vehicle Pose: "<<current_frame_pose<<std::endl;
  q.push(current_frame_pose);
  std::cout << "Fitness: " << registration->getFitnessScore() << std::endl;
  lat.push(duration * 1000);
  avg_time=avg_time + (duration * 1000);
  count_pcds=count_pcds+1;
  std::cout << "Latency : " << duration * 1000 << " msec" << std::endl;
  return aligned;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "Kaleem");
  initial_guess_list.open("Closer_AUTOPASS.txt");
  vehicle_files_list.open("vehicle_files_list.txt");
  infra_files_list.open("infra_files_list.txt");
  vehicle_pose_file.open("example.txt");
  loc_latency.open("latency.txt");
  ros::NodeHandle n_h;
  pcl::PointCloud<pcl::PointXYZ>::Ptr V_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr I_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (int frame_number=0;frame_number<100;frame_number++)
  {

    init_guess = Get_Init_Guess ();
    std::cout << "Initial Guess : " << init_guess << std::endl;
    getline (vehicle_files_list,vehicle_point_cloud_path);

    pcl::io::loadPCDFile<pcl::PointXYZ> (vehicle_point_cloud_path, *V_point_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_V_point_cloud_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(1.0, 1.0f, 1.0f);
    voxelgrid.setInputCloud(V_point_cloud);
    voxelgrid.filter(*downsampled_V_point_cloud_cloud);

    getline (infra_files_list,infra_point_cloud_path);
    pcl::io::loadPCDFile<pcl::PointXYZ> (infra_point_cloud_path, *I_point_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_I_point_cloud_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid1;
    voxelgrid1.setLeafSize(1.5f, 1.5f, 1.5f);
    voxelgrid1.setInputCloud(I_point_cloud);
    voxelgrid1.filter(*downsampled_I_point_cloud_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    std::cout << "--- pclomp::GICP ---" << std::endl;
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp_omp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
    gicp_omp->setTransformationEpsilon(2e-2);
    gicp_omp->setMaxCorrespondenceDistance(0.01);
    gicp_omp->setMaximumIterations(100);
    aligned = align(gicp_omp, downsampled_V_point_cloud_cloud, downsampled_I_point_cloud_cloud);
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

