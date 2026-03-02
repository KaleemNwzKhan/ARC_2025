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
pcl::PointCloud<pcl::PointXYZ>::Ptr NDT_Align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud);
void Initialize_Init_Guess();
void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform);
Eigen::Matrix4f Get_Init_Guess();
std::ofstream vehicle_pose_file;
std::ofstream loc_latency;

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
  //std::cout << "Initializing " << NODE_NAME << std::endl;


  ros::init(argc, argv, "Kaleem");
  vehicle_pose_file.open("example.txt");
  loc_latency.open("latency.txt");
  ros::NodeHandle n_h;
  if(Sanity_Check(n_h) == -1) {
    std::cout << "Exitting early" << std::endl;
    return -1;
  }


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
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_to_cloud/filename1, *point_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }	

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid1;
    voxelgrid1.setLeafSize(0.6f, 0.6f, 0.6f);
    voxelgrid1.setInputCloud(point_cloud);
    voxelgrid1.filter(*downsampled_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    int num_threads = omp_get_max_threads();
    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_omp->setResolution(5.0);
    ndt_omp->setStepSize(5.0);
    ndt_omp->setNumThreads(num_threads);
    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    ndt_omp->setInputTarget(downsampled_map);
    aligned = NDT_Align(ndt_omp, downsampled_cloud);
 
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
pcl::PointCloud<pcl::PointXYZ>::Ptr NDT_Align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud) {
  // Set input and output clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  // Registration
  auto t1 = ros::WallTime::now();
  registration->setInputSource(source_cloud);
  registration->align(*aligned, init_guess);
  auto t2 = ros::WallTime::now();
  auto duration = (t2 - t1).toSec();
  // Save transformation
  Eigen::Matrix4f current_frame_pose = registration->getFinalTransformation();
  //std::cout << "Vehicle Pose: "<<current_frame_pose<<std::endl;
  q.push(current_frame_pose);
  std::cout << "Fitness: " << registration->getFitnessScore() << std::endl;
  lat.push(duration * 1000);
  avg_time=avg_time+(duration * 1000);
  count_pcds=count_pcds+1;
  std::cout << "Latency : " << duration * 1000 << " msec" << std::endl;

  Update_Init_Guess(current_frame_pose);
  return aligned;
}

void Initialize_Init_Guess() {
  std::cout << "Initializing init_guess" << std::endl;
   init_guess << 9.99999842e-01, -3.50288325e-04, -1.89105178e-04, -1.33695036e+02,
                3.50211293e-04,  9.99999712e-01, -4.06936777e-04,  1.37406246e+02,
                1.89247693e-04,  4.06870558e-04 , 9.99999799e-01,  1.08245653e+00,
                0.00, 0.00, 0.00, 1.00; 

// init_guess <<1.00, 0.00, 0.00, -102.47,
// -0.00, 1.00, -0.00, 137.40,
// -0.00, 0.00, 1.00, 1.03,
// 0.00, 0.00, 0.00, 1.00;

                /* init_guess << 8.599999999999999867e-01, 0.000000000000000000e+00, 5.100000000000000089e-01, -3.740000000000000213e+00,
0.000000000000000000e+00, 1.000000000000000000e+00, -0.000000000000000000e+00, -2.060000000000000053e+00,
-5.100000000000000089e-01, 0.000000000000000000e+00, 8.599999999999999867e-01, 3.970000000000000195e+00,
0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00; */
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
