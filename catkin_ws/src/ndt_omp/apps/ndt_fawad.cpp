#include <iostream>
#include <filesystem>
#include <fstream>
#include <queue>
#include <ros/ros.h>
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

// Goal:
// Align point clouds with a map using NDT
// Inputs: Point cloud topic, map file
// Outputs: Aligned point clouds, map point cloud

// Constants
const std::string NODE_NAME = "align";
std::string map_path = "";
std::string vehicle_initial_guess_path ="";
std::string point_cloud_topic = "";
std::string aligned_pc_topic = "";
std::string map_pc_topic = "";
pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_xyz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_map_pc_xyz(new pcl::PointCloud<pcl::PointXYZ>());
ros::Subscriber pc_sub;
ros::Publisher aligned_pc_pub;
ros::Publisher map_pc_pub;
const int LIDAR_FREQUENCY = 10;
Eigen::Matrix4f init_guess;
//std::queue<Eigen::Matrix4f> q;

// Function prototypes
pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
std::vector<float> splitString(std::string str, char splitter);
bool Read_Point_Cloud(const std::string path_to_file, pcl::PCLPointCloud2::Ptr cloud);
int Sanity_Check(ros::NodeHandle nh);
bool Subscribe_To_PC_Topic(ros::NodeHandle *nh, const std::string pc_topic);
void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_msg);
pcl::PointCloud<pcl::PointXYZ>::Ptr NDT_Align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud);
void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);
void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
void Initialize_Init_Guess(std::string initial_guess_path);
void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform);
Eigen::Matrix4f Get_Init_Guess();
std::ofstream vehicle_pose_file;

int main(int argc, char **argv) {
  vehicle_pose_file.open("example.txt");
  ///// 1. Initialization Code /////
  // 1.1 Initialize ROS
  std::cout << "Initializing " << NODE_NAME << std::endl;
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  // 1.2 Argument sanity checks
  if(Sanity_Check(nh) == -1) {
    std::cout << "Exitting early" << std::endl;
    return -1;
  }

  // 1.3 Read map from file
  std::cout << "Reading map file ..." << std::endl;
  pcl::PCLPointCloud2::Ptr map_cloud(new pcl::PCLPointCloud2);
  if(Read_Point_Cloud(map_path, map_cloud)) {
    std::cout << "Successfully read map from file" << std::endl;
  } else {
    std::cout << "Failed to read map from file" << std::endl;
    return -1;
  }

  // Convert PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  pcl::fromPCLPointCloud2(*map_cloud, *map_pc_xyz);
  //*downsampled_map_pc_xyz = *map_pc_xyz;
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid1;
    voxelgrid1.setLeafSize(0.3f, 0.3f, 0.3f);

    voxelgrid1.setInputCloud (map_pc_xyz);
    voxelgrid1.filter ( *downsampled_map_pc_xyz );
  


  int num_threads = omp_get_max_threads();
  std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {{"DIRECT1", pclomp::DIRECT1}};
  ndt_omp->setResolution(1.0);
  ndt_omp->setStepSize(0.05);
  ndt_omp->setNumThreads(num_threads);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
  ndt_omp->setInputTarget(downsampled_map_pc_xyz);

  // 1.4 Advertise point cloud topics for aligned points and the map
  std::cout << "Advertising point cloud topics for aligned points and the map" << std::endl;
  Advertise_PC_Topic(&nh, aligned_pc_topic, &aligned_pc_pub, LIDAR_FREQUENCY);
  Advertise_PC_Topic(&nh, map_pc_topic, &map_pc_pub, 1);

  // 1.5 Initialize initial guess
  Initialize_Init_Guess(vehicle_initial_guess_path);

  /*
    // pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    // voxelgrid.setLeafSize(0.5f, 0.5f, 0.5f);
​
    // voxelgrid.setInputCloud (map_pc_xyz);
    // voxelgrid.filter ( *downsampled_map_pc_xyz );
  */

  // 1.6 Publish map point cloud
  std::cout << "Publishing map point cloud" << std::endl;
  Publish_Cloud(&map_pc_pub, 1, downsampled_map_pc_xyz);

  //// 2. Main Loop /////
  std::cout << "Subscribing to point cloud topic: " << point_cloud_topic << std::endl;
  pc_sub = nh.subscribe(point_cloud_topic, 10, Point_Cloud_Callback);
  std::cout << "Going into ros::spin() ..." << std::endl;

  ros::spin();
  //  while (not q.empty ())
  //   {
  //     // Output front of the queue
  //     vehicle_pose_file<<q.front () <<std::endl;
  //     // Pop the queue, delete item
  //     q.pop ();
  //   }
  //vehicle_pose_file<<current_frame_pose<<std::endl;
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

  if(n_h.getParam("/vehicle_pc_topic", point_cloud_topic))
    std::cout << "Point cloud topic: " << point_cloud_topic << std::endl;
  else {
    std::cout << "/point_cloud_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/aligned_pc_topic", aligned_pc_topic))
    std::cout << "Aligned point cloud topic: " << aligned_pc_topic << std::endl;
  else {
    std::cout << "/aligned_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/map_pc_topic", map_pc_topic))
    std::cout << "Map point cloud topic: " << map_pc_topic << std::endl;
  else {
    std::cout << "/map_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/vehicle_initial_guess_path", vehicle_initial_guess_path))
    std::cout << "vehicle_initial_guess_path:  " << vehicle_initial_guess_path << std::endl;
  else {
    std::cout << "/vehicle_initial_guess_path parameter not set" << std::endl;
    return -1;
  }
  // All good otherwise
  return 0;
}

bool Read_Point_Cloud(const std::string path_to_file, pcl::PCLPointCloud2::Ptr cloud) {
  std::cout << "Reading: " << path_to_file << std::endl << std::flush;
  if(pcl::io::loadPCDFile(path_to_file, *cloud) == -1)  //* load the file
    return false;
  else
    return true;
}
void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg) {
  static int pc_counter = 0;
  std::cout << "******************************" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud = msg->makeShared();
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input_cloud = msg->makeShared();

  // Dowsample vehicle point cloud for faster alignment
 auto t1 = ros::WallTime::now();


  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.2f, 0.2f, 0.2f);
  voxelgrid.setInputCloud(msg_cloud);
  voxelgrid.filter(*downsampled_cloud);
  auto t2 = ros::WallTime::now();
  auto duration1 = (t2 - t1).toSec();
     std::cout<<"Duration: "<<duration1*1000<<std::endl;
  msg_cloud = downsampled_cloud;

  std::cout << "---------- Point cloud: " << pc_counter << " -----------" << std::endl;

  // Set up NDT aligner


  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
/*   int num_threads = omp_get_max_threads();
  std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {{"DIRECT1", pclomp::DIRECT1}};
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt_omp->setResolution(1.0);
  ndt_omp->setStepSize(0.05);


  int counter = 0;
  for(const auto &search_method : search_methods) {
    ndt_omp->setNumThreads(num_threads);
    ndt_omp->setNeighborhoodSearchMethod(search_method.second); */
    auto t3 = ros::WallTime::now();
    aligned = NDT_Align(ndt_omp, msg_cloud);
    auto t4 = ros::WallTime::now();
    auto duration_ndt = (t4 - t3).toSec();
     std::cout<<"Duration_ndt: "<<duration_ndt*1000<<std::endl;
  //}

  // Transform the raw point cloud because we want to publish that one instead
  Eigen::Matrix4f transformation_matrix = Get_Init_Guess ();
  pcl::transformPointCloud(*raw_input_cloud, *raw_input_cloud, transformation_matrix);

  // Publish aligned point cloud
  Publish_Cloud(&aligned_pc_pub, pc_counter++, raw_input_cloud);
  Publish_Cloud(&map_pc_pub, 1, downsampled_map_pc_xyz);
}

// align point clouds and measure processing time
pcl::PointCloud<pcl::PointXYZ>::Ptr NDT_Align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud) {
  // Set input and output clouds
  //registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  // Registration

  registration->align(*aligned, init_guess);
  // Save transformation
  Eigen::Matrix4f current_frame_pose = registration->getFinalTransformation();
  //std::cout << "Vehicle Pose: "<<current_frame_pose<<std::endl;
  //q.push(current_frame_pose);
  std::cout << "Fitness: " << registration->getFitnessScore() << std::endl;

  Update_Init_Guess(current_frame_pose);
  return aligned;
}

void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string pc_topic, ros::Publisher *pub_ptr, int frequency) {
  // Advertise topics
  *pub_ptr = nh_ptr->advertise<sensor_msgs::PointCloud2>(pc_topic, frequency);
  std::cout << "Advertised: " << pc_topic << " at " << frequency << " Hz" << std::endl;
  return;
}

void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  input_cloud->header.frame_id = "os_sensor";
  pcl::PCLPointCloud2::Ptr input_cloud_pc2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*input_cloud, *input_cloud_pc2);
  pc_pub->publish(*input_cloud_pc2);
}

void Initialize_Init_Guess(std::string initial_guess_path) {

  std::cout << "Initializing init_guess" << std::endl;
  std::ifstream pose_file(initial_guess_path);
  std::string line;
  std::vector<float> pose ;

  while(getline (pose_file, line)) 
  {   
    std::vector<float> split_line = splitString(line, ' ');
    pose.insert(std::end(pose), std::begin(split_line), std::end(split_line));
  }
    

  init_guess << pose[0], pose[1], pose[2], pose[3],
                pose[4], pose[5], pose[6], pose[7],
                pose[8], pose[9], pose[10], pose[11],
                pose[12], pose[13], pose[14], pose[15]; 
  //init_guess = Eigen::Matrix4f::Identity();
  return;
}

std::vector<float> splitString(std::string str, char splitter){
  std::vector<float> result;
  std::string current = ""; 
  for(int i = 0; i < str.size(); i++){
    if(str[i] == splitter){
      if(current != ""){
        result.push_back(std::stof(current));
        current = "";
      } 
      continue;
    }
    current += str[i];
  }
  if(current.size() != 0)
    result.push_back(std::stof(current));
  return result;
}
Eigen::Matrix4f Get_Init_Guess() {
  return init_guess;
}

void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform) {
  init_guess = prev_frame_transform;
  return;
}