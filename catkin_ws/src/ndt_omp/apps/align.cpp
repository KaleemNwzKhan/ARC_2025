#include <ros/ros.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pclomp/ndt_omp.h>
#include <fast_gicp/Transform.h>

// //#ifdef USE_VGICP_CUDA
// #include <fast_gicp/ndt/ndt_cuda.hpp>
// #include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
// //#endif

// Goal:
// Align point clouds with a map using NDT
// Inputs: Point cloud topic, map file
// Outputs: Aligned point clouds, map point cloud

// variables to get from rospram 
std::string map_path = "";
std::string vehicle_pc_topic = "";
std::string aligned_pc_topic = "";
std::string map_pc_topic = "";
std::string vehicle_transform_topic = "";
std::string vehicle_initial_guess_path="";

// subscribers and publishers
ros::Subscriber pc_sub;
ros::Publisher aligned_pc_pub;
ros::Publisher map_pc_pub;
ros::Publisher transform;


// global variables
const std::string NODE_NAME = "vehicle_ndt";
const int LIDAR_FREQUENCY = 10;
const int QUEUE_SIZE_PUB = 10;
const int QUEUE_SIZE_SUB = 1;

Eigen::Matrix4f init_guess;
fast_gicp::Transform tf;

// global objects
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;


// Function prototypes
int Sanity_Check(ros::NodeHandle nh);
bool Read_Point_Cloud(const std::string path_to_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_msg);

void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);
void Advertise_Transform(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);

void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
void Publish_Transform(ros::Publisher *pc_pub, int frame_number,  fast_gicp::Transform tf);

bool Subscribe_To_PC_Topic(ros::NodeHandle *nh, const std::string pc_topic);

std::vector<float> splitString(std::string str, char splitter);
Eigen::Matrix4f Initialize_Init_Guess1(std::string initial_guess_path);
void Initialize_Init_Guess();

void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform);
void Update_Transform(Eigen::Matrix4f prev_frame_transform);
Eigen::Matrix4f Get_Init_Guess();
fast_gicp::Transform  Get_Transform();

pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr NDT_Align(pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud);


int main(int argc, char **argv) {
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


  //ros::Rate loop_rate(10);

  // 1.3 Read basemap and downsample it
  std::cout << "Reading map file ..." << std::endl;
  

  if(Read_Point_Cloud(map_path, map_cloud)) 
  {
    std::cout << "Successfully read map from file" << std::endl;
  } 
  else 
  {
    std::cout << "Failed to read map from file" << std::endl;
    return -1;
  }


  voxelgrid.setLeafSize(0.9f, 0.9f, 0.9f);
  voxelgrid.setInputCloud (map_cloud);
  voxelgrid.filter (*map_cloud);
  

  // 1.4 Advertise point cloud topics for aligned points and the map
  std::cout << "Advertising point cloud topics for aligned points and the map" << std::endl;
  Advertise_PC_Topic(&nh, aligned_pc_topic, &aligned_pc_pub, QUEUE_SIZE_PUB);
  Advertise_PC_Topic(&nh, map_pc_topic, &map_pc_pub, QUEUE_SIZE_PUB);
  Advertise_Transform(&nh,vehicle_transform_topic, &transform, QUEUE_SIZE_PUB);

  // 1.5 Initialize initial guess
  //init_guess =  Initialize_Init_Guess(vehicle_initial_guess_path);
  Initialize_Init_Guess();

  //inintialize ndt
  int num_threads = omp_get_max_threads();
  ndt_omp->setNumThreads(num_threads);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
  ndt_omp->setResolution(5.0);
  ndt_omp->setStepSize(5);
  ndt_omp->setInputTarget(map_cloud);


  voxelgrid.setLeafSize(0.6f, 0.6f, 0.6f);


  tf.header.frame_id = "os_sensor";



  // 1.6 Publish map point cloud
  std::cout << "Publishing map point cloud" << std::endl;
  Publish_Cloud(&map_pc_pub, 1, map_cloud);

  //// 2. Main Loop /////
  std::cout << "Subscribing to point cloud topic: " << vehicle_pc_topic << std::endl;
  pc_sub = nh.subscribe(vehicle_pc_topic, QUEUE_SIZE_SUB, Point_Cloud_Callback);
  std::cout << "Going into ros::spin() ..." << std::endl;
  ros::spin();
  //loop_rate.sleep();

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

  if(n_h.getParam("/vehicle_pc_topic", vehicle_pc_topic))
    std::cout << "vehicle_pc_topic: " << vehicle_pc_topic << std::endl;
  else {
    std::cout << "/vehicle_pc_topic parameter not set" << std::endl;
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

  if(n_h.getParam("/vehicle_transform_topic", vehicle_transform_topic))
    std::cout << "vehicle transform topic: " << vehicle_transform_topic << std::endl;
  else {
    std::cout << "/vehicle_transform_topic parameter not set" << std::endl;
    return -1;
  }


  if(n_h.getParam("/vehicle_initial_guess_path", vehicle_initial_guess_path))
    std::cout << "vehicle_initial_guess_path:  " << vehicle_initial_guess_path << std::endl;
  else {
    std::cout << "/vehicle_initial_guess_path parameter not set" << std::endl;
    return -1;
  }

  return 0;
}

bool Read_Point_Cloud(const std::string path_to_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::cout << "Reading: " << path_to_file << std::endl << std::flush;
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_to_file, *cloud) == -1)  //* load the file
    return false;
  else
    return true;
}

void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg) {

  auto t1 = ros::WallTime::now();

  static int pc_counter = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input_cloud = msg->makeShared();

  // Dowsample vehicle point cloud for faster alignment
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  voxelgrid.setInputCloud(raw_input_cloud);
  voxelgrid.filter(*downsampled_cloud);
 
  auto t2 = ros::WallTime::now();

  // std::cout << "---------- Point cloud: " << pc_counter << " -----------" << std::endl;


  //aligned = NDT_Align(ndt_omp, downsampled_cloud);
  ndt_omp = NDT_Align(ndt_omp, downsampled_cloud);
  auto t3 = ros::WallTime::now();


  Publish_Transform(&transform, 1, Get_Transform());

  // auto t4 = ros::WallTime::now();
  // auto duration_downsampling = (t2 - t1).toSec();
  // auto duration_ndt = (t3 - t2).toSec();
  // auto duration_publish = (t4 - t3).toSec();

  //std::cout << "Fitness: " << registration->getFitnessScore() << std::endl;
  // std::cout << "Latency_downsampling :" << duration_downsampling * 1000 << ": msec" << std::endl;
  // std::cout << "Latency_ndt :" << duration_ndt * 1000 << ": msec" << std::endl;
  // std::cout << "Latency_publish :" << duration_publish * 1000 << ": msec" << std::endl;
  // std::cout << "Fitness :" << ndt_omp->getFitnessScore() << ": score" << std::endl;



  // Transform the raw point cloud because we want to publish that one instead
  pcl::transformPointCloud(*raw_input_cloud, *raw_input_cloud, Get_Init_Guess ());
  // Publish aligned point cloud
  Publish_Cloud(&aligned_pc_pub, pc_counter++, raw_input_cloud);
  Publish_Cloud(&map_pc_pub, 1, map_cloud);
  //pc_counter++;

}

// align point clouds and measure processing time
pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr NDT_Align(pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud) {
 
  // Set input and output clouds
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  // Registration
  //auto t1 = ros::WallTime::now();
  registration->align(*aligned, init_guess);
  //auto t2 = ros::WallTime::now();

  // Save transformation
  Eigen::Matrix4f current_frame_pose = registration->getFinalTransformation();
  //auto t3 = ros::WallTime::now();
  std::cout << "Fitness :" << registration->getFitnessScore() << ": score" << std::endl;
  //std::cout << "Latency_ndt :" << duration_ndt * 1000 << ": msec" << std::endl;
  //auto t4 = ros::WallTime::now();
  Update_Init_Guess(current_frame_pose);
  Update_Transform(current_frame_pose);
  //auto t5= ros::WallTime::now();

  // auto duration_ndt = (t2 - t1).toSec();
  // auto duration_get_final_trans = (t3 - t2).toSec();
  // auto duration_print = (t4 - t3).toSec();
  // auto duration_update = (t5 - t4).toSec();
  // std::cout << "Latency_ndt :" << duration_ndt * 1000 << ": msec" << std::endl;
  // std::cout << "Latency_final_t :" << duration_get_final_trans * 1000 << ": msec" << std::endl;
  // std::cout << "Latency_print:" << duration_print * 1000 << ": msec" << std::endl;
  // std::cout << "Latency_update :" << duration_update * 1000 << ": msec" << std::endl;




  return registration;
}

void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string pc_topic, ros::Publisher *pub_ptr, int frequency) {
  // Advertise topics
  *pub_ptr = nh_ptr->advertise<sensor_msgs::PointCloud2>(pc_topic, frequency);
  std::cout << "Advertised: " << pc_topic << " at " << frequency << " Hz" << std::endl;
  return;
}

void Advertise_Transform(ros::NodeHandle *nh_ptr, std::string transform_topic, ros::Publisher *pub_ptr, int queue_size) {

  // Advertise topics
  *pub_ptr = nh_ptr->advertise<fast_gicp::Transform>(transform_topic, queue_size);
  std::cout << "Advertised: " << transform_topic << " with queue size " << queue_size  << std::endl;
  return;
}


void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  pcl::PCLPointCloud2::Ptr input_cloud_pc2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*input_cloud, *input_cloud_pc2);
  input_cloud_pc2->header.frame_id = "os_sensor";
  pcl_conversions::toPCL(ros::Time::now(), input_cloud_pc2->header.stamp);
  pc_pub->publish(*input_cloud_pc2);
}

void Publish_Transform(ros::Publisher *pc_pub, int frame_number,  fast_gicp::Transform tf) {
  tf.header.frame_id = "os_sensor";
  tf.header.stamp = ros::Time::now();
  pc_pub->publish(tf);
}

void Initialize_Init_Guess() {
  std::cout << "Initializing init_guess" << std::endl;
//   init_guess <<     0.999989 ,0.000213231 ,  -0.00480045  , -0.0266008,
//  -0.00020679,    0.999999,  0.00134219,  -0.0328619,
//   0.00480073, -0.00134119,    0.999988,   0.063891,
//   0    ,        0     ,       0      ,      1;
  init_guess = Eigen::Matrix4f::Identity();
  return;
}

Eigen::Matrix4f Get_Init_Guess() {
  return init_guess;
}

fast_gicp::Transform Get_Transform() {
  return tf;
}

void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform) {
  init_guess = prev_frame_transform;
  return;
}

void Update_Transform( Eigen::Matrix4f prev_frame_transform) {
  tf.data = {prev_frame_transform(0,0), prev_frame_transform(0,1), prev_frame_transform(0,2), prev_frame_transform(0,3), 
        prev_frame_transform(1,0), prev_frame_transform(1,1), prev_frame_transform(1,2), prev_frame_transform(1,3),
        prev_frame_transform(2,0), prev_frame_transform(2,1), prev_frame_transform(2,2), prev_frame_transform(2,3)};

  return;
}

Eigen::Matrix4f Initialize_Init_Guess(std::string initial_guess_path) {

  Eigen::Matrix4f pose_matrix;
  std::cout << "Initializing init_guess" << std::endl;
  std::ifstream pose_file(initial_guess_path);
  std::string line;
  std::vector<float> pose ;

  while(getline (pose_file, line)) 
  {   
    std::vector<float> split_line = splitString(line, ' ');
    pose.insert(std::end(pose), std::begin(split_line), std::end(split_line));
  }
    

  pose_matrix << pose[0], pose[1], pose[2], pose[3],
                pose[4], pose[5], pose[6], pose[7],
                pose[8], pose[9], pose[10], pose[11],
                pose[12], pose[13], pose[14], pose[15]; 
  //init_guess = Eigen::Matrix4f::Identity();
  return pose_matrix;
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


//////////////////////////////////////////////////////////////////////////////
