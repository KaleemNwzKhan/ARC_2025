#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <fast_gicp/Transform.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <pcl/filters/voxel_grid.h>


// variables to get from rospram 
std::string map_path = "";
std::string vehicle_pc_topic = "";
// std::string map_pc_topic = "";
std::string vehicle_transform_topic = "";
std::string vehicle_initial_guess_path ="";

// subscribers and publishers
ros::Subscriber pc_sub;
ros::Publisher transform;


// global variables
const std::string NODE_NAME = "vehicle_localization_pipeline_1";
const int LIDAR_FREQUENCY = 10;
const int QUEUE_SIZE_SUB = 1;
const int QUEUE_SIZE_PUB = 1;
Eigen::Matrix4f init_guess;
fast_gicp::Transform tf;


// global objects
std::ofstream log_time_ndt, ndt_poses;
pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_map,voxelgrid_pc;
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr d_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ> ndt_cuda;



// Function prototypes
int Sanity_Check(ros::NodeHandle nh);

bool Read_Point_Cloud(const std::string path_to_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg);

void Advertise_Transform(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);

void Publish_Transform(ros::Publisher *pc_pub, int frame_number,  fast_gicp::Transform tf);

bool Subscribe_To_PC_Topic(ros::NodeHandle *nh, const std::string pc_topic);

Eigen::Matrix4f Initialize_Init_Guess(std::string initial_guess_path);
std::vector<float> splitString(std::string str, char splitter);
void Update_Init_Guess(Eigen::Matrix4f prev_frame_transform);
void Update_Transform(Eigen::Matrix4f prev_frame_transform);
Eigen::Matrix4f Get_Init_Guess();
fast_gicp::Transform  Get_Transform();

pcl::PointCloud<pcl::PointXYZ>::Ptr NDT_Align(fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ>& registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud);


int main(int argc, char **argv) {

  int append = atoi(argv[1]);

  if (append ==1){
    log_time_ndt.open("/workspace/log_time_ndt_v1.csv",std::ios::app );
    ndt_poses.open("/workspace/ndt_v1.txt", std::ios::app );
  }
  else {
    log_time_ndt.open("/workspace/log_time_ndt_v1.csv" );
    ndt_poses.open("/workspace/ndt_v1.txt" );
    log_time_ndt << "frame_number" << "," << "latency_downsample" << "," << "latency_compression" << ","<< "fitness" << "," << "latency_ndt"<<std::endl;
  }

  std::cout << "Initializing " << NODE_NAME << std::endl;
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  if(Sanity_Check(nh) == -1) {
    std::cout << "Exitting early" << std::endl;
    return -1;
  }
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

  voxelgrid_map.setLeafSize(0.9f, 0.9f, 0.9f);
  voxelgrid_map.setInputCloud (map_cloud);
  voxelgrid_map.filter (*d_map_cloud);

  voxelgrid_pc.setLeafSize(0.6f, 0.6f, 0.6f);
  ndt_cuda.setResolution(5.0);
  ndt_cuda.setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
  ndt_cuda.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
  ndt_cuda.setInputTarget(d_map_cloud);

  std::cout << "Advertising point cloud topics for aligned points and the map" << std::endl;
  Advertise_Transform(&nh,vehicle_transform_topic, &transform, QUEUE_SIZE_PUB);

  init_guess = Initialize_Init_Guess(vehicle_initial_guess_path);

  std::cout << "Publishing map point cloud" << std::endl;
 

  //// 2. Main Loop /////
  std::cout << "Subscribing to point cloud topic: " << vehicle_pc_topic << std::endl;
  pc_sub = nh.subscribe(vehicle_pc_topic, QUEUE_SIZE_SUB, Point_Cloud_Callback);
  std::cout << "Going into ros::spin() ..." << std::endl;
  ros::spin();

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

  if(n_h.getParam("/vehicle_1_pc_topic", vehicle_pc_topic))
    std::cout << "vehicle_pc_topic: " << vehicle_pc_topic << std::endl;
  else {
    std::cout << "/vehicle_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/vehicle_1_transform_topic", vehicle_transform_topic))
    std::cout << "vehicle transform topic: " << vehicle_transform_topic << std::endl;
  else {
    std::cout << "/vehicle_transform_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/vehicle_1_initial_guess_path", vehicle_initial_guess_path))
    std::cout << "vehicle_initial_guess_path pc topic: " << vehicle_initial_guess_path << std::endl;
  else {
    std::cout << "/vehicle_initial_guess_path parameter not set" << std::endl;
    return -1;
  }
  return 0;
}


void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg) {

  static int pc_counter = 1;
  log_time_ndt << pc_counter  << ",";

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input_cloud = msg->makeShared();
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  auto t1 = ros::WallTime::now();
  voxelgrid_pc.setInputCloud(raw_input_cloud);
  voxelgrid_pc.filter(*downsampled_cloud);
  auto t2 = ros::WallTime::now();
  auto duration_downsample = (t2 - t1).toSec() * 1000;
  log_time_ndt << duration_downsample<< ",";
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
  aligned = NDT_Align(ndt_cuda, downsampled_cloud);
  Publish_Transform(&transform, pc_counter++, Get_Transform()); 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NDT_Align(fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ>&  registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    auto t2 = ros::WallTime::now();
    registration.setInputSource(source_cloud);
    registration.align(*aligned,init_guess);
    auto t3 = ros::WallTime::now();
    auto duration_ndt = (t3 - t2).toSec() * 1000;
  
    Eigen::Matrix4f current_frame_pose = registration.getFinalTransformation();
    ndt_poses << current_frame_pose << std::endl;
    log_time_ndt << registration.getFitnessScore() << "," ; 
    log_time_ndt  << duration_ndt<< std::endl;

    Update_Init_Guess(current_frame_pose);
    Update_Transform(current_frame_pose);
    return aligned;
}

bool Read_Point_Cloud(const std::string path_to_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::cout << "Reading: " << path_to_file << std::endl << std::flush;
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_to_file, *cloud) == -1)  //* load the file
    return false;
  else
    return true;
}

void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string pc_topic, ros::Publisher *pub_ptr, int queue_size) {

  *pub_ptr = nh_ptr->advertise<sensor_msgs::PointCloud2>(pc_topic, queue_size);
  std::cout << "Advertised: " << pc_topic << " with queue size " << queue_size  << std::endl;
  return;
}

void Advertise_Transform(ros::NodeHandle *nh_ptr, std::string transform_topic, ros::Publisher *pub_ptr, int queue_size) {

  *pub_ptr = nh_ptr->advertise<fast_gicp::Transform>(transform_topic, queue_size);
  std::cout << "Advertised: " << transform_topic << " with queue size " << queue_size  << std::endl;
  return;
}

void Publish_Transform(ros::Publisher *pc_pub, int frame_number,  fast_gicp::Transform tf) {
  tf.header.frame_id = std::to_string(frame_number);
  tf.header.stamp = ros::Time::now();
  pc_pub->publish(tf);
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

  for (int i =0; i<4 ;++i)
  {
    getline (pose_file, line);
    std::vector<float> split_line = splitString(line, ' ');
    pose.insert(std::end(pose), std::begin(split_line), std::end(split_line));
  }

  pose_matrix << pose[0], pose[1], pose[2], pose[3],
                pose[4], pose[5], pose[6], pose[7],
                pose[8], pose[9], pose[10], pose[11],
                pose[12], pose[13], pose[14], pose[15]; 
  //init_guess = Eigen::Matrix4f::Identity();

  std::cout << pose_matrix << std::endl;
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