#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <pcl/registration/icp.h>
#include <fast_gicp/Transform.h>
#include <fast_gicp/CompressDiff.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <unistd.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/extract_indices.h>

bool received_topic1 = false;
bool received_topic2 = false;
bool received_topic3 = false;
bool received_topic4 = false;
auto duration_network=0;

std::string vehicle_1_pc_topic = "";
std::string vehicle_2_pc_topic = "";
std::string vehicle_1_transform_topic = "";
std::string vehicle_2_transform_topic = "";

Eigen::Matrix4f vehicle_1_pose;
Eigen::Matrix4f vehicle_2_pose;

ros::Subscriber vehicle_1_pc_sub, vehicle_2_pc_sub, vehicle_1_transform_sub,vehicle_2_transform_sub;

const std::string NODE_NAME = "vehicle_fusion";
const int LIDAR_FREQUENCY = 10;
const int QUEUE_SIZE_PUB = 1;
const int QUEUE_SIZE_SUB = 1;

fast_gicp::Transform tf1,tf2;
std::mutex m_reconstructed, m_vehicle_1,m_vehicle_2, m_vehicle_1_pose,m_vehicle_2_pose;
std::ofstream log_file_vehicle_1, log_file_vehicle_1_transform,log_file_vehicle_2, log_file_vehicle_2_transform, EtE_with_icp, EtE_without_icp,log_file_network_icp_latency;
pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_vehicle_1, voxelgrid_vehicle_2;

pcl::PointCloud<pcl::PointXYZ>::Ptr vehicle_1_cloud (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr d_vehicle_1_cloud (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr vehicle_2_cloud (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr d_vehicle_2_cloud (new pcl::PointCloud<pcl::PointXYZ>());


// Function prototypes
int Sanity_Check(ros::NodeHandle nh);
void callback_transform_1( const fast_gicp::TransformConstPtr& vehicle_transform);
void callback_transform_2( const fast_gicp::TransformConstPtr& vehicle_transform);
void callback_vehicle_1(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> vehicle_pc);
void callback_vehicle_2(const fast_gicp::CompressDiffConstPtr& diff_compressed);
void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);
bool Subscribe_To_PC_Topic(ros::NodeHandle *nh, const std::string pc_topic);
void performOperationIfAllReceived();
Eigen::Matrix4f Get_Transform_as_matrix(const fast_gicp::TransformConstPtr& tf);
std::vector<float> splitString(std::string str, char splitter);

int main(int argc, char **argv) {

   int append = atoi(argv[1]);

  if (append ==1 ){
    log_file_vehicle_1.open("/workspace/log_file_vehicle_1_pcd.csv",std::ios::app);
    log_file_vehicle_2.open("/workspace/log_file_vehicle_2_pcd.csv",std::ios::app);
    log_file_vehicle_1_transform.open("/workspace/log_file_vehicle_1_transform.csv",std::ios::app);
    log_file_vehicle_2_transform.open("/workspace/log_file_vehicle_2_transform.csv",std::ios::app);
    EtE_with_icp.open("/workspace/EtE_with_icp.txt", std::ios::app);
    EtE_without_icp.open("/workspace/EtE_without_icp.txt", std::ios::app);
  }

  else {
    log_file_vehicle_1.open("/workspace/log_file_vehicle_1_pcd.csv");
    log_file_vehicle_2.open("/workspace/log_file_vehicle_2_pcd.csv");
    log_file_network_icp_latency.open("/workspace/End2end_latency.csv");
    log_file_vehicle_1_transform.open("/workspace/log_file_vehicle_1_transform.csv");
    log_file_vehicle_2_transform.open("/workspace/log_file_vehicle_2_transform.csv");
    EtE_with_icp.open("/workspace/EtE_with_icp.txt");
    EtE_without_icp.open("/workspace/EtE_without_icp.txt");
    log_file_vehicle_1 << "frame_number" << "," << "latency_downsample_vehicle" <<std::endl;
    log_file_vehicle_2 << "frame_number" << "," << "latency_downsample_vehicle" << "," << "latency_network"<<"," << "latency_decompression"<<std::endl;
    log_file_vehicle_1_transform << "frame_number" << "," <<"transformation"  << ","<<std::endl;
    log_file_vehicle_2_transform << "frame_number" << "," <<"transformation"<<"," <<"latency_network" <<std::endl;
    log_file_network_icp_latency<<"frame_number" <<"," <<"latency_icp" << "," << " latency_fusion" <<std::endl;
  }

  std::cout << "Initializing " << NODE_NAME << std::endl;
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;


  if(Sanity_Check(nh) == -1) {
    std::cout << "Exitting early" << std::endl;
    return -1;
  }

  fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp_cuda;
  vgicp_cuda.setResolution(1);
  vgicp_cuda.setMaximumIterations(100);
  vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
  vgicp_cuda.setKernelWidth(0.3);
  voxelgrid_vehicle_1.setLeafSize(0.5f, 0.5f, 0.5f);
  voxelgrid_vehicle_2.setLeafSize(0.5f, 0.5f, 0.5f);;

  std::cout << "Subscribing to point cloud topic: " << vehicle_1_pc_topic << std::endl;
  std::cout << "Subscribing to point cloud topic: " << vehicle_2_pc_topic << std::endl;

 vehicle_1_pc_sub = nh.subscribe(vehicle_1_pc_topic, QUEUE_SIZE_SUB,callback_vehicle_1);
 vehicle_2_pc_sub = nh.subscribe(vehicle_2_pc_topic, QUEUE_SIZE_SUB,callback_vehicle_2);
 vehicle_1_transform_sub = nh.subscribe(vehicle_1_transform_topic, QUEUE_SIZE_SUB,callback_transform_1);
 vehicle_2_transform_sub = nh.subscribe(vehicle_2_transform_topic, QUEUE_SIZE_SUB,callback_transform_2);

 
  std::cout << "Going into ros::spin() ..." << std::endl;
  ros::spin();

  return 0;
}

int Sanity_Check(ros::NodeHandle n_h) {

  if(n_h.getParam("/vehicle_1_pc_topic", vehicle_1_pc_topic))
    std::cout << "vehicle_1_pc_topic: " << vehicle_1_pc_topic << std::endl;
  else {
    std::cout << "/vehicle_1_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/vehicle_1_transform_topic", vehicle_1_transform_topic))
    std::cout << "vehicle 1 transform topic: " << vehicle_1_transform_topic << std::endl;
  else {
    std::cout << "/vehicle_1_transform_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/vehicle_2_pc_topic", vehicle_2_pc_topic))
    std::cout << "vehicle_2_pc_topic: " << vehicle_2_pc_topic << std::endl;
  else {
    std::cout << "/vehicle_2_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/vehicle_2_transform_topic", vehicle_2_transform_topic))
    std::cout << "vehicle 2 transform topic: " << vehicle_2_transform_topic << std::endl;
  else {
    std::cout << "/vehicle_2_transform_topic parameter not set" << std::endl;
    return -1;
  }

  return 0;
}

void callback_transform_1(const fast_gicp::TransformConstPtr& vehicle_transform) 
{
  std::cout << "T1" << std::endl;
  auto t1 = ros::WallTime::now();
  m_vehicle_1_pose.lock();
  vehicle_1_pose =  Get_Transform_as_matrix(vehicle_transform); 
  m_vehicle_1_pose.unlock();
  auto t2 = ros::WallTime::now();
  received_topic1 = true;
  performOperationIfAllReceived();
  auto duration_vehicle_transform = (t2 - t1).toSec() * 1000;
  log_file_vehicle_1_transform << vehicle_transform->header.frame_id  << ","<< duration_vehicle_transform << std::endl;
}

void callback_vehicle_1(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> vehicle_pc) 
{
  std::cout << "V1" << std::endl;
  auto t1 = ros::WallTime::now();
  static int pc_counter =1;
  vehicle_1_cloud = vehicle_pc->makeShared();
  voxelgrid_vehicle_1.setInputCloud(vehicle_1_cloud);
  m_vehicle_1.lock();
  voxelgrid_vehicle_1.filter(*d_vehicle_1_cloud);
  m_vehicle_1.unlock();
  auto t2 = ros::WallTime::now();
  received_topic2 = true;
  performOperationIfAllReceived(); 
  auto duration_downsample_vehicle = (t2 - t1).toSec() * 1000; 
  log_file_vehicle_1 << pc_counter << "," << duration_downsample_vehicle << std::endl;
  pc_counter++;
}
void callback_transform_2( const fast_gicp::TransformConstPtr& vehicle_transform) 
{
  std::cout << "T2" << std::endl;
  auto frame_time = vehicle_transform->header.stamp;
  ros::Time t = ros::Time::now(); 
  auto t1 = ros::WallTime::now();
  m_vehicle_2_pose.lock();
  vehicle_2_pose =  Get_Transform_as_matrix(vehicle_transform); 
  m_vehicle_2_pose.unlock();
  auto t2 = ros::WallTime::now();
  received_topic3 = true;
  performOperationIfAllReceived();
  auto duration_network = (t.toSec() - frame_time.toSec()) * 1000;
  auto duration_vehicle_transform = (t2 - t1).toSec() * 1000;
  log_file_vehicle_2_transform << vehicle_transform->header.frame_id  << ","<< duration_vehicle_transform<< ","<< duration_network << std::endl;
}
void callback_vehicle_2(const fast_gicp::CompressDiffConstPtr& diff_compressed) 
{
  std::cout << "V2" << std::endl;
  ros::Time t = ros::Time::now(); 
  auto frame_time = diff_compressed->header.stamp;
  auto duration_network = (t.toSec() - frame_time.toSec()) * 1000;
  std::string s;
  std::stringstream compressedData;
  s =  diff_compressed->data;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression (pcl::io::MANUAL_CONFIGURATION,false, 0.001, 0.1, false, 1, false, 8);
  auto t0 = ros::WallTime::now();
  // if (s.size() != 0)
  // {
    compressedData.write(s.data(), s.size());
  	octree_compression.decodePointCloud(compressedData, vehicle_2_cloud);
  // }
  auto t1 = ros::WallTime::now();
  static int pc_counter =1;
  // auto t2 = ros::WallTime::now();
  // voxelgrid_vehicle_2.setInputCloud(vehicle_2_cloud);
  // m_vehicle_2.lock();
  // voxelgrid_vehicle_2.filter(*d_vehicle_2_cloud);
  // m_vehicle_2.unlock();
  // auto t3 = ros::WallTime::now();
  received_topic4 = true; 
  performOperationIfAllReceived();
  auto duration_decompression_vehicle = (t1 - t0).toSec() * 1000; 
  // auto duration_downsample_vehicle = (t3 - t2).toSec() * 1000; 
  log_file_vehicle_2 << pc_counter << "," << "0" <<"," << duration_network<< "," << duration_decompression_vehicle<<std::endl;
  pc_counter++;
}


void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string pc_topic, ros::Publisher *pub_ptr, int queue_size) {
  *pub_ptr = nh_ptr->advertise<sensor_msgs::PointCloud2>(pc_topic, queue_size);
  std::cout << "Advertised: " << pc_topic << " with queue size " << queue_size  << std::endl;
  return;
}

Eigen::Matrix4f Get_Transform_as_matrix(const fast_gicp::TransformConstPtr& tf) {
  Eigen::Matrix4f transform;
  transform <<      tf->data[0],  tf->data[1], tf->data[2],  tf->data[3], 
                    tf->data[4],  tf->data[5], tf->data[6],  tf->data[7], 
                    tf->data[8],  tf->data[9], tf->data[10],  tf->data[11], 
                    0.,          0.,          0.,          1.  ;
  return transform;
}

void performOperationIfAllReceived() {
    if (received_topic1 && received_topic2 && received_topic3 && received_topic4) {
       static int pc_counter =1;
fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp_cuda;
  vgicp_cuda.setResolution(1);
  vgicp_cuda.setMaximumIterations(100);
  vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
  vgicp_cuda.setKernelWidth(0.3);
  auto t4 = ros::WallTime::now(); 
  m_vehicle_1_pose.lock();
  Eigen::Matrix4f vehicle_pose_inv = vehicle_1_pose.inverse();
  m_vehicle_1_pose.unlock();
  Eigen::Matrix4f Final_pose = vehicle_pose_inv * vehicle_2_pose;
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
  m_vehicle_2.lock();
  vgicp_cuda.setInputTarget(d_vehicle_1_cloud);
  vgicp_cuda.setInputSource(vehicle_2_cloud);
  vgicp_cuda.align(*aligned,Final_pose);
  m_vehicle_2.unlock();
  auto t5 = ros::WallTime::now(); 
  auto duration_icp= (t5 - t4).toSec() * 1000;
  auto t6  = ros::WallTime::now();
  pcl::transformPointCloud(*vehicle_2_cloud, *aligned, vgicp_cuda.getFinalTransformation());
  *aligned += *vehicle_1_cloud ;
  auto t7= ros::WallTime::now();
  auto duration_fusion= (t7 - t6).toSec() * 1000;
  log_file_network_icp_latency<< "sensor_frame" << ","<< duration_icp << "," << duration_fusion << std::endl;
  EtE_with_icp << vgicp_cuda.getFinalTransformation() <<std::endl;
  EtE_without_icp << Final_pose << std::endl;
  pc_counter++;
        received_topic1 = false;
        received_topic2 = false;
        received_topic3 = false;
        received_topic4 = false;
    }
}

