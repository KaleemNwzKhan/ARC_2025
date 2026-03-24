#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <fast_gicp/Transform.h>
#include <fast_gicp/CompressDiff.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <pcl/filters/voxel_grid.h>


//#include <message_filters/time_synchronizer.h>
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/registration/ndt.h>
//#include <pcl/registration/gicp.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <chrono>
//#include <pcl/common/transforms.h>
//#include <pcl/filters/approximate_voxel_grid.h>
//#include <fast_gicp/gicp/fast_gicp.hpp>
//#include <fast_gicp/gicp/fast_gicp_st.hpp>
//#include <fast_gicp/gicp/fast_vgicp.hpp>
//

// ///////////////// Goal \\\\\\\\\\\\\\\\\\\\\\\
// Align point clouds with a map using NDT
// Inputs: Point cloud topic, map file
// Outputs: Aligned point clouds, map point cloud

// variables to get from rospram 
std::string infra_static_pc_path = "";
std::string diff_compress_topic = "";
std::string diff_pc_topic = "";
std::string vehicle_pc_topic = "";
std::string vehicle_transform_topic = "";
std::string infra_reconstructed_topic = "";
std::string fused_pc_topic = "";
std::string infra_initial_guess_path ="";

// subscribers and publishers
ros::Subscriber vehicle_pc_sub, diff_compress_sub, diff_pc_sub, vehicle_transform_sub;
ros::Publisher infra_reconstructed_pub, fused_pc_pub;


// global variables
const std::string NODE_NAME = "vehicle_fusion";
const int LIDAR_FREQUENCY = 10;
const int QUEUE_SIZE_PUB = 10;
const int QUEUE_SIZE_SUB = 1;
Eigen::Matrix4f lidar_pose;
Eigen::Matrix4f vehicle_pose;


fast_gicp::Transform tf;


// global objects
pcl::PointCloud<pcl::PointXYZ>::Ptr infra_static_pc(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr d_infra_static_pc(new pcl::PointCloud<pcl::PointXYZ>());

pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp_cuda;




// Function prototypes
int Sanity_Check(ros::NodeHandle nh);
bool Read_Point_Cloud(const std::string path_to_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void callback(const sensor_msgs::PointCloud2ConstPtr& msg, const fast_gicp::CompressDiffConstPtr& diff, const fast_gicp::TransformConstPtr& vehicle_transform);

void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);
void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

Eigen::Matrix4f Get_Transform_as_matrix(const fast_gicp::TransformConstPtr& tf);
Eigen::Matrix4f Initialize_Init_Guess(std::string initial_guess_path);
std::vector<float> splitString(std::string str, char splitter);

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

   // 1.3 Read infra static point cloud
  std::cout << "Reading infra static point cloud file ..." << std::endl;
  

  if(Read_Point_Cloud(infra_static_pc_path, infra_static_pc)) 
  {
    std::cout << "Successfully read infra static point cloud from file" << std::endl;
  } 
  else 
  {
    std::cout << "Failed to read infra static point cloud from file" << std::endl;
    return -1;
  }


  // 1.4 Advertise point cloud topics for aligned points and the map
  std::cout << "Advertising point cloud topics for aligned points and the map" << std::endl;
  Advertise_PC_Topic(&nh, infra_reconstructed_topic, &infra_reconstructed_pub, QUEUE_SIZE_PUB);
  Advertise_PC_Topic(&nh, fused_pc_topic, &fused_pc_pub, QUEUE_SIZE_PUB);


  // 1.5 Initialize initial guess
  lidar_pose = Initialize_Init_Guess(infra_initial_guess_path);

  voxelgrid.setLeafSize(1.0f, 1.0f, 1.0f);
  voxelgrid.setInputCloud(infra_static_pc);
  voxelgrid.filter(*d_infra_static_pc);


  vgicp_cuda.setResolution(1.0);
  vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
  vgicp_cuda.setKernelWidth(0.5);
  vgicp_cuda.setMaximumIterations(100);



  //// 2. Main Loop /////
  std::cout << "Subscribing to point cloud topic: "  << std::endl;
  message_filters::Subscriber<sensor_msgs::PointCloud2> vehicle_pc_sub(nh, vehicle_pc_topic, 1);
  message_filters::Subscriber<fast_gicp::CompressDiff> diff_compress_sub(nh, diff_compress_topic, 1);
  message_filters::Subscriber<fast_gicp::Transform> vehicle_transform_sub(nh, vehicle_transform_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, fast_gicp::CompressDiff, fast_gicp::Transform> MySyncPolicy;


  //message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, fast_gicp::Transform> sync(vehicle_pc_sub, diff_pc_sub, vehicle_transform_sub, 10);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1),vehicle_pc_sub, diff_compress_sub, vehicle_transform_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));


  std::cout << "Going into ros::spin() ..." << std::endl;
  ros::spin();
  //loop_rate.sleep();

  return 0;
}

int Sanity_Check(ros::NodeHandle n_h) {
  if(n_h.getParam("/infra_static_pc_path", infra_static_pc_path))
    std::cout << "infra static pc path : " << infra_static_pc_path << std::endl;
  else {
    std::cout << "infra static pc path : " << infra_static_pc_path << std::endl;
    std::cout << "/infra_static_pc_path parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/diff_compress_topic", diff_compress_topic))
    std::cout << "diff compress topic: " << diff_compress_topic << std::endl;
  else {
    std::cout << "/diff_compress_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/vehicle_pc_topic", vehicle_pc_topic))
    std::cout << "vehicle_pc_topic: " << vehicle_pc_topic << std::endl;
  else {
    std::cout << "/vehicle_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/vehicle_transform_topic", vehicle_transform_topic))
    std::cout << "vehicle transform topic: " << vehicle_transform_topic << std::endl;
  else {
    std::cout << "/vehicle_transform_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/infra_reconstructed_topic", infra_reconstructed_topic))
    std::cout << "Infra reconstructed topic: " << infra_reconstructed_topic << std::endl;
  else {
    std::cout << "/infra_reconstructed_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/fused_pc_topic", fused_pc_topic))
    std::cout << "fused pc topic: " << fused_pc_topic << std::endl;
  else {
    std::cout << "/fused_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/diff_pc_topic", diff_pc_topic))
    std::cout << "diff pc topic: " << diff_pc_topic << std::endl;
  else {
    std::cout << "/diff_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/infra_initial_guess_path", infra_initial_guess_path))
    std::cout << "infra_initial_guess_path pc topic: " << infra_initial_guess_path << std::endl;
  else {
    std::cout << "/infra_initial_guess_path parameter not set" << std::endl;
    return -1;
  }

  // All good otherwise
  return 0;
}

bool Read_Point_Cloud(const std::string path_to_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::cout << "Reading: " << path_to_file << std::endl << std::flush;
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_to_file, *cloud) == -1)  //* load the file
    return false;
  else
    return true;
}

void callback(const sensor_msgs::PointCloud2ConstPtr& msg, const fast_gicp::CompressDiffConstPtr& diff, const fast_gicp::TransformConstPtr& vehicle_transform)
{

    auto t1 = ros::WallTime::now();

    static int pc_counter = 0;
   //std::cout << "******************************" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr infra_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr d_infra_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr vehicle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr d_vehicle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);

    auto t2  = ros::WallTime::now();

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*vehicle_cloud);

    auto t3  = ros::WallTime::now();




    // RECONSTRUCTING INFRA FRAME

    std::string s;
    std::stringstream compressedData;
    s =  diff->data;
    compressedData.write(s.data(), s.size());
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression1 (pcl::io::MANUAL_CONFIGURATION,false, 0.001, 0.1, false, 1, false, 8);
    octree_compression1.decodePointCloud(compressedData, infra_cloud);

    auto t4  = ros::WallTime::now();
    //*d_infra_cloud = *d_infra_static_pc + *infra_cloud ;
    *infra_cloud += *infra_static_pc;

    auto t5  = ros::WallTime::now();

    // ICP 

    Eigen::Matrix4f inital_guess_icp =  Get_Transform_as_matrix(vehicle_transform).inverse() * lidar_pose ;

    auto t6  = ros::WallTime::now();

    voxelgrid.setInputCloud(infra_cloud);
    voxelgrid.filter(*d_infra_cloud);
    
    voxelgrid.setInputCloud(vehicle_cloud);
    voxelgrid.filter(*d_vehicle_cloud);

    auto t7  = ros::WallTime::now();

    //double fitness_score = 0.0;
    vgicp_cuda.setInputTarget(d_vehicle_cloud);
    vgicp_cuda.setInputSource(d_infra_cloud);
    vgicp_cuda.align(*aligned,inital_guess_icp);
    //fitness_score = vgicp_cuda.getFitnessScore();

    auto t8  = ros::WallTime::now();

    pcl::transformPointCloud(*infra_cloud, *infra_cloud, vgicp_cuda.getFinalTransformation());

    auto t9  = ros::WallTime::now();


    //*vehicle_cloud += *infra_cloud;


    auto t10 = ros::WallTime::now();

    auto duration_local_variables = (t2 - t1).toSec();
    auto duration_pc2_to_pc = (t3 - t2).toSec();
    auto duration_decompression = (t4 - t3).toSec();
    auto duration_reconstruction = (t5 - t4).toSec();
    auto duration_inital_guess = (t6 - t5).toSec();
    auto duration_downsampling = (t7 - t6).toSec();
    auto duration_ICP = (t8 - t7).toSec();
    auto duration_transform_infra = (t9 - t8).toSec();
    auto duration_fusion = (t10 - t9).toSec();
    auto duration_total = (t10 - t1).toSec();



    std::cout << "duration_local_variables :" << duration_local_variables * 1000 << ": msec" << std::endl;
    std::cout << "duration_pc2_to_pc :" << duration_pc2_to_pc * 1000 << ": msec" << std::endl;
    std::cout << "duration_decompression :" << duration_decompression * 1000 << ": msec" << std::endl;
    std::cout << "duration_reconstruction :" << duration_reconstruction * 1000 << ": msec" << std::endl;
    std::cout << "duration_inital_guess :" << duration_inital_guess * 1000 << ": msec" << std::endl;
    std::cout << "duration_downsampling :" << duration_downsampling * 1000 << ": msec" << std::endl;
    std::cout << "duration_ICP :" << duration_ICP * 1000 << ": msec" << std::endl;
    std::cout << "duration_transform_infra :" << duration_transform_infra * 1000 << ": msec" << std::endl;
    std::cout << "duration_fusion :" << duration_fusion * 1000 << ": msec" << std::endl;



    std::cout << "duration_total :" << duration_total * 1000 << ": msec" << std::endl;


    //Publish_Cloud(&fused_pc_pub, pc_counter++, infra_cloud);

}


void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string pc_topic, ros::Publisher *pub_ptr, int queue_size) {

  // Advertise topics
  *pub_ptr = nh_ptr->advertise<sensor_msgs::PointCloud2>(pc_topic, queue_size);
  std::cout << "Advertised: " << pc_topic << " with queue size " << queue_size  << std::endl;
  return;
}


void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  pcl::PCLPointCloud2::Ptr input_cloud_pc2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*input_cloud, *input_cloud_pc2);
  input_cloud_pc2->header.frame_id = "os_sensor";
  pcl_conversions::toPCL(ros::Time::now(), input_cloud_pc2->header.stamp);
  pc_pub->publish(*input_cloud_pc2);
}


Eigen::Matrix4f Get_Transform_as_matrix(const fast_gicp::TransformConstPtr& tf) {
  Eigen::Matrix4f transform;
  transform <<      tf->data[0],  tf->data[1], tf->data[2],  tf->data[3], 
                    tf->data[4],  tf->data[5], tf->data[6],  tf->data[7], 
                    tf->data[8],  tf->data[9], tf->data[10],  tf->data[11], 
                    0.,          0.,          0.,          1.  ;

  return transform;
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