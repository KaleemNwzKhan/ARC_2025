#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sstream>
#include <string.h>
#include <fast_gicp/CompressDiff.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/compression/octree_pointcloud_compression.h>


//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <chrono>

// ///////////////// Goal \\\\\\\\\\\\\\\\\\\\\\\
// perform diff on infrastructure point clouds agains the initial static point cloud
// Inputs: Point cloud topic, initial static infra point cloud
// Outputs: diff point cloud, diff compressed

// variables to get from rospram
std::string infra_static_pc_path = "";
std::string infra_pc_topic = "";
std::string diff_pc_topic = "";
std::string diff_compress_topic = "";


// publishers and subscribers
ros::Subscriber pc_sub_compress, pc_sub_diff, pc_sub_raw;
ros::Publisher reconstructed_pub;

// global variavbles
const std::string NODE_NAME = "decode";
const int LIDAR_FREQUENCY = 10;
float resolution = 0.9;
double frame_time, current_time;

// global objects 
std::ofstream log_file_compress, log_file_diff, log_file_raw,log_file_total;
pcl::PointCloud<pcl::PointXYZ>::Ptr infra_static_pc(new pcl::PointCloud<pcl::PointXYZ>());

fast_gicp::CompressDiff compressed_diff;


// Function prototypes
int Sanity_Check(ros::NodeHandle nh);
bool Read_Point_Cloud(const std::string path_to_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void Point_Cloud_Callback_compress(const fast_gicp::CompressDiffConstPtr& diff_compressed);
void Point_Cloud_Callback_raw(const sensor_msgs::PointCloud2ConstPtr& msg);
void Point_Cloud_Callback_diff(const sensor_msgs::PointCloud2ConstPtr& msg);


bool Subscribe_To_PC_Topic(ros::NodeHandle *nh, const std::string pc_topic);

void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);
void Advertise_String_Topic(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);

void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
void Publish_String(ros::Publisher *pc_pub, int frame_number,fast_gicp::CompressDiff data);


int main(int argc, char **argv) {

  ///// 1. Initialization Code /////
  std::cout.precision(17);
  
  //log_file_compress.open("/workspace/log_time_network_compress.csv");
  //log_file_diff.open("/workspace/log_time_network_diff.csv");
  //log_file_raw.open("/workspace/log_time_network_raw.csv");
  //log_file_compress << "frame_number" << ","  << "frame_time_stamp" << "," << "current_time" <<  "," << "diffrence (ms)" << std::endl;
  //log_file_diff << "frame_number" << ","  << "frame_time_stamp" << "," << "current_time" <<  "," << "diffrence(ms)" << std::endl;
  //log_file_raw << "frame_number" << ","  << "frame_time_stamp" << "," << "current_time" << "," <<"diffrence(ms)" << std::endl;
  
  
  log_file_total.open("/workspace/log_time_all.csv");
  log_file_total << "frame_number" << ","  << "network_time " << "," << "decompression_time" << "," <<"reconstruction_time(ms)" << "," << "total time"<< std::endl;
  
  
  // 1.1 Initialize ROS
  std::cout << "Initializing " << NODE_NAME << std::endl;
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  // 1.2 Argument sanity checks
  if(Sanity_Check(nh) == -1) {
    std::cout << "Exitting early" << std::endl;
    return -1;
  }


  
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
  Advertise_PC_Topic(&nh, "autopass/reconstructed", &reconstructed_pub, LIDAR_FREQUENCY);
  //Advertise_String_Topic(&nh, diff_compress_topic, &diff_compress_pub, LIDAR_FREQUENCY);


  //// 2. Main Loop /////
  std::cout << "Subscribing to point cloud topic: " << infra_pc_topic << std::endl;
  pc_sub_compress = nh.subscribe(diff_compress_topic, 10000, Point_Cloud_Callback_compress);
  //pc_sub_diff = nh.subscribe(diff_pc_topic, 1, Point_Cloud_Callback_diff);
  //pc_sub_raw = nh.subscribe(infra_pc_topic, 1, Point_Cloud_Callback_raw);
  std::cout << "Going into ros::spin() ..." << std::endl;
  ros::spin();

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

  if(n_h.getParam("/infra_pc_topic_2", infra_pc_topic))
    std::cout << "infra_pc_topic: " << infra_pc_topic << std::endl;
  else {
    std::cout << "/vehicle_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/diff_pc_topic", diff_pc_topic))
    std::cout << "Diff point cloud topic: " << diff_pc_topic << std::endl;
  else {
    std::cout << "/diff_pc_topic parameter not set" << std::endl;
    return -1;
  }

  if(n_h.getParam("/diff_compress_topic", diff_compress_topic))
    std::cout << "diff compress topic: " << diff_compress_topic << std::endl;
  else {
    std::cout << "/diff_compress_topic parameter not set" << std::endl;
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

// void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg) {



//   static int pc_counter = 0;
//   std::cout << "******************************" << std::endl;
//   std::cout << "---------- Point cloud: " << pc_counter << " -----------" << std::endl;


//   pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input_cloud = msg->makeShared();
//   pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructed (new pcl::PointCloud<pcl::PointXYZ>());

//   std::stringstream compressedData;

//     *reconstructed = *infra_static_pc + *raw_input_cloud;


//   pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression1 (pcl::io::MANUAL_CONFIGURATION,false, 0.001, 0.1, false, 1, false, 8);

//   //compressed_diff.data = compressedData.str();
//   // Publish diff point cloud
//   Publish_Cloud(&reconstructed_pub, pc_counter++, reconstructed);
//   //Publish_String(&diff_compress_pub, 1, compressed_diff);
// //   Publish_Transform(&transform, 1, Get_Transform());
// }

void Point_Cloud_Callback_compress(const fast_gicp::CompressDiffConstPtr& diff_compressed) {

  static int pc_counter = 0;
  auto t1 = ros::WallTime::now();
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructed (new pcl::PointCloud<pcl::PointXYZ>());

  std::string s;
  std::stringstream compressedData;
  s =  diff_compressed->data;
  if (s.size() != 0)
  
  {
    compressedData.write(s.data(), s.size());
  	//compressedData << diff_compressed->data.c_str();
 		pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression1 (pcl::io::MANUAL_CONFIGURATION,false, 0.001, 0.1, false, 1, false, 8);
  	octree_compression1.decodePointCloud(compressedData, reconstructed);
  }

  
  auto t2 = ros::WallTime::now();
  
  *reconstructed += *infra_static_pc;
  
  auto t3 = ros::WallTime::now();
  
  auto frame_time = diff_compressed->header.stamp;
  auto duration_network = t1.toSec() - frame_time.toSec();
  auto duration_decompression = (t2 - t1).toSec();
  auto duration_reconstruction = (t3 - t2).toSec();
  auto duration_total  = t3.toSec() - frame_time.toSec();
  //auto current_time = ros::WallTime::now();
  //auto current_time_s = current_time.toSec ();
	//auto converted_current_time = ros::Time::fromWallTime (current_time);

  //std::cout << frame_time << std::endl;
  //auto frame_time_s = frame_time.toSec();
  
  log_file_total << pc_counter << ","<< duration_network*1000 << "," << duration_decompression*1000 << "," <<  duration_reconstruction* 1000 << "," << duration_total*1000 <<std::endl; 
  

	
	//std::cout <<"frame time: "  << std::fixed << frame_time.toSec() << std::endl;
	//std::cout << "curnt time: "  << std::fixed << current_time_s << std::endl;
	//std::cout << "frame time: "  << std::fixed << frame_time_s << std::endl;
	//std::cout << "difference: "  << std::fixed << ( current_time_s - frame_time_s ) * 1000 << " ms"  << std::endl;
	
	//log_file_compress << pc_counter << "," 	<< frame_time_s << "," << current_time_s << "," <<  ( current_time_s - frame_time_s ) * 1000 <<std::endl; 
	
  //std::cout << "******************************" << std::endl;
  //std::cout << "---------- Point cloud: " << pc_counter << " -----------" << std::endl;



  //pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input_cloud = msg->makeShared();
  
  //auto current_time = ros::WallTime::now();
  //auto frame_time = diff_compressed->header.stamp;

  //

	std::cout<< pc_counter <<std::endl;
	pc_counter++;
  //compressed_diff.data = compressedData.str();
  // Publish diff point cloud
  //Publish_Cloud(&reconstructed_pub, pc_counter++, reconstructed);
  //Publish_String(&diff_compress_pub, 1, compressed_diff);
//   Publish_Transform(&transform, 1, Get_Transform());
}

void Point_Cloud_Callback_raw	(const sensor_msgs::PointCloud2ConstPtr& msg) {

  auto current_time = ros::WallTime::now();
  auto current_time_s = current_time.toSec ();
	//auto converted_current_time = ros::Time::fromWallTime (current_time);
  auto frame_time = msg->header.stamp;
  //std::cout << frame_time << std::endl;
  auto frame_time_s = frame_time.toSec();
  
  static int pc_counter = 0;
  

	
	//std::cout <<"frame time: "  << std::fixed << frame_time.toSec() << std::endl;
	std::cout << "curnt time: "  << std::fixed << current_time_s << std::endl;
	std::cout << "frame time: "  << std::fixed << frame_time_s << std::endl;
	std::cout << "difference: "  << std::fixed << ( current_time_s - frame_time_s ) * 1000 << " ms"  << std::endl;
	
	log_file_raw << pc_counter << "," 	<< frame_time_s << "," << current_time_s << "," <<  ( current_time_s - frame_time_s ) * 1000 <<std::endl; 
	
  std::cout << "******************************" << std::endl;
  std::cout << "---------- Point cloud: " << pc_counter << " -----------" << std::endl;


	pc_counter++;

}

void Point_Cloud_Callback_diff(const sensor_msgs::PointCloud2ConstPtr& msg) {

  auto current_time = ros::WallTime::now();
  auto current_time_s = current_time.toSec ();
//auto converted_current_time = ros::Time::fromWallTime (current_time);
  auto frame_time = msg->header.stamp;
  //std::cout << frame_time << std::endl;
  auto frame_time_s = frame_time.toSec();
  
  static int pc_counter = 0;
  

	
	//std::cout <<"frame time: "  << std::fixed << frame_time.toSec() << std::endl;
	std::cout << "curnt time: "  << std::fixed << current_time_s << std::endl;
	std::cout << "frame time: "  << std::fixed << frame_time_s << std::endl;
	std::cout << "difference: "  << std::fixed << ( current_time_s - frame_time_s ) * 1000 << " ms"  << std::endl;
	
	log_file_diff << pc_counter << "," 	<< frame_time_s << "," << current_time_s << "," <<  ( current_time_s - frame_time_s ) * 1000 <<std::endl; 
	
  std::cout << "******************************" << std::endl;
  std::cout << "---------- Point cloud: " << pc_counter << " -----------" << std::endl;


	pc_counter++;

}


void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string pc_topic, ros::Publisher *pub_ptr, int queue_size) {

  // Advertise topics
  *pub_ptr = nh_ptr->advertise<sensor_msgs::PointCloud2>(pc_topic, queue_size);
  std::cout << "Advertised: " << pc_topic << " with queue size " << queue_size  << std::endl;
  return;
}

void Advertise_String_Topic(ros::NodeHandle *nh_ptr, std::string topic, ros::Publisher *pub_ptr, int queue_size) {

  // Advertise topics
  *pub_ptr = nh_ptr->advertise<fast_gicp::CompressDiff>(topic, queue_size);
  std::cout << "Advertised: " << topic << " with queue size " << queue_size  << std::endl;
  return;
}


void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  input_cloud->header.frame_id = "os_sensor";
  pcl::PCLPointCloud2::Ptr input_cloud_pc2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*input_cloud, *input_cloud_pc2);
  pc_pub->publish(*input_cloud_pc2);
}

void Publish_String(ros::Publisher *pc_pub, int frame_number, fast_gicp::CompressDiff data) {
data.header.frame_id = "os_sensor";
data.header.stamp = ros::Time::now();
pc_pub->publish(data);
}

//////////////////////////////////////////////////////////////////////////////
