#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sstream>
#include <string.h>
#include <vector>  

#include <fast_gicp/CompressDiff.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/compression/octree_pointcloud_compression.h>

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
ros::Subscriber pc_sub;
ros::Publisher diff_pc_pub;
ros::Publisher diff_compress_pub;

// global variables
const std::string NODE_NAME = "infra";
const int LIDAR_FREQUENCY = 10;
const int QUEUE_SIZE_PUB = 10;
const int QUEUE_SIZE_SUB = 1;
float resolution = 0.9;

// global objects 
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>::Ptr octree_change_detection;
pcl::PointCloud<pcl::PointXYZ>::Ptr infra_static_pc(new pcl::PointCloud<pcl::PointXYZ>());
fast_gicp::CompressDiff compressed_diff;
pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression (pcl::io::MANUAL_CONFIGURATION,false, 0.001, 0.1, false, 1, false, 8);
std::ofstream log_file;
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>::Ptr octree_change_detector;



// Function prototypes
int Sanity_Check(ros::NodeHandle nh);
bool Read_Point_Cloud(const std::string path_to_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_msg);
bool Subscribe_To_PC_Topic(ros::NodeHandle *nh, const std::string pc_topic);
void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);
void Advertise_String_Topic(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);
void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
void Publish_String(ros::Publisher *pc_pub, int frame_number,fast_gicp::CompressDiff data);
void initialize_octree (pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>& octree_change_add );
std::vector<int> octree_change (pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>& octree_change_add,  pcl::PointCloud<pcl::PointXYZ>::Ptr pc );
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>::Ptr Create_Change_Detector ();

int main(int argc, char **argv) {

  ///// 1. Initialization Code /////

  // Open log file
  log_file.open("/workspace/log.csv");
  // log_file << "frame_number,load_static_cloud,input_cloud_read,output_variables_init,load_dynamic_cloud,diff_time,diff_compress_time\n";


  log_file << "frame_number" << "," 
    << "change_detector_init" << "," 
    << "static_cloud_load" << "," 
    << "dynamic_cloud_load" << "," 
    << "diff_time" << "," 
    << "extraction_time" << "," 
    << "compression_time" << "," 
    << "e2e_time" << std::endl;


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

  // Load static cloud
  octree_change_detector = Create_Change_Detector();

  // 1.4 Advertise point cloud topics for aligned points and the map
  std::cout << "Advertising point cloud topics for aligned points and the map" << std::endl;
  Advertise_PC_Topic(&nh, diff_pc_topic, &diff_pc_pub, QUEUE_SIZE_PUB);
  Advertise_String_Topic(&nh, diff_compress_topic, &diff_compress_pub, QUEUE_SIZE_PUB);


  //// 2. Main Loop /////
  std::cout << "Subscribing to point cloud topic: " << infra_pc_topic << std::endl;
  pc_sub = nh.subscribe(infra_pc_topic, QUEUE_SIZE_SUB, Point_Cloud_Callback);
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

  if(n_h.getParam("/infra_pc_topic", infra_pc_topic))
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

pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>::Ptr Create_Change_Detector ()
{
  // Create octree object
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>::Ptr octree_change_detector 
  ( new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> (resolution) );
  // Load static cloud into octree
  octree_change_detector->setInputCloud (infra_static_pc);
  octree_change_detector->addPointsFromInputCloud ();
  // Switch buffers
  octree_change_detector->switchBuffers ();
  // Return octree change detector
  return octree_change_detector;
}

//void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg) {
void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg) {

  static int pc_counter = 0;
  std::cout << "******************************" << std::endl;
  std::cout << "---------- Point cloud: " << pc_counter << " -----------" << std::endl;

  /////////////////////////
  //// Change detector ////
  /////////////////////////

  auto start = ros::WallTime::now();
  // Declare octree change detector class
  auto change_detector_init = ros::WallTime::now();
  octree_change_detector = Create_Change_Detector();

  // Load static cloud
  // octree_change_detector->setInputCloud (infra_static_pc);
  // octree_change_detector->addPointsFromInputCloud ();

  // Switch the buffer so that we can load a new point cloud to compare against the reference cloud
  // octree_change_detector->switchBuffers ();
  auto static_cloud_load = ros::WallTime::now();

  // Read dynamic point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input_cloud = msg->makeShared();

  // Load the dynamic cloud
  octree_change_detector->setInputCloud (raw_input_cloud);
  octree_change_detector->addPointsFromInputCloud ();

  auto dynamic_cloud_load = ros::WallTime::now();

  // Diff operation
  std::vector<int> newPointIdxVector;
  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree_change_detector->getPointIndicesFromNewVoxels (newPointIdxVector);
  auto diff_time = ros::WallTime::now();

  // Extract the diff points
  pcl::PointCloud<pcl::PointXYZ>::Ptr diff_points (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  inliers->indices = newPointIdxVector;
  extract.setInputCloud (raw_input_cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*diff_points);
  auto extraction_time = ros::WallTime::now();

  // Compression
  std::stringstream compressed_data;
  octree_compression.encodePointCloud(diff_points, compressed_data);
  compressed_diff.data = compressed_data.str();
  auto diff_compress_time = ros::WallTime::now();

  // Publish diff point cloud
  Publish_String(&diff_compress_pub, 1, compressed_diff);

  auto end = ros::WallTime::now();

  auto duration_change_detector_init = (change_detector_init - start).toSec() * 1000.0;
  auto duration_static_cloud_load = (static_cloud_load - change_detector_init).toSec() * 1000.0;
  auto duration_dynamic_cloud_load = (dynamic_cloud_load - static_cloud_load).toSec() * 1000.0;
  auto duration_diff_time = (diff_time - dynamic_cloud_load).toSec() * 1000.0;
  auto duration_extraction_time = (extraction_time - diff_time).toSec() * 1000.0;
  auto duration_diff_compress_time = (diff_compress_time - extraction_time).toSec() * 1000.0;
  auto duration_end = (end - start).toSec() * 1000.0;

  log_file << pc_counter << "," 
    << duration_change_detector_init << "," 
    << duration_static_cloud_load << "," 
    << duration_dynamic_cloud_load << "," 
    << duration_diff_time << "," 
    << duration_extraction_time << "," 
    << duration_diff_compress_time << "," 
    << duration_end << std::endl;

  std::cout << "Number of diff points: " << newPointIdxVector.size() << std::endl;
  std::cout << "Diff time: " << duration_diff_time << " ms" << std::endl;

  // Publish diff cloud for debugging
  Publish_Cloud(&diff_pc_pub, pc_counter++, diff_points);

  std::cout << "Loading new change detector object" << std::endl;
  // delete octree_change_detector;
  // octree_change_detector = NULL;
  //octree_change_detector = Create_Change_Detector();
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
  pcl::PCLPointCloud2::Ptr input_cloud_pc2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*input_cloud, *input_cloud_pc2);
  input_cloud_pc2->header.frame_id = "os_sensor";
  pcl_conversions::toPCL(ros::Time::now(), input_cloud_pc2->header.stamp);
  pc_pub->publish(*input_cloud_pc2);
}

void Publish_String(ros::Publisher *pc_pub, int frame_number, fast_gicp::CompressDiff data) {
data.header.frame_id = "os_sensor";
data.header.stamp = ros::Time::now();
pc_pub->publish(data);
}


void initialize_octree (pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>& octree_change_add )
{
  octree_change_add.setInputCloud (infra_static_pc);
  octree_change_add.addPointsFromInputCloud ();
  octree_change_add.switchBuffers ();
}

std::vector<int> octree_change (pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>& octree_change_add,  pcl::PointCloud<pcl::PointXYZ>::Ptr pc )
{
    std::vector<int> newPointIdxVector_add;
    octree_change_add.setInputCloud (pc);
    octree_change_add.addPointsFromInputCloud ();
    octree_change_add.getPointIndicesFromNewVoxels (newPointIdxVector_add);
    octree_change_add.deleteCurrentBuffer();

    return newPointIdxVector_add;
}


//////////////////////////////////////////////////////////////////////////////