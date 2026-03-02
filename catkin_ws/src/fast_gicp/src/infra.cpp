#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sstream>
#include <string.h>
#include <vector>  

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
ros::Subscriber pc_sub;
ros::Publisher diff_pc_pub;
ros::Publisher diff_compress_pub;

// global variavbles
const std::string NODE_NAME = "infra";
const int LIDAR_FREQUENCY = 10;
const int QUEUE_SIZE_PUB = 10;
const int QUEUE_SIZE_SUB = 1;
float resolution = 0.9;

// global objects 
pcl::PointCloud<pcl::PointXYZ>::Ptr infra_static_pc(new pcl::PointCloud<pcl::PointXYZ>());

fast_gicp::CompressDiff compressed_diff;

pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add (resolution);

pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add1 (resolution);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add2 (resolution);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add3 (resolution);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add4 (resolution);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add5 (resolution);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add6 (resolution);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add7 (resolution);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add8 (resolution);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add9 (resolution);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add10 (resolution);

std::vector<pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>> octree_vector ;



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

int main(int argc, char **argv) {

  ///// 1. Initialization Code /////


    octree_vector.push_back(octree_change_add1);
    octree_vector.push_back(octree_change_add2);
    octree_vector.push_back(octree_change_add3);
    octree_vector.push_back(octree_change_add4);
    octree_vector.push_back(octree_change_add5);
    octree_vector.push_back(octree_change_add6);
    octree_vector.push_back(octree_change_add7);
    octree_vector.push_back(octree_change_add8);
    octree_vector.push_back(octree_change_add9);
    octree_vector.push_back(octree_change_add10);

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


    for(std::vector<pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>>::size_type i = 0; i != octree_vector.size(); i++) 
    {
        initialize_octree(octree_vector[i]);

    }
//   initialize_octree(octree_change_add1);
//   initialize_octree(octree_change_add2);
//   initialize_octree(octree_change_add3);
//   initialize_octree(octree_change_add4);
//   initialize_octree(octree_change_add5);
//   initialize_octree(octree_change_add6);
//   initialize_octree(octree_change_add7);
//   initialize_octree(octree_change_add8);
//   initialize_octree(octree_change_add9);
//   initialize_octree(octree_change_add10);

  initialize_octree(octree_change_add);

  octree_change_add.setInputCloud (infra_static_pc);
  octree_change_add.addPointsFromInputCloud ();
  octree_change_add.switchBuffers ();
  // 1.4 Advertise point cloud topics for aligned points and the map
  std::cout << "Advertising point cloud topics for aligned points and the map" << std::endl;
  Advertise_PC_Topic(&nh, diff_pc_topic, &diff_pc_pub, QUEUE_SIZE_PUB);
  Advertise_String_Topic(&nh, diff_compress_topic, &diff_compress_pub, QUEUE_SIZE_PUB);


  compressed_diff.header.frame_id = "os_sensor";

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

//void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg) {
void Point_Cloud_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg) {

    //compressed_diff.header.stamp = ros::Time::now();
    //auto t1 = ros::WallTime::now();
    static int pc_counter = 0;
    
    //std::cout << "******************************" << std::endl;
    //std::cout << "---------- Point cloud: " << pc_counter << " -----------" << std::endl;

    //int octree_select = pc_counter/500;
    // pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add (resolution);
    // octree_change_add.setInputCloud (infra_static_pc);
    // octree_change_add.addPointsFromInputCloud ();
    // octree_change_add.switchBuffers ();
    


    //std::cout<< "octree_select = "<< octree_select <<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input_cloud = msg->makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr diff (new pcl::PointCloud<pcl::PointXYZ>());

    std::stringstream compressedData;

    std::vector<int> newPointIdxVector_add;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    auto t1 = ros::WallTime::now();

    octree_change_add.setInputCloud (raw_input_cloud);
    octree_change_add.addPointsFromInputCloud ();

    auto t2 = ros::WallTime::now();

    octree_change_add.getPointIndicesFromNewVoxels (newPointIdxVector_add);
    octree_change_add.deleteCurrentBuffer();

    // octree_vector[octree_select].setInputCloud (raw_input_cloud);
    // octree_vector[octree_select].addPointsFromInputCloud ();
    // octree_vector[octree_select].getPointIndicesFromNewVoxels (newPointIdxVector_add);
    // octree_vector[octree_select].deleteCurrentBuffer();

    //std::cout << "size of index vector ="<<newPointIdxVector_add.size()<<std::endl;

    inliers->indices = newPointIdxVector_add;
    extract.setInputCloud (raw_input_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*diff);

    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression1 (pcl::io::MANUAL_CONFIGURATION,false, 0.001, 0.1, false, 1, false, 8);
    octree_compression1.encodePointCloud(diff, compressedData);
    //std::cout<<compressedData.str() << std::endl;
    compressed_diff.data = compressedData.str();
  // Publish diff point cloud
    Publish_String(&diff_compress_pub, 1, compressed_diff);

    octree_change_add.switchBuffers ();
    octree_change_add.setInputCloud (infra_static_pc);
    octree_change_add.addPointsFromInputCloud ();
    octree_change_add.switchBuffers ();

    // auto t3 = ros::WallTime::now();
    // auto duration_dynamic_load= (t2 - t1).toSec();
    // auto duration_compress = (t3 - t2).toSec();

    // std::cout << "Latency_diff :" << duration_diff * 1000 << ": msec" << std::endl;
    // std::cout << "Latency_dynamnic_load:" << duration_dynamic_load * 1000 << ": msec" << std::endl;

    //Publish_Cloud(&diff_pc_pub, pc_counter++, diff);

//   Publish_Transform(&transform, 1, Get_Transform());
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
//data.header.frame_id = "os_sensor";
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