#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sstream>
#include <string.h>
#include <vector>  
#include <fast_gicp/CompressDiff.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
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
std::string infra_pc_topic_2 = "";
std::string diff_pc_topic = "";
std::string diff_compress_topic = "";


// publishers and subscribers
ros::Subscriber pc_sub;
ros::Publisher diff_pc_pub, infra_pc_pub;
ros::Publisher diff_compress_pub;

// global variavbles
const std::string NODE_NAME = "infra";
const int LIDAR_FREQUENCY = 10;
const int QUEUE_SIZE_PUB = 10;
const int QUEUE_SIZE_SUB = 1;
float resolution = 0.9;
auto t0 =ros::WallTime::now();

// global objects 
std::ofstream log_file, diff_size;
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

  int append = atoi(argv[1]);
  if (append == 1) {
    log_file.open("/workspace/log_time_infra.csv" , std::ios::app);
    diff_size.open("/workspace/diff_size.csv" , std::ios::app);
  }
  else{
    log_file.open("/workspace/log_time_infra.csv" );
    diff_size.open("/workspace/diff_size.csv");
    log_file << "frame_number" << "," << "Latency_pass"<< ","  << "Latency_diff" << ", "<< "Latency_compression" << std::endl;
    diff_size << "frame_number" << "," << "no of points"<< ","  << "compressed diff size" << std::endl;
  }
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



    // octree_vector.push_back(octree_change_add1);
    // octree_vector.push_back(octree_change_add2);
    // octree_vector.push_back(octree_change_add3);
    // octree_vector.push_back(octree_change_add4);
    // octree_vector.push_back(octree_change_add5);
    // octree_vector.push_back(octree_change_add6);
    // octree_vector.push_back(octree_change_add7);
    // octree_vector.push_back(octree_change_add8);
    // octree_vector.push_back(octree_change_add9);
    // octree_vector.push_back(octree_change_add10);

    // for(std::vector<pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>>::size_type i = 0; i != octree_vector.size(); i++) 
    // {
    //     initialize_octree(octree_vector[i]);

    // }
     initialize_octree(octree_change_add);
     //initialize_octree(octree_change_add1);
//   initialize_octree(octree_change_add2);
//   initialize_octree(octree_change_add3);
//   initialize_octree(octree_change_add4);
//   initialize_octree(octree_change_add5);
//   initialize_octree(octree_change_add6);
//   initialize_octree(octree_change_add7);
//   initialize_octree(octree_change_add8);
//   initialize_octree(octree_change_add9);
//   initialize_octree(octree_change_add10);


  // 1.4 Advertise point cloud topics for aligned points and the map
  std::cout << "Advertising point cloud topics for aligned points and the map" << std::endl;
  Advertise_PC_Topic(&nh, diff_pc_topic, &diff_pc_pub, QUEUE_SIZE_PUB);
  //Advertise_PC_Topic(&nh, infra_pc_topic_2, &infra_pc_pub, QUEUE_SIZE_PUB);
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
  
  //   if(n_h.getParam("/infra_pc_topic_2", infra_pc_topic_2))
  //   std::cout << "infra_pc_topic: " << infra_pc_topic_2 << std::endl;
  // else {
  //   std::cout << "/vehicle_pc_topic parameter not set" << std::endl;
  //   return -1;
  // }

  // if(n_h.getParam("/diff_pc_topic", diff_pc_topic))
  //   std::cout << "Diff point cloud topic: " << diff_pc_topic << std::endl;
  // else {
  //   std::cout << "/diff_pc_topic parameter not set" << std::endl;
  //   return -1;
  // }

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


    auto t1 = ros::WallTime::now();
    static int pc_counter = 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = msg->makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr diff (new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,0.0);
    pass.setNegative(true);
    pass.filter(*raw_input_cloud);

    //std::cout << raw_input_cloud->size() << std::endl;

    
    auto t2 = ros::WallTime::now();
  

    std::stringstream compressedData;

    std::vector<int> newPointIdxVector_add;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    octree_change_add.setInputCloud (raw_input_cloud);
    octree_change_add.addPointsFromInputCloud ();
    octree_change_add.getPointIndicesFromNewVoxels (newPointIdxVector_add);
    octree_change_add.deleteCurrentBuffer();

    //std::cout << "without zero points " <<newPointIdxVector_add1.size() << std::endl;


    inliers->indices = newPointIdxVector_add;
    extract.setInputCloud (raw_input_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*diff);

    auto t3 = ros::WallTime::now();
    
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression (pcl::io::MANUAL_CONFIGURATION,false, 0.001, 0.1, false, 1, false, 8);
    octree_compression.encodePointCloud(diff, compressedData);
    compressed_diff.data = compressedData.str();

    auto t4 = ros::WallTime::now();

    Publish_String(&diff_compress_pub, pc_counter, compressed_diff);

    compressedData.seekp(0, std::ios::end);
    int size = compressedData.tellp();
  // // Publish diff point cloud
   
  //   //auto t2 = ros::WallTime::now();
  //   // auto t3 = ros::WallTime::now();
    // auto duration_with0_pcd = (t4 - t3).toSec();
    // auto duration_without0_pcd = (t5 - t4).toSec();

    //std::cout << "latency with 0 points" << duration_with0_pcd*1000 << std::endl;
    //std::cout << "latency without 0 points" << duration_without0_pcd*1000 << std::endl;

    auto duration_pass = (t2 - t1).toSec()* 1000 ;
    auto duration_diff = (t3 - t2).toSec() *1000 ;
    auto duration_compress = (t4 - t3).toSec() *1000 ;
    // auto duration_total = (t6 - t1).toSec();
    // auto duration_compress = (t3 - t2).toSec();
    diff_size << pc_counter << "," << diff->size()  <<","<< size <<std::endl;
    log_file << pc_counter << "," << duration_pass <<","<< duration_diff <<"," << duration_compress <<std::endl;
    pc_counter++;
    //std::cout << "Latency_diff :" << duration_diff * 1000 << ": msec" << std::endl;
    //std::cout << "Latency_compress :" << duration_compress * 1000 << ": msec" << std::endl;
    //std::cout << pc_counter <<std::endl;
   
    //Publish_Cloud(&diff_pc_pub, pc_counter++, diff);
    //Publish_Cloud(&infra_pc_pub, pc_counter++, raw_input_cloud);
//   Publish_Transform(&transform, 1, Get_Transform());

    //octree_change_add.switchBuffers ();
    //initialize_octree(octree_change_add);

    octree_change_add.switchBuffers ();
    initialize_octree(octree_change_add);
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
  
  auto t = ros::WallTime::now();
  std::cout<< t << std::endl;
  auto t1 = ros::Time::now();
  t1.sec = t.sec;
  t1.nsec = t.nsec;
  //std::cout << t << std::endl;
  //std::cout << t1 << std::endl;
  
	//input_cloud_pc2->header.stamp = t1;
	//input_cloud_pc2->header.stamp.nsec = t.nsec;
  pcl_conversions::toPCL(t1, input_cloud_pc2->header.stamp);
  //std::cout << input_cloud_pc2->header.stamp << std::endl;
  pc_pub->publish(*input_cloud_pc2);
}

void Publish_String(ros::Publisher *pc_pub, int frame_number, fast_gicp::CompressDiff data) {
	//data.header.frame_id = "os_sensor";
	data.header.frame_id = std::to_string(frame_number);
	auto t = ros::WallTime::now();
  //data.header.stamp = ros::Time::now();
	data.header.stamp.sec = t.sec;
	data.header.stamp.nsec = t.nsec;
  //std::cout <<data.header.stamp << std::endl;
	//log_file << data.header.stamp << ","	<< ros::Time::now()<<  std::endl;

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
