// Input:
  //  A) The path to two point clouds
// Output:
  //  A) Stitched point cloud

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <filesystem>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

// Global variables
const std::string NODE_NAME = "pc_stitcher";
const std::string point_cloud_suffix = ".pcd";
const int LIDAR_FREQUENCY = 10;
std::string point_cloud_path = "";
int lidar_id = 0;
int number_of_clouds = 0;
int number_of_lidars = 0;
int start_index = 0;
std::string pc_topic_prefix = "";
std::string pc_topic_suffix = "";
std::string vehicle_point_cloud = "";
std::string infra_point_cloud = "";
#define LOG_RUN_TIME false
std::vector < ros::Publisher > pub;

// Function declarations
int Sanity_Check ( ros::NodeHandle );
bool Read_Point_Cloud ( const std::string, pcl::PCLPointCloud2::Ptr );
void Print_Number_Of_Points ( pcl::PCLPointCloud2 cloud );
void Publish_Cloud ( ros::Publisher &, pcl::PCLPointCloud2::Ptr input_cloud, std::string );
void Advertise_Topic ( ros::NodeHandle *nh_ptr, std::string, ros::Publisher & );
void ICP ( pcl::PCLPointCloud2::Ptr, pcl::PCLPointCloud2::Ptr, pcl::PCLPointCloud2::Ptr );

int
main (int argc, char** argv)
{
  ///// Initialization Code /////
  std::cout << "Initializing " << NODE_NAME << " ...." << std::endl;
  // Initialize ROS
  ros::init (argc, argv, NODE_NAME );
  ros::NodeHandle nh;
  // Argument sanity checks
  if ( Sanity_Check ( nh ) == -1 )
  {
    std::cout << "Exitting early" << std::endl;
    return -1;
  }
  // For logging point cloud publishing times
  std::ofstream logger;
  if ( LOG_RUN_TIME )
  {
    logger.open ("point_cloud_publisher.log");
    logger << "FrameID,PointCloudID,Seconds,Microseconds" << std::endl;
  }

  // Advertise topics
  ros::Publisher vpc_pub, ipc_pub, avpc_pub;
  Advertise_Topic ( &nh, "/vehicle/points", vpc_pub );
  Advertise_Topic ( &nh, "/infra/points", ipc_pub );
  Advertise_Topic ( &nh, "/aligned_vehicle/points", avpc_pub );

  ///////////////////////////
  ///// Read point clouds ///
  ///////////////////////////
  pcl::PCLPointCloud2::Ptr vehicle_pc ( new pcl::PCLPointCloud2  );
  pcl::PCLPointCloud2::Ptr infra_pc ( new pcl::PCLPointCloud2 );
//   std::cout << "Vehicle PC path: " << vehicle_point_cloud << std::endl;
//   std::cout << "Infra PC path: " << infra_point_cloud << std::endl;

  if ( Read_Point_Cloud ( vehicle_point_cloud, vehicle_pc ) == false )
  {
    std::cout << "Could not read vehicle point cloud, exitting" << std::endl;
    return -1;
  }

  if ( Read_Point_Cloud ( infra_point_cloud, infra_pc) == false )
  {
    std::cout << "Could not read infra point cloud, exitting" << std::endl;
    return -1;
  }

  ///////////////////////////
  ////// Run alignment //////
  ///////////////////////////
  pcl::PointCloud <pcl::PointXYZ>::Ptr vpc ( new pcl::PointCloud <pcl::PointXYZ> );
  pcl::PointCloud <pcl::PointXYZ>::Ptr ipc ( new pcl::PointCloud <pcl::PointXYZ> );
  pcl::PointCloud <pcl::PointXYZ>::Ptr vrpc ( new pcl::PointCloud <pcl::PointXYZ> );

  pcl::fromPCLPointCloud2 ( *vehicle_pc, *vpc );
  pcl::fromPCLPointCloud2 ( *infra_pc, *ipc );

  Publish_Cloud ( vpc_pub, vehicle_pc, "/vehicle/points" );
  Publish_Cloud ( ipc_pub, infra_pc, "/infra/points" );

  std::cout << "Starting alignment using ICP ...." << std::endl;

  pcl::PCLPointCloud2::Ptr vehicle_repositioned_cloud ( new pcl::PCLPointCloud2 );
  ICP ( vehicle_pc, infra_pc, vehicle_repositioned_cloud );
  std::cout << "Publishing point clouds now ...." << std::endl;

  pcl::fromPCLPointCloud2 ( *vehicle_repositioned_cloud, *vrpc );

  std::cout << "***** Publishing point clouds *****" << std::endl;
  while ( true )
  {
        Publish_Cloud ( vpc_pub, vehicle_pc, "/vehicle/points" );
        Publish_Cloud ( ipc_pub, infra_pc, "/infra/points" );
        Publish_Cloud ( avpc_pub, vehicle_repositioned_cloud, "/aligned_vehicle/points" );
        ros::Duration ( 10.0 / ( LIDAR_FREQUENCY ) ).sleep ();

  }

  /*
  // Read all the point clouds and start publishing them
  for ( int pc_counter = start_index; pc_counter < start_index + number_of_clouds; pc_counter++ )
  {
    for ( int lidar_counter = lidar_id; lidar_counter < lidar_id + number_of_lidars; lidar_counter++ )
    {
      // Temp storage for point clouds
      PointCloud::Ptr temp_cloud_storage ( new PointCloud () );
      // Path to point cloud
      std::string path_to_cloud = 
        point_cloud_prefixes.at ( lidar_counter - lidar_id ) + std::to_string ( pc_counter ) + point_cloud_suffix;
      // Read the point cloud
      // NdTimer op_timer ( "op_timer" );
      if ( Read_Point_Cloud ( path_to_cloud, temp_cloud_storage ) )
      {
        // input_clouds.push_back ( temp_cloud_storage );
        Publish_Cloud ( lidar_counter - lidar_id , pc_counter, temp_cloud_storage  );

        if ( LOG_RUN_TIME )
        {
          // long seconds = op_timer.Get_Current_Seconds ( );
          // long microseconds = op_timer.Get_Current_Microseconds ( );
          // logger << "frame_" << lidar_counter - lidar_id << "," 
              // << temp_cloud_storage->header.seq << "," << seconds << "," << microseconds << std::endl;
        }
      }
    }
    */
    // Here, we should have read all point clouds for a given frame
    // Next, we should advertise these point clouds
  //   std::cout << "*********************" << std::endl;
  //   ros::Duration ( 1.0 / LIDAR_FREQUENCY ).sleep ();
  //   if ( LOG_RUN_TIME )
  //     logger << std::flush;
  // // }

  // if ( LOG_RUN_TIME )
  //   logger.close ();
}

int Sanity_Check ( ros::NodeHandle n_h  )
{
  std::cout << "**********************" << std::endl;
  std::cout << "**** Sanity check ****" << std::endl;
  std::cout << "**********************" << std::endl;

  if ( n_h.getParam ("/vehicle_point_cloud", vehicle_point_cloud ) )
      std::cout << "Vehicle point cloud path: " << vehicle_point_cloud << std::endl;
  else
  {
      std::cout << "/vehicle_point_cloud parameter not set" << std::endl;
      return -1;
  }

  if ( n_h.getParam ("/infra_point_cloud", infra_point_cloud ) )
      std::cout << "Infra point cloud path: " << infra_point_cloud << std::endl;
  else
  {
      std::cout << "/infra_point_cloud parameter not set" << std::endl;
      return -1;
  }
 
  if ( n_h.getParam ("/pc_path", point_cloud_path ) )
      std::cout << "Point cloud path: " << point_cloud_path << std::endl;
  else
  {
      std::cout << "pc_path parameter not set" << std::endl;
      return -1;
  }

  std::cout << "**********************" << std::endl;
  std::cout << "**********************" << std::endl;
  std::cout << "**********************" << std::endl;
  
/*
  if ( n_h.getParam ("/start_index", start_index ) )
    std::cout << "Start index: " << start_index << std::endl;
  else
  {
    std::cout << "/start_index parameter not set" << std::endl;
    return -1;
  }

  if ( n_h.getParam ("/number_of_clouds", number_of_clouds ) )
      std::cout << "Number of clouds: " << number_of_clouds << std::endl;
  else
  {
      std::cout << "/number_of_clouds not set" << std::endl;
      return -1;
  }

  if ( n_h.getParam ( "/pc_topic_prefix", pc_topic_prefix ) )
    std::cout << "PC topic prefix: " << pc_topic_prefix << std::endl;
  else
  {
    std::cout << "/pc_topic_prefix not set" << std::endl;
    return -1;
  }

  if ( n_h.getParam ( "/pc_topic_suffix", pc_topic_suffix ) )
    std::cout << "PC topic prefix: " << pc_topic_suffix << std::endl;
  else
  {
    std::cout << "/pc_topic_suffix not set" << std::endl;
    return -1;
  }
*/
  // All good otherwise
  return 0;
}

bool Read_Point_Cloud 
  ( const std::string path_to_file, pcl::PCLPointCloud2::Ptr cloud)
{
  // pcl::PCLPointCloud2 cloud;
//   std::cout << "Reading: " << path_to_file << std::endl << std::flush;
  if (pcl::io::loadPCDFile (path_to_file, *cloud) == -1) //* load the file
    return false;
  else
    return true;
}

void Print_Number_Of_Points ( pcl::PCLPointCloud2 cloud )
{
//   std::cout << " : number of points: " << cloud.points.size () << std::endl;
}


void Publish_Cloud ( ros::Publisher &pub, pcl::PCLPointCloud2::Ptr input_cloud, std::string topic )
{
  input_cloud->header.frame_id = "os_sensor";
  pub.publish ( *input_cloud  );
//   std::cout << "Published point_cloud on " << topic << " with " 
    // << input_cloud->width * input_cloud->height <<  " points" << std::endl;
  return;
}

void Advertise_Topic ( ros::NodeHandle *nh_ptr, std::string topic, ros::Publisher &pub )
{
    pub = nh_ptr->advertise < sensor_msgs::PointCloud2 > ( topic, LIDAR_FREQUENCY );
    std::cout << "Advertised: " << topic << " at " << LIDAR_FREQUENCY << " Hz" << std::endl;
    return;
}

void ICP ( pcl::PCLPointCloud2::Ptr source, 
            pcl::PCLPointCloud2::Ptr target, 
              pcl::PCLPointCloud2::Ptr stitched_cloud )
{
  pcl::PointCloud <pcl::PointXYZ>::Ptr source_cloud ( new pcl::PointCloud <pcl::PointXYZ> );
  pcl::PointCloud <pcl::PointXYZ>::Ptr target_cloud ( new pcl::PointCloud <pcl::PointXYZ> );

  pcl::fromPCLPointCloud2 ( *source, *source_cloud );
  pcl::fromPCLPointCloud2 ( *source, *target_cloud );
  
  pcl::IterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource ( source_cloud );
  icp.setInputTarget ( target_cloud );
  icp.setMaximumIterations ( 1 );
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud <pcl::PointXYZ> );
  icp.align ( *final );

  std::cout << "Aligment is done ..." << std::endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  pcl::toPCLPointCloud2 ( *final, *stitched_cloud );
  return;
}