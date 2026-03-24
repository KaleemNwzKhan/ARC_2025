// Input:
  //  A) The path to the point clouds
  //  B) The number of point clouds to parse
// Output:
  //  A) Publish point clouds to ROS

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <filesystem>
#include <pcl/registration/icp.h>
 #include <pcl/visualization/cloud_viewer.h>

// Global variables
const std::string NODE_NAME = "pc_publisher";
const std::string point_cloud_suffix = ".pcd";
const int LIDAR_FREQUENCY = 10;
std::string point_cloud_1_path = "";
std::string point_cloud_2_path = "";
//int lidar_id = 1;
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
void Publish_Cloud ( int, int, pcl::PCLPointCloud2::Ptr input_cloud );
void Advertise_Topics ( ros::NodeHandle *nh_ptr, std::string, std::string, int );
void ICP ( pcl::PCLPointCloud2::Ptr, pcl::PCLPointCloud2::Ptr, pcl::PCLPointCloud2::Ptr );

int
main (int argc, char** argv)
{
  ///// Initialization Code /////
  std::cout << "Initializing " << NODE_NAME << std::endl;
  // Initialize ROS
  ros::init (argc, argv, NODE_NAME );
  ros::NodeHandle nh;
  // Argument sanity checks
  cout << number_of_lidars << endl ;
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
  // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

  // Advertise *number_of_lidars* topics wth *pc_topic_prefix* + _ + index + / + *pc_topic_suffix* 
  Advertise_Topics ( &nh, pc_topic_prefix, pc_topic_suffix, number_of_lidars );
  std::cout << "Advertised topics" << std::endl;  

  // // Read all the point clouds and start publishing them
  // for ( int pc_counter = start_index; pc_counter < start_index + number_of_clouds; pc_counter++ )
  // {
  //   // Temp storage for point clouds
  //   pcl::PCLPointCloud2::Ptr temp_cloud_storage ( new pcl::PCLPointCloud2 );
  //   // Path to point cloud
  //   std::string path_to_cloud = 
  //     point_cloud_path + std::to_string ( pc_counter ) + point_cloud_suffix;
  //   // Read the point cloud
  //   if ( Read_Point_Cloud ( path_to_cloud, temp_cloud_storage ) )
  //   {
  //     Publish_Cloud ( pc_counter, temp_cloud_storage  );
  //     ros::Duration ( 1.0 / ( LIDAR_FREQUENCY ) ).sleep ();
  //   }
  // }
  // return 0;



  // /////////////////////////
  // /// Read point clouds ///
  // /////////////////////////
  // pcl::PCLPointCloud2::Ptr vehicle_pc ( new pcl::PCLPointCloud2  );
  // pcl::PCLPointCloud2::Ptr infra_pc ( new pcl::PCLPointCloud2 );
  // std::cout << "Vehicle PC path: " << vehicle_point_cloud << std::endl;
  // std::cout << "Infra PC path: " << infra_point_cloud << std::endl;

  // if ( Read_Point_Cloud ( vehicle_point_cloud, vehicle_pc ) )
  // {
  //   std::cout << "Read vehicle point cloud ...." << std::endl;
  // }

  // if ( Read_Point_Cloud ( infra_point_cloud, infra_pc) )
  // {
  //   std::cout << "Read infra point cloud ...." << std::endl;
  //   // std::cout << "Read infra point cloud at: " << infra_point_cloud << std::endl;
  // }

  ////////// Run alignment

  // pcl::PointCloud <pcl::PointXYZ>::Ptr vpc ( new pcl::PointCloud <pcl::PointXYZ> );
  // pcl::PointCloud <pcl::PointXYZ>::Ptr ipc ( new pcl::PointCloud <pcl::PointXYZ> );
  // pcl::PointCloud <pcl::PointXYZ>::Ptr vrpc ( new pcl::PointCloud <pcl::PointXYZ> );

  // pcl::fromPCLPointCloud2 ( *vehicle_pc, *vpc );
  // pcl::fromPCLPointCloud2 ( *infra_pc, *ipc );


  // viewer.showCloud ( vpc );
  // viewer.showCloud ( ipc );

  // std::cout << "Publishing vehicle and infra point clouds ...." << std::endl;
  // Publish_Cloud ( 0, 0, vehicle_pc );
  // Publish_Cloud ( 1, 0, infra_pc );

  // pcl::PCLPointCloud2::Ptr vehicle_repositioned_cloud ( new pcl::PCLPointCloud2 );
  // std::cout << "Running ICP ...." << std::endl;
  // ICP ( vehicle_pc, infra_pc, vehicle_repositioned_cloud );
  // std::cout << "Publishing point clouds again" << std::endl;
 

  // pcl::fromPCLPointCloud2 ( *vehicle_repositioned_cloud, *vrpc );


  // viewer.showCloud ( vrpc );

  // while (!viewer.wasStopped ());

  //   return 0;
  // int counter = 0;
  // while ( true )
  // {
    
  //   Publish_Cloud ( 0, counter, vehicle_pc );
  //   Publish_Cloud ( 1, counter, infra_pc );
  //   Publish_Cloud ( 2, counter++, vehicle_repositioned_cloud );

  //   ros::Duration ( 1.0 / ( LIDAR_FREQUENCY ) ).sleep ();

  // }

  
  // Read all the point clouds and start publishing them
  for ( int pc_counter = start_index; pc_counter < start_index + number_of_clouds; pc_counter++ )
  {
    std::string path_to_cloud = "";
    for ( int lidar_counter = 0; lidar_counter < number_of_lidars; lidar_counter++ )
    {
      // Temp storage for point clouds
      pcl::PCLPointCloud2::Ptr temp_cloud_storage ( new pcl::PCLPointCloud2 );
      // Path to point cloud
      if ( lidar_counter == 0 )
        path_to_cloud = point_cloud_1_path;
      else
        path_to_cloud = point_cloud_2_path;

      path_to_cloud += std::to_string (pc_counter) + point_cloud_suffix;
      // std::string path_to_cloud = ""
        // point_cloud_prefixes.at ( lidar_counter - lidar_id ) + std::to_string ( pc_counter ) + point_cloud_suffix;
      // Read the point cloud
      // NdTimer op_timer ( "op_timer" );
      if ( Read_Point_Cloud ( path_to_cloud, temp_cloud_storage ) )
      {
        // input_clouds.push_back ( temp_cloud_storage );
        Publish_Cloud ( lidar_counter, pc_counter, temp_cloud_storage  );

        if ( LOG_RUN_TIME )
        {
          // long seconds = op_timer.Get_Current_Seconds ( );
          // long microseconds = op_timer.Get_Current_Microseconds ( );
          // logger << "frame_" << lidar_counter - lidar_id << "," 
              // << temp_cloud_storage->header.seq << "," << seconds << "," << microseconds << std::endl;
        }
      }
    }
  } 
    // Here, we should have read all point clouds for a given frame
    // Next, we should advertise these point clouds
  //   std::cout << "*********************" << std::endl;
  //   ros::Duration ( 1.0 / LIDAR_FREQUENCY ).sleep ();
  //   if ( LOG_RUN_TIME )
  //     logger << std::flush;
  // // }

  // if ( LOG_RUN_TIME )
  //   logger.close ();
  return 0;
}

int Sanity_Check ( ros::NodeHandle n_h  )
{
  /*
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
  */

/*
  if ( n_h.getParam ("/lidar_id", lidar_id ) )
      std::cout << "First Lidar ID: " << lidar_id << std::endl;
  else
  {
      std::cout << "lidar_id parameter not set" << std::endl;
      return -1;
  }
*/
  if ( n_h.getParam ("/number_of_lidars", number_of_lidars ) )
    std::cout << "Number of Lidars: " << number_of_lidars << std::endl;
  else
  {
    std::cout << "/number_of_lidars parameter not set" << std::endl;
    return -1;
  }

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

  // Point cloud paths
  if ( n_h.getParam ("/pc_path_1", point_cloud_1_path ) )
      std::cout << "Point cloud [1] path: " << point_cloud_1_path << std::endl;
  else
  {
      std::cout << "pc_path_1 parameter not set" << std::endl;
      return -1;
  }

  if ( number_of_lidars > 1 )
  {
    if ( n_h.getParam ("/pc_path_2", point_cloud_2_path ) )
      std::cout << "Point cloud [2] path: " << point_cloud_2_path << std::endl;
    else
    {
    std::cout << "pc_path_2 parameter not set" << std::endl;
    return -1;
    }
  }
  // All good otherwise
  return 0;
}

bool Read_Point_Cloud 
  ( const std::string path_to_file, pcl::PCLPointCloud2::Ptr cloud)
{
  // pcl::PCLPointCloud2 cloud;
  std::cout << "Reading: " << path_to_file << std::endl << std::flush;
  if (pcl::io::loadPCDFile (path_to_file, *cloud) == -1) //* load the file
    return false;
  else
    return true;
}

void Print_Number_Of_Points ( pcl::PCLPointCloud2 cloud )
{
  // std::cout << " : number of points: " << cloud.points.size () << std::endl;
}


void Publish_Cloud ( int lidar_id, int frame_number, pcl::PCLPointCloud2::Ptr input_cloud )
{
  // input_cloud->header.frame_id = "lidar_" + std::to_string ( id ) + "_frame_" + std::to_string ( frame_number ) ;
  // std::cin.get();
  input_cloud->header.frame_id = "os_sensor";
  // input_cloud->header.seq =   frame_number;
  pub.at ( lidar_id ).publish ( *input_cloud  );
  std::cout << "Published point_cloud [" << frame_number << "] from lidar [" << lidar_id << "] with "
    << input_cloud->width * input_cloud->height <<  " points" << std::endl;
  // pub.publish ( ros_ptr ( input_cloud ) );
  /*

  switch ( id )
  {
    case 0:
      pub_one.publish ( ros_ptr ( input_cloud ) );
      break;

    case 1:
      pub_two.publish ( ros_ptr ( input_cloud ) );
      break;
      
    case 2:
      pub_three.publish ( ros_ptr ( input_cloud ) );
      break;
      
    case 3:
      pub_four.publish ( ros_ptr ( input_cloud ) );
      break;

    default:
      std::cout << "Should not be here" << std::endl;
      break;
  }
  */
  // std::cout << "Published point cloud [ " << lidar_id << " ]: " << input_cloud->points.size () << " points" << std::endl;
}

void Advertise_Topics ( ros::NodeHandle *nh_ptr, std::string pc_prefix, std::string pc_suffix, int no_of_lidars )
{

  for ( int counter = 0; counter <= no_of_lidars; counter++ )
  {
    std::string pc_topic = pc_topic_prefix + "/" + pc_topic_suffix + "_" + std::to_string ( counter );
    pub.push_back ( nh_ptr->advertise<sensor_msgs::PointCloud2>   ( pc_topic, LIDAR_FREQUENCY) );
    std::cout << "Advertised: " << pc_topic << " at " << LIDAR_FREQUENCY << " Hz" << std::endl;
  }

  // Pubish topics
  // for ( int counter = 0; counter < no_of_lidars - 1; counter++ )
  {

    // std::string pc_topic = "";
    // if ( counter == 0)
    //   pc_topic = "vehicle";
    // else if ( counter == 1)
    //   pc_topic = "infra";
    // else
    //   pc_topic = "repos_vehicle";
    

    // pub.push_back ( nh_ptr->advertise<sensor_msgs::PointCloud2>   ( pc_topic, LIDAR_FREQUENCY) );
    // std::cout << "Advertised: " << pc_topic << " at " << LIDAR_FREQUENCY << " Hz" << std::endl;
  }
}

// void ICP ( pcl::PCLPointCloud2::Ptr source, 
//             pcl::PCLPointCloud2::Ptr target, 
//               pcl::PCLPointCloud2::Ptr stitched_cloud )
// {
//   pcl::PointCloud <pcl::PointXYZ>::Ptr source_cloud ( new pcl::PointCloud <pcl::PointXYZ> );
//   pcl::PointCloud <pcl::PointXYZ>::Ptr target_cloud ( new pcl::PointCloud <pcl::PointXYZ> );

//   pcl::fromPCLPointCloud2 ( *source, *source_cloud );
//   pcl::fromPCLPointCloud2 ( *source, *target_cloud );
  
//   pcl::IterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setInputSource ( source_cloud );
//   icp.setInputTarget ( target_cloud );
//   icp.setMaximumIterations ( 1 );
  
  
//   pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud <pcl::PointXYZ> );
//   std::cout << "Starting alignment ..." << std::endl;
//   icp.align ( *final );

//   std::cout << "Aligment is done ..." << std::endl;
//   std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//   icp.getFitnessScore() << std::endl;
//   std::cout << icp.getFinalTransformation() << std::endl;

//   pcl::toPCLPointCloud2 ( *final, *stitched_cloud );
//   Publish_Cloud ( 2, 0, stitched_cloud );

// }
