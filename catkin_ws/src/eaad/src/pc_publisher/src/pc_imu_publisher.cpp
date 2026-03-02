// Input:
//  A) The path to the point clouds
//  B) The number of point clouds to parse
//  C) The path to the IMU csv
// Output:
//  A) Publish point clouds and IMU to ROS

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>
#include <filesystem>
#include <iomanip>
#include <cfloat>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h> // C++17 standard header file name
#include <experimental/filesystem>          // Header file for pre-standard implementation
// Global variables
const std::string NODE_NAME = "pc_imu_publisher";
const std::string point_cloud_suffix = ".pcd";
const std::string imu_suffix = ".csv";
const int LIDAR_FREQUENCY = 10;
int imu_frequency = 0;
std::string point_cloud_1_path = "";
std::string point_cloud_2_path = "";
std::string imu_path = "";
std::string imu_stamp_path = "";
std::string vehicle_stamp_path = "";
// int lidar_id = 1;
int number_of_clouds = 0;
int number_of_lidars = 0;
int start_index = 0;
std::string pc_topic_prefix = "";
std::string pc_topic_suffix = "";
std::string imu_topic = "";
std::string vehicle_point_cloud = "";
std::string infra_point_cloud = "";
#define LOG_RUN_TIME false
std::vector<ros::Publisher> pub;
std::ifstream imu_csv;
std::ifstream vehicle_stamp;
int csv_line_number = 0;

// Function declarations
int Sanity_Check(ros::NodeHandle);
bool Read_Point_Cloud(const std::string, pcl::PCLPointCloud2::Ptr, std::string &);
void Print_Number_Of_Points(pcl::PCLPointCloud2 cloud);
void Publish_Cloud(int, int, pcl::PCLPointCloud2::Ptr input_cloud, const std::string, std::string);
void Advertise_Topics(ros::NodeHandle *nh_ptr, std::string, std::string, int);
void ICP(pcl::PCLPointCloud2::Ptr, pcl::PCLPointCloud2::Ptr, pcl::PCLPointCloud2::Ptr);
// std::ifstream Read_CSV_File ( std::string path_to_csv );
// std::string Read_CSV_Line ( std::ifstream &csv_file, int line_number );
// std::string Read_CSV_Next_Line ( std::ifstream &csv_file );
// void Parse_CSV_Line ( std::string csv_line, std::vector < float > &parsed_csv_line );
void Publish_IMU(ros::Publisher, std::string);
void Euler_To_Quat(double roll, double pitch, double yaw, double &q0, double &q1, double &q2, double &q3);
void Rotation_Matrix_To_Quat(double R[3][3], double &q0, double &q1, double &q2, double &q3);
ros::Time timestamp1, timestamp;
int main(int argc, char **argv)
{
  struct
  {
    bool operator()(std::filesystem::path a, std::filesystem::path b) const
    {
      std::string a1 = a;
      std::string b1 = b;
      // std::size_t lastindex = fullname.find_last_of(".");
      // std::string rawname = fullname.substr(0, lastindex);
      int a2 = stoi(a1.substr(a1.find_last_of("/\\") + 1));
      int b2 = stoi(b1.substr(b1.find_last_of("/\\") + 1));
      //std::cout << a2 << " " << b2 << std::endl;
      return a2 < b2;
    }
  } customLess;

  ///// Initialization Code /////
  std::cout << "Initializing " << NODE_NAME << std::endl;
  // Initialize ROS
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  // Argument sanity checks
  if (Sanity_Check(nh) == -1)
  {
    std::cout << "Exitting early" << std::endl;
    return -1;
  }
  // For logging point cloud publishing times
  std::ofstream logger;
  if (LOG_RUN_TIME)
  {
    logger.open("point_cloud_publisher.log");
    logger << "FrameID,PointCloudID,Seconds,Microseconds" << std::endl;
  }
  imu_csv.open(imu_path, std::ifstream::in);
  vehicle_stamp.open(vehicle_stamp_path, std::ifstream::in);
  // Read the IMU csv
  // imu_csv = Read_CSV_File ( imu_path );

  if (!imu_csv.is_open())
  {
    std::cout << "Could not open IMU csv file" << std::endl;
    return -1;
  }
  // else // discard the first line
  // Read_CSV_Next_Line ( imu_csv );

  // Adverstise point clouds and IMU
  // Advertise *number_of_lidars* topics wth *pc_topic_prefix* + _ + index + / + *pc_topic_suffix*
  Advertise_Topics(&nh, pc_topic_prefix, pc_topic_suffix, number_of_lidars);
  // Advertisement of IMU topics
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, imu_frequency);
  std::cout << "Advertised IMU on: " << imu_topic << " at " << imu_frequency << "Hz" << std::endl;

  int pc_counter = 0;
  // Read all the point clouds and start publishing them
  // for ( int pc_counter = start_index; pc_counter < start_index + number_of_clouds; pc_counter++ )
  //{
  std::string path_to_cloud = "";
  std::string path_to_cloud_temp = "";
  for (int lidar_counter = 0; lidar_counter < number_of_lidars; lidar_counter++)
  {
    // std::cout << "Parsed CSV line: ";
    // for ( int i = 0; i < parsed_csv_line.size (); i++ )
    // std::cout << parsed_csv_line.at ( i ) << " ";
    // std::cout << csv_line << std::endl;
    // Temp storage for point clouds
    pcl::PCLPointCloud2::Ptr temp_cloud_storage(new pcl::PCLPointCloud2);
    // Path to point cloud
    std::filesystem::path path_to_cloud;
    if (lidar_counter == 0)
      path_to_cloud = point_cloud_1_path;
    else
      path_to_cloud = point_cloud_2_path;

    std::vector<std::filesystem::path> files_in_directory;
    std::copy(std::filesystem::directory_iterator(path_to_cloud), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
    // int len = sizeof(files_in_directory.end())/sizeof(files_in_directory.begin());
    std::sort(files_in_directory.begin(), files_in_directory.end(), customLess);
    std::cout << "Will start publishing ..." << std::endl;
    std::cout << files_in_directory.size () << std::endl;
    for (const std::string &filename1 : files_in_directory)
    {
      ros::Duration(0.5).sleep(); 
// /      std::cout << filename1 << std::endl;
 //     std::cout << "Inside the loop ..." << std::endl;
      // std::vector <float> parsed_csv_line;
      // std::string csv_line = Read_CSV_Next_Line ( imu_csv );
      std::string csv_line = "";
      if (std::getline(imu_csv, csv_line))
      {
        // std::cout << csv_line << std::endl;
        Publish_IMU(imu_pub, csv_line);
        std::string line_vehicle = "";
        if (std::getline(vehicle_stamp, line_vehicle))
        {
          // Parse_CSV_Line ( csv_line, parsed_csv_line );

          // path_to_cloud += std::to_string (pc_counter) + point_cloud_suffix;
          path_to_cloud_temp = filename1;
          std::cout << path_to_cloud_temp << std::endl;
          // std::string path_to_cloud = ""
          // point_cloud_prefixes.at ( lidar_counter - lidar_id ) + std::to_string ( pc_counter ) + point_cloud_suffix;
          // Read the point cloud
          // NdTimer op_timer ( "op_timer" );
          std::string filename = "";
          if (Read_Point_Cloud(path_to_cloud_temp, temp_cloud_storage, filename))
          {
            // input_clouds.push_back ( temp_cloud_storage );
            Publish_Cloud(lidar_counter, pc_counter, temp_cloud_storage, filename, line_vehicle);
            if (LOG_RUN_TIME)
            {
              // long seconds = op_timer.Get_Current_Seconds ( );
              // long microseconds = op_timer.Get_Current_Microseconds ( );
              // logger << "frame_" << lidar_counter - lidar_id << ","
              // << temp_cloud_storage->header.seq << "," << seconds << "," << microseconds << std::endl;
            }
          }
        }
      }
    }
    /* for (int i = 0; i < 1; i++)
    {
      std::string csv_line = "";
      // std::vector <float> parsed_csv_line;
      // std::string csv_line = Read_CSV_Next_Line ( imu_csv );
      std::getline(imu_csv, csv_line);
      // Parse_CSV_Line ( csv_line, parsed_csv_line );
      Publish_IMU(imu_pub, csv_line);
    } */
  }

  /* // Read all the point clouds and start publishing them
  for ( int pc_counter = start_index; pc_counter < start_index + number_of_clouds; pc_counter++ )
  {
    std::string path_to_cloud = "";
    for ( int lidar_counter = 0; lidar_counter < number_of_lidars; lidar_counter++ )
    {
      std::vector <float> parsed_csv_line;
      std::string csv_line = Read_CSV_Next_Line ( imu_csv );
      Parse_CSV_Line ( csv_line, parsed_csv_line );
      Publish_IMU ( imu_pub, parsed_csv_line );
      // std::cout << "Parsed CSV line: ";
      // for ( int i = 0; i < parsed_csv_line.size (); i++ )
        // std::cout << parsed_csv_line.at ( i ) << " ";
      // std::cout << csv_line << std::endl;
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
      std::string filename="";
      if ( Read_Point_Cloud ( path_to_cloud, temp_cloud_storage, filename ) )
      {
        // input_clouds.push_back ( temp_cloud_storage );
        Publish_Cloud ( lidar_counter, pc_counter, temp_cloud_storage,filename  );

        if ( LOG_RUN_TIME )
        {
          // long seconds = op_timer.Get_Current_Seconds ( );
          // long microseconds = op_timer.Get_Current_Microseconds ( );
          // logger << "frame_" << lidar_counter - lidar_id << ","
              // << temp_cloud_storage->header.seq << "," << seconds << "," << microseconds << std::endl;
        }
      }
    }
  }  */
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

std::vector<std::string> tokenize(std::string s, std::string del = " ")
{
  std::vector<std::string> values;
  int start, end = -1 * del.size();
  do
  {
    start = end + del.size();
    end = s.find(del, start);
    values.push_back(s.substr(start, end - start));
  } while (end != -1);
  return values;
}

// Publish IMU
void Publish_IMU(ros::Publisher pub, std::string imu_line)
{
  // C++ version of the following CarLA Python code
  // https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge/src/carla_ros_bridge/imu.py
  const int TIMESTAMP = 1;
  // const int TIMESTAMP_MSEC = 2;
  const int ACCELEROMETER_X = 2;
  const int ACCELEROMETER_Y = 3;
  const int ACCELEROMETER_Z = 4;
  const int GYROSCOPE_X = 5;
  const int GYROSCOPE_Y = 6;
  const int GYROSCOPE_Z = 7;
  static int timestamp_counter = 0;
  sensor_msgs::Imu imu_msg;
  // Populate imu_msg with frame_id and time stamp
  imu_msg.header.frame_id = "os_imu";
  std::vector<std::string> imu_values = tokenize(imu_line, ", ");
  //std::cout << imu_values.at(ACCELEROMETER_X) << endl;
  // double sec1=double(imu_values.at( TIMESTAMP_SEC ));
  // double nano1=double(imu_values.at( TIMESTAMP_MSEC ));
  // timestamp1.sec=sec1;
  // timestamp1.nsec=nano1;
  // imu_msg.header.stamp=(ros::Time)(timestamp1.toNSec());
  // int secs=int(imu_values.at( TIMESTAMP_SEC ));
  // int nsecs=int(imu_values.at( TIMESTAMP_MSEC ));
  imu_msg.header.stamp = ros::Time(std::stold(imu_values.at(TIMESTAMP)));

  // pcl_conversions::toPCL(timestamp1, imu_msg.header.stamp);
  //std::cout << "IMU Header: " << imu_msg.header.stamp << std::endl;
  // std::cout<<"IMU time: "<<imu_msg.header.stamp<<endl;
  // std::cout<<imu_msg.header.stamp.nsec<<std::endl;
  // std::cout<<imu_msg.header.stamp.sec<<std::endl;
  // imu_msg.header.stamp = ros::Time::now ();
  //  Angular velocity
  //  Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
  //  Here, these measurements are converted to the right-handed ROS convention
  imu_msg.angular_velocity.x = (std::stold(imu_values.at(GYROSCOPE_X)));
  imu_msg.angular_velocity.y = (std::stold(imu_values.at(GYROSCOPE_Y)));
  imu_msg.angular_velocity.z = (std::stold(imu_values.at(GYROSCOPE_Z)));

  // Linear acceleration
  imu_msg.linear_acceleration.x = (std::stold(imu_values.at(ACCELEROMETER_X)));
  imu_msg.linear_acceleration.y = (std::stold(imu_values.at(ACCELEROMETER_Y)));
  imu_msg.linear_acceleration.z = (std::stold(imu_values.at(ACCELEROMETER_Z)));

  // Orientation
  imu_msg.orientation.x = 0;
  imu_msg.orientation.y = 0;
  imu_msg.orientation.z = 0;
  imu_msg.orientation.w = 0;

  pub.publish(imu_msg);
  // ros::Duration(3.0).sleep();
}

/* void Parse_CSV_Line ( std::string csv_line, std::vector < float > &parsed_csv_line )
{
  std::stringstream ss ( csv_line );
  std::string token;
  while ( std::getline ( ss, token, ',' ))
  {
    parsed_csv_line.push_back ( std::stof ( token ));
  }
} */

/* std::ifstream Read_CSV_File ( std::string path_to_csv )
{
  std::ifstream csv_file ( path_to_csv );
  if ( !csv_file.is_open () )
  {
    std::cout << "Could not open " << path_to_csv << std::endl;
    return csv_file;
  }
  return csv_file;
} */

/* std::string Read_CSV_Next_Line ( std::ifstream &csv_file )
{
  std::string line = "";
  std::getline ( csv_file, line );
  return line;
} */

/* std::string Read_CSV_Line ( std::ifstream &csv_file, int line_number )
{
  std::string line = "";
  for ( int i = 0; i < line_number; i++ )
    std::getline ( csv_file, line );
  return line;
} */

int Sanity_Check(ros::NodeHandle n_h)
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
  if (n_h.getParam("/number_of_lidars", number_of_lidars))
    std::cout << "Number of Lidars: " << number_of_lidars << std::endl;
  else
  {
    std::cout << "/number_of_lidars parameter not set" << std::endl;
    return -1;
  }

  if (n_h.getParam("/start_index", start_index))
    std::cout << "Start index: " << start_index << std::endl;
  else
  {
    std::cout << "/start_index parameter not set" << std::endl;
    return -1;
  }

  if (n_h.getParam("/number_of_clouds", number_of_clouds))
    std::cout << "Number of clouds: " << number_of_clouds << std::endl;
  else
  {
    std::cout << "/number_of_clouds not set" << std::endl;
    return -1;
  }

  if (n_h.getParam("/pc_topic_prefix", pc_topic_prefix))
    std::cout << "PC topic prefix: " << pc_topic_prefix << std::endl;
  else
  {
    std::cout << "/pc_topic_prefix not set" << std::endl;
    return -1;
  }

  if (n_h.getParam("/pc_topic_suffix", pc_topic_suffix))
    std::cout << "PC topic prefix: " << pc_topic_suffix << std::endl;
  else
  {
    std::cout << "/pc_topic_suffix not set" << std::endl;
    return -1;
  }

  // Point cloud paths
  if (n_h.getParam("/pc_path_1", point_cloud_1_path))
    std::cout << "Point cloud [1] path: " << point_cloud_1_path << std::endl;
  else
  {
    std::cout << "pc_path_1 parameter not set" << std::endl;
    return -1;
  }

  if (number_of_lidars != 1)
  {
    std::cout << "number_of_lidars != 1" << std::endl;
    return -1;
  }
  //   {
  //     if ( n_h.getParam ("/pc_path_2", point_cloud_2_path ) )
  //       std::cout << "Point cloud [2] path: " << point_cloud_2_path << std::endl;
  //     else
  //     {
  //     std::cout << "pc_path_2 parameter not set" << std::endl;
  //     return -1;
  //     }
  //   }

  // IMU path
  if (n_h.getParam("/imu_path", imu_path))
    std::cout << "IMU path: " << imu_path << std::endl;
  else
  {
    std::cout << "imu_path parameter not set" << std::endl;
    return -1;
  }

  if (n_h.getParam("/vehicle_timestamp_path", vehicle_stamp_path))
    std::cout << "Vehicle Timestamp path: " << vehicle_stamp_path << std::endl;
  else
  {
    std::cout << "Vehicle Timestamp path not set" << std::endl;
    return -1;
  }

  if (n_h.getParam("/imu_freq", imu_frequency))
    std::cout << "IMU freq: " << imu_frequency << std::endl;
  else
  {
    std::cout << "imu_freq parameter not set" << std::endl;
    return -1;
  }

  if (n_h.getParam("/imu_topic", imu_topic))
    std::cout << "IMU topic: " << imu_topic << std::endl;
  else
  {
    std::cout << "imu_topic parameter not set" << std::endl;
    return -1;
  }
  // All good otherwise
  return 0;
}

bool Read_Point_Cloud(const std::string path_to_file, pcl::PCLPointCloud2::Ptr cloud, std::string &filename)
{
  // pcl::PCLPointCloud2 cloud;
  std::string PCD_filename = path_to_file.substr(path_to_file.find_last_of("/\\") + 1);
  PCD_filename.erase(PCD_filename.find_last_of("."), std::string::npos);
  filename = PCD_filename;
  std::cout << "Reading: " << path_to_file << std::endl
            << std::flush;
  if (pcl::io::loadPCDFile(path_to_file, *cloud) == -1) //* load the file
    return false;
  else
    return true;
}

void Print_Number_Of_Points(pcl::PCLPointCloud2 cloud)
{
  //   std::cout << " : number of points: " << cloud.points.size () << std::endl;
}

void Publish_Cloud(int lidar_id, int frame_number, pcl::PCLPointCloud2::Ptr input_cloud, std::string filename, std::string line_vehicle)
{
  input_cloud->header.frame_id = "os_sensor";
  int starting = 0;
  // std::string delimiter = ".";
  // int ending = filename.find(delimiter);
  // std::cout<<filename.substr(starting, ending-starting)<<endl;
  // int sec=std::stold(filename.substr(starting, ending-starting));
  // std::cout<<filename.substr((ending + delimiter.size()), filename.find(delimiter,(ending + delimiter.size())))<<endl;
  // int nsec=std::stold(filename.substr((ending + delimiter.size()), filename.find(delimiter,(ending + delimiter.size()))));
  ros::Time timestamp;
  timestamp = ros::Time(std::stold(line_vehicle));
  //std::cout << "Vehile Timestamps:" << timestamp << endl;
  // input_cloud->header.stamp=timestamp;
  // std::cout <<filename<<std::endl;
  // input_cloud->header.stamp =  std::stold(filename);
  pcl_conversions::toPCL(timestamp, input_cloud->header.stamp);
  // input_cloud->header.stamp=(timestamp.toNSec());
  //std::cout << "PCD time: " << input_cloud->header.stamp << endl;
  pub.at(lidar_id).publish(*input_cloud);
  std::cout << "Published point_cloud [" << filename << "] from lidar [" << lidar_id << "] with "
            << input_cloud->width * input_cloud->height << " points" << std::endl;
  // ros::Duration(3.0).sleep();;
}

void Advertise_Topics(ros::NodeHandle *nh_ptr, std::string pc_prefix, std::string pc_suffix, int no_of_lidars)
{
  for (int counter = 0; counter < no_of_lidars; counter++)
  {
    std::string pc_topic = pc_topic_prefix + "/" + pc_topic_suffix;
    pub.push_back(nh_ptr->advertise<sensor_msgs::PointCloud2>(pc_topic, LIDAR_FREQUENCY));
    std::cout << "Advertised point clouds on: " << pc_topic << " at " << LIDAR_FREQUENCY << " Hz" << std::endl;
  }
}
