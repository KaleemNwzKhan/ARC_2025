// Input:
  //  A) PCDs in a rosbag
  //  B) Transformation file
// Output:
  //  A) Transformed pcd files

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

// Global variables
const std::string NODE_NAME = "pcd_transformer";
std::string input_infra_pcd_topic = "";
std::string output_infra_pcd_topic = "";
ros::Subscriber pc_sub;
ros::Publisher transformed_pc_pub;

// Function prototypes
int Sanity_Check(ros::NodeHandle nh);
void PCD_Transformer_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg);
void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string, ros::Publisher *pub_ptr, int frequency);
void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

int main(int argc, char **argv) {


    // Initialize ROS
    std::cout << "Initializing " << NODE_NAME << std::endl;
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;

    // Sanity Check
    if(Sanity_Check(nh) == -1) {
    std::cout << "Exitting early" << std::endl;
    return -1;
    }

    // Advertise PC topic
    Advertise_PC_Topic(&nh, output_infra_pcd_topic, &transformed_pc_pub, 10);

    //// 2. Main Loop /////
    std::cout << "Subscribing to point cloud topic: " << input_infra_pcd_topic << std::endl;
    pc_sub = nh.subscribe (input_infra_pcd_topic, 1000, PCD_Transformer_Callback);
    std::cout << "Going into ros::spin () ...." << std::endl;
    ros::spin();


//     pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile("/workspace/catkin_ws/"+target_pcd, *target_cloud);
//     pcl::PointCloud<pcl::PointXYZ> pc_transformed;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZ>);
//     Eigen::Matrix4f trans;
//     trans<<-0.99432615, -0.01285728, -0.10534294,  7.21220809,
//  0.01772098, -0.99885261, -0.04551818, 11.31863322,
//  -0.10461942, -0.04714063,  0.99341656,  0.97205018,
//   0. ,         0. ,         0. ,         1.  ;
//    clock_t tStart = clock();
//     pcl::transformPointCloud(*target_cloud, *ptr_transformed, trans);
// 		  double  time_cal=(double)(clock() - tStart)/CLOCKS_PER_SEC;
//                     time_cal=time_cal*1000.0;
// 		    printf("%.2f",time_cal);
//     pc_transformed = *ptr_transformed;
//     pcl::io::savePCDFileASCII ("/workspace/catkin_ws/I_Transformed.pcd", pc_transformed);
//     return 0;
    }


int Sanity_Check(ros::NodeHandle n_h) {

  if (n_h.getParam("/input_infra_pcd_topic", input_infra_pcd_topic))
    std::cout << "/input_infra_pcd_topic: " << input_infra_pcd_topic << std::endl;
  else {
    std::cout << "/input_infra_pcd_topic not set" << std::endl;
    return -1;
  }
  
  if (n_h.getParam("/output_infra_pcd_topic", output_infra_pcd_topic))
    std::cout << "/output_infra_pcd_topic: " << output_infra_pcd_topic << std::endl;
  else {
    std::cout << "/output_infra_pcd_topic not set" << std::endl;
    return -1;
  }

  return 0;
}

void PCD_Transformer_Callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> msg)
{
    static int pc_counter = 0;
    std::cout << "--------- Infra PC: " << pc_counter << " ----------------" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = msg->makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud ( new pcl::PointCloud<pcl::PointXYZ> () );

    // Transformation matrix
    Eigen::Matrix4f trans;
    trans <<0.87, -0.00, 0.50, -91.70,
            0.00, 1.00, 0.00, 135.40,
            -0.50, -0.00, 0.87, 5.00,
            0.00, 0.00, 0.00, 1.00;
    
    // Transformation
    pcl::transformPointCloud(*input_cloud, *output_cloud, trans);
    Publish_Cloud(&transformed_pc_pub, pc_counter++, output_cloud);
}

void Advertise_PC_Topic(ros::NodeHandle *nh_ptr, std::string pc_topic, ros::Publisher *pub_ptr, int frequency) {
  // Advertise topics
  *pub_ptr = nh_ptr->advertise<sensor_msgs::PointCloud2>(pc_topic, frequency);
  std::cout << "Advertised: " << pc_topic << " at " << frequency << " Hz" << std::endl;
  return;
}

void Publish_Cloud(ros::Publisher *pc_pub, int frame_number, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  input_cloud->header.frame_id = "os_sensor";
  pcl::PCLPointCloud2::Ptr input_cloud_pc2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*input_cloud, *input_cloud_pc2);
  pc_pub->publish(*input_cloud_pc2);
}