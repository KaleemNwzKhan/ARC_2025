#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_to_pointcloud2_pub");
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/3d_map", 1, true); // latched

    // Load PCD
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("Intersection_32_Beam.pcd", *cloud) == -1)
    {
        ROS_ERROR("Couldn't read PCD file");
        return -1;
    }
    ROS_INFO("Loaded %zu points from PCD", cloud->points.size());

    // Convert to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "os_sensor";

    // Publish repeatedly
    ros::Rate rate(1); // 1 Hz
    while (ros::ok())
    {
        output.header.stamp = ros::Time::now();
        pub.publish(output);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
