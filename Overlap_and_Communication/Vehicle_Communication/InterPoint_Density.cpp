#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

int main(int argc, char** argv) {
    // Check for proper number of arguments
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd>" << std::endl;
        return -1;
    }

    // Parse the input point cloud filename
    std::string input_pcd = argv[1];

    float minX =-56.9998;
    float minY =-5.99401;
    float maxX =-47.9998;
    float maxY = 6.00599;

    // Step 1: Load the point cloud from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd, *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file\n");
        return -1;
    }
    std::cout << "Loaded " << cloud->points.size() << " points from " << input_pcd << std::endl;

    // Step 2: Apply PassThrough filter to crop based on x range
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(minX, maxX);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
    pass_x.filter(*cloud_filtered_x);

    // Step 3: Apply PassThrough filter to crop based on y range
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud_filtered_x);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(minY, maxY);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZ>());
    pass_y.filter(*cloud_filtered_xy);

    std::cout << "After cropping, the cloud contains " << cloud_filtered_xy->points.size() << " points" << std::endl;
    cloud_filtered_xy->width = cloud_filtered_xy->points.size();  // Set width to the number of points
    cloud_filtered_xy->height = 1; 
    // Step 4: Save the cropped point cloud to a new file
    std::string output_pcd = "cropped_" + input_pcd;
    pcl::io::savePCDFileASCII(output_pcd, *cloud_filtered_xy);
    std::cout << "Cropped point cloud saved to " << output_pcd << std::endl;

    return 0;
}
