#include <iostream>
#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main(int argc, char **argv){

    std::string target_pcd = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(target_pcd, *target_cloud);

    pcl::PointCloud<pcl::PointXYZ> pc_transformed;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f trans;
    trans<<-0.99432615, -0.01285728, -0.10534294,  7.21220809,
 0.01772098, -0.99885261, -0.04551818, 11.31863322,
 -0.10461942, -0.04714063,  0.99341656,  0.97205018,
  0. ,         0. ,         0. ,         1.  ;
    pcl::transformPointCloud(target_cloud, *ptr_transformed, trans);

    pc_transformed = *ptr_transformed;
    pcl::io::savePCDFileASCII ("I_Transformed.pcd", pc_transformed);
    return 0;

}
