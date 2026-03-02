#ifndef VVR_NDT_H
#define VVR_NDT_H

#include <chrono>
#include <string>
#include <stdlib.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

class VVR_Ndt {
private:
    float MIN_EPSILON = 0.01;
    float RESOLUTION = 5;
    float MAX_STEP = 5;

public:
    Eigen::Matrix4f matrix;
    double fitness;
    int converge;

    VVR_Ndt();
    Eigen::Matrix4f Do_Ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr src_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_ptr);
};

#endif