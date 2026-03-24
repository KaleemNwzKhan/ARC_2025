#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 #include <time.h>

// Input: pcl::PointCloud source, namely cloud_src
//Output: Transformed pcl::PointCloud, namely pc_transformed, via 4x4 transformation matrix
int main(int argc, char **argv){

    pcl::PointCloud<pcl::PointXYZ>::Ptr V_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/workspace/catkin_ws/V_transform.pcd", *V_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr I_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/workspace/catkin_ws/I_Transformed.pcd", *I_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
   clock_t tStart = clock();
   *merged=*I_cloud+*V_cloud;
	   double  time_cal=(double)(clock() - tStart)/CLOCKS_PER_SEC;
                    time_cal=time_cal*1000.0;
                    printf("%.2f",time_cal);
   pcl::io::savePCDFileASCII ("/workspace/catkin_ws/Final_Merged.pcd", *merged);
    return 0;


}

