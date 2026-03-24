#include <vvr_ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>

VVR_Ndt::VVR_Ndt()
{
    return;
}

Eigen::Matrix4f VVR_Ndt::Do_Ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr src_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_ptr)
{
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    ndt.setInputSource(src_ptr);
    ndt.setInputTarget(tgt_ptr);

    ndt.setTransformationEpsilon(MIN_EPSILON);
    ndt.setResolution(RESOLUTION);
    //ndt.setMaximumIterations (705032704);
    ndt.setStepSize(MAX_STEP);
Eigen::Matrix4f init_guess;
  init_guess<< 0.93969262,  0.34202014 , 0,  -4.70483091,
  -0.34202014,  0.93969262,  0. ,   -1.45099729,
 0.00365182, -0.00501666  ,  0.999981 ,   -1,
          0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00; 
    //Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
    //Eigen::Translation3f init_translation(0.0, 0.0, 0.0);
    //Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    clock_t tStart = clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_ndt_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*src_ndt_ptr, init_guess);
    matrix = ndt.getFinalTransformation();
    printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    // fitness = ndt.getFitnessScore();
    // converge = ndt.hasConverged();

    return matrix;
}

using namespace std::chrono_literals;
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("src/eaad/src/pc_publisher/src/SOR_filtered.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from Map_Colored.pcd" << std::endl;
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (1, 1, 1);
  approximate_voxel_filter.setInputCloud (target_cloud);
  approximate_voxel_filter.filter (*target_cloud_1);
std::cout << "Loaded " << target_cloud_1->size() << " data points from Map_Colored.pcd" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("src/eaad/src/pc_publisher/src/V1_Transformed.pcd", *input_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from 0.pcd" << std::endl;

    VVR_Ndt obj = VVR_Ndt();
     clock_t tStart = clock();
    Eigen::Matrix4f a = obj.Do_Ndt(input_cloud, target_cloud_1);
    printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    std::cout << a;

    return (0);
}