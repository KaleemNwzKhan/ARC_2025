#include <iostream>
#include <fstream>
#include <unordered_set>
#include <filesystem>
#include <vector>  
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>
#include <pcl/filters/passthrough.h>
#include <omp.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h> //for minmax3D
#include <cuda_runtime.h>
using namespace std;


std::string dataset_folder, Map, Vehicle_PCDs_Path;
float accumulate_time=0, time1, time2,time3,time4, time5, time6, run_time_start,run_time_end,run_time_start1,run_time_end1;
const int threadsPerBlock = 256;
const float cell_x_size = 2.5;
const float cell_y_size = 2.5;
const float cell_z_size = 4.0;
std::string filename = "grid_centroids.pcd";
std::vector<pcl::PointXYZ> positions_list;

float xmin = -57.0;
float xmax = -42.0;
float ymin = -9.0;
float ymax = 11.0;
std::string decide="";

//-------------------------------Diff CUDA Part----------------------------------------
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
// CUDA kernel for nearest neighbor search
__global__ void nearestNeighborSearch(const PointT* cloud2, const PointT* cloud1, 
                                      const int cloud2_size, const int cloud1_size,
                                      float threshold, int* change_indices, 
                                      int* change_count) {
    extern __shared__ PointT shared_cloud1[]; // Shared memory for a tile of cloud1

    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < cloud2_size) {
        PointT point2 = cloud2[idx];
        float min_distance = threshold * threshold; // Use squared distance for comparison
        int nearest_idx = -1;

        // Loop over tiles of cloud1
        for (int tile = 0; tile < (cloud1_size + blockDim.x - 1) / blockDim.x; ++tile) {
            int local_idx = tile * blockDim.x + threadIdx.x;
            // Load a tile of cloud1 into shared memory
            if (local_idx < cloud1_size) {
                shared_cloud1[threadIdx.x] = cloud1[local_idx];
            }
            __syncthreads();

            // Compare point2 with each point in the tile of cloud1 in shared memory
            for (int j = 0; j < blockDim.x && tile * blockDim.x + j < cloud1_size; ++j) {
                PointT point1 = shared_cloud1[j];
                float distance = (point2.x - point1.x) * (point2.x - point1.x) +
                                 (point2.y - point1.y) * (point2.y - point1.y) +
                                 (point2.z - point1.z) * (point2.z - point1.z);
                
                // Update nearest point if closer than current minimum distance
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest_idx = tile * blockDim.x + j;
                }
            }
            __syncthreads(); // Ensure all threads finish using shared memory before loading the next tile
        }

        // If the nearest point is further than the threshold
        if (nearest_idx == -1 || min_distance > threshold * threshold) {
            int pos = atomicAdd(change_count, 1);
            change_indices[pos] = idx; // Store the index of the point in cloud2 that changed
        }
    }
}

void detectChangesCUDA(const PointCloudT::Ptr& cloud1, const PointCloudT::Ptr& cloud2, 
                       double threshold, PointCloudT::Ptr& changes) {
    PointT* d_cloud1;
    PointT* d_cloud2;
    int* d_change_indices;
    int* d_change_count;

    int cloud1_size = cloud1->size();
    int cloud2_size = cloud2->size();
    
    cudaMalloc(&d_cloud1, cloud1_size * sizeof(PointT));
    cudaMalloc(&d_cloud2, cloud2_size * sizeof(PointT));
    cudaMalloc(&d_change_indices, cloud2_size * sizeof(int)); // Max possible changes
    cudaMalloc(&d_change_count, sizeof(int));
    cudaMemset(d_change_count, 0, sizeof(int));

    // Copy point clouds to device
    cudaMemcpy(d_cloud1, cloud1->points.data(), cloud1_size * sizeof(PointT), cudaMemcpyHostToDevice);
    cudaMemcpy(d_cloud2, cloud2->points.data(), cloud2_size * sizeof(PointT), cudaMemcpyHostToDevice);

    // Launch CUDA kernel
    int blockSize = 256; // Number of threads per block
    int numBlocks = (cloud2_size + blockSize - 1) / blockSize;
    size_t sharedMemSize = sizeof(PointT) * blockSize;
    
    run_time_start = clock();
    nearestNeighborSearch<<<numBlocks, blockSize,sharedMemSize>>>(d_cloud2, d_cloud1, cloud2_size, cloud1_size, threshold, d_change_indices, d_change_count);
    cudaDeviceSynchronize();
    run_time_end = clock();

    time3 = ((float(run_time_end - run_time_start) / CLOCKS_PER_SEC)*1000);
    // std::cout << "Time taken for nearest neighbor search: " << elapsed_time << " seconds." << std::endl;

    int change_count;
    cudaMemcpy(&change_count, d_change_count, sizeof(int), cudaMemcpyDeviceToHost);
    std::vector<int> change_indices(change_count);
    cudaMemcpy(change_indices.data(), d_change_indices, change_count * sizeof(int), cudaMemcpyDeviceToHost);

    // Store changed points in the changes point cloud
    for (int i = 0; i < change_count; ++i) {
        changes->points.push_back(cloud2->points[change_indices[i]]);
    }

    // Free device memory
    cudaFree(d_cloud1);
    cudaFree(d_cloud2);
    cudaFree(d_change_indices);
    cudaFree(d_change_count);

    // std::cout << "Change detection found " << changes->points.size() << " changed points." << std::endl;
    // changes->width = changes->points.size();
    // changes->height = 1;
    //      pcl::io::savePCDFileASCII("filtered_detected_changes.pcd", *changes);
    //     std::cout << "Filtered detected changes saved to filtered_detected_changes.pcd" << std::endl;

    // std::cout << "Filtered changes contain " << changes->points.size() << " points from valid clusters." << std::endl;
}



//-------------------------------------------Positions Assignment-----------------------------------
// PointXYZ structure for CUDA


// Kernel to assign each point in the point cloud to the nearest position
__global__ void assignPointsToPositions(pcl::PointXYZ* cloud_points, int cloud_size, pcl::PointXYZ* positions, int positions_size, int* assignments) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < cloud_size) {
        pcl::PointXYZ point = cloud_points[idx];
        float min_distance = FLT_MAX;
        int nearest_position = -1;

        // Loop over each position to find the nearest one
        for (int i = 0; i < positions_size; ++i) {
            // Calculate Euclidean distance considering x, y, and z
            float distance = (point.x - positions[i].x) * (point.x - positions[i].x) +
                             (point.y - positions[i].y) * (point.y - positions[i].y) +
                             (point.z - positions[i].z) * (point.z - positions[i].z);

            // Update the nearest position if this distance is smaller
            if (distance < min_distance) {
                min_distance = distance;
                nearest_position = i;
            }
        }

        // Store the index of the nearest position for this point
        assignments[idx] = nearest_position;
    }
}




#pragma omp declare reduction(vec_int_plus : std::vector<int> : \
                              std::transform(omp_out.begin(), omp_out.end(), omp_in.begin(), omp_out.begin(), std::plus<float>())) \
                    initializer(omp_priv = omp_orig)

#define NUM_PROCS 8

__device__ inline int getGridCentroidIdx( int, int, int, int, int, int );
__device__ float computeDist ( float, float );
__device__ float computeDist ( float, float, float, float, float, float );
__device__ inline bool isWithinBounds( int, int, int, int, int, int );
inline bool isWithinBoundsHost( pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ );

struct CellInfo {
    float minX;
    float minY;
    float maxX;
    float maxY;
    float centerX;
    float centerY;
    float centerZ;

    __device__ CellInfo(float x, float y, float z, float cell_x_size, float cell_y_size) {
        centerX = x;
        centerY = y;
        centerZ = z;
        minX = centerX - cell_x_size / 2;
        maxX = centerX + cell_x_size / 2;
        minY = centerY - cell_y_size / 2;
        maxY = centerY + cell_y_size / 2;
    }
};

struct BoundingBox {
    float minX, minY, maxX, maxY;
};

__device__ float computeDist1(float x1, float y1, float z1, float x2, float y2, float z2) {
    return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

__device__ float atomicMinFloat(float* address, float value) {
    int* address_as_int = (int*)address;
    int old = *address_as_int, assumed;
    
    do {
        assumed = old;
        old = atomicCAS(address_as_int, assumed, __float_as_int(fminf(value, __int_as_float(assumed))));
    } while (assumed != old);

    return __int_as_float(old);
}

__device__ float atomicMaxFloat(float* address, float value) {
    int* address_as_int = (int*)address;
    int old = *address_as_int, assumed;
    
    do {
        assumed = old;
        old = atomicCAS(address_as_int, assumed, __float_as_int(fmaxf(value, __int_as_float(assumed))));
    } while (assumed != old);

    return __int_as_float(old);
}

__global__ void updateBoundingBoxesKernel(
    const pcl::PointXYZ* centroids,
    const int* vehicle_pd,
    const pcl::PointXYZ* positions_list,
    BoundingBox* bounding_boxes,
    int* cells_count,
    int vehicle_size,
    int positions_size,
    float cell_x_size,
    float cell_y_size) {
    
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= vehicle_size || vehicle_pd[i] != 0) return;

    pcl::PointXYZ centroid = centroids[i];
    float min_dist = FLT_MAX;
    int closest_position_idx = -1;

    for (int pos_idx = 0; pos_idx < positions_size; ++pos_idx) {
        float dist = computeDist1(centroid.x, centroid.y, centroid.z,
                                  positions_list[pos_idx].x, positions_list[pos_idx].y, positions_list[pos_idx].z);
        if (dist < min_dist) {
            min_dist = dist;
            closest_position_idx = pos_idx;
        }
    }
    if (closest_position_idx != -1) {
        CellInfo cell_info(centroid.x, centroid.y, centroid.z, cell_x_size, cell_y_size);
        atomicMinFloat(&bounding_boxes[closest_position_idx].minX, cell_info.minX);
        atomicMinFloat(&bounding_boxes[closest_position_idx].minY, cell_info.minY);
        atomicMaxFloat(&bounding_boxes[closest_position_idx].maxX, cell_info.maxX);
        atomicMaxFloat(&bounding_boxes[closest_position_idx].maxY, cell_info.maxY);
        atomicAdd(&cells_count[closest_position_idx], 1);
    }
}

std::vector<pcl::PointXYZ> get_grid_centroids(const std::vector<pcl::PointXYZ>& grid_info) {
    std::vector<pcl::PointXYZ> centroids;
    
    pcl::PointXYZ min_centroid = grid_info[0];
    pcl::PointXYZ lims = grid_info[2];
    
for (int j = 0; j < static_cast<int>(lims.y); ++j) { // y-dimension cells
    for (int i = 0; i < static_cast<int>(lims.x); ++i) { // x-dimension cells
            for (int k = 0; k < static_cast<int>(lims.z); ++k) { // z-dimension cells
                pcl::PointXYZ centroid;
                centroid.x = min_centroid.x + i * cell_x_size;
                centroid.y = min_centroid.y + j * cell_y_size;
                centroid.z = min_centroid.z + k * cell_z_size;
                
                centroids.push_back(centroid);
            }
        }
    }
    
    return centroids;
}
#define cudaCheckError() {                                          \
    cudaError_t err = cudaGetLastError();                           \
    if (err != cudaSuccess) {                                       \
        std::cerr << "CUDA Error: " << cudaGetErrorString(err)      \
                  << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
        exit(EXIT_FAILURE);                                         \
    }                                                               \
}

__global__ void incrementKernel(int* d_result2, int* d_vehicle_pd, int cloudSize, int gridSize) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < cloudSize) {
        int bin = d_result2[idx];
        if (bin < gridSize) { 
            atomicAdd(&d_vehicle_pd[bin], 1);
        }
    }
}


__global__ void getNN( int N, pcl::PointXYZ *points, int *res, float min_x, float min_y, float min_z, int lims_x, int lims_y, int lims_z ) {
    size_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if ( tid < N ) {
        float curr_x = points[tid].x;
        float curr_y = points[tid].y;
        float curr_z = points[tid].z;

        int xT = (curr_x - min_x) / cell_x_size;
        int yT = (curr_y - min_y) / cell_y_size;
        int zT = (curr_z - min_z) / cell_z_size;

        const int step = 1;
        float min_dist_to_centroid = (cell_x_size + cell_y_size + cell_z_size) * (cell_x_size + cell_y_size + cell_z_size);
        int closest_xt;
        int closest_yt;
        int closest_zt;

        for (int xC = xT - step; xC <= xT + step; xC++) {
            for (int yC = yT - step; yC <= yT + step; yC++) {
                for (int zC = zT - step; zC <= zT + step; zC++) {
                    float curr_grid_x = min_x + xC * cell_x_size;
                    float curr_grid_y = min_y + yC * cell_y_size;
                    float curr_grid_z = min_z + zC * cell_z_size;

                    if (isWithinBounds(xC, yC, zC, lims_x, lims_y, lims_z)) {
                        float curr_dist = computeDist(curr_x, curr_y, curr_z, curr_grid_x, curr_grid_y, curr_grid_z);
                        if (curr_dist < min_dist_to_centroid) {
                            closest_xt = xC;
                            closest_yt = yC;
                            closest_zt = zC;
                            min_dist_to_centroid = curr_dist;
                        }
                    }
                }
            }
        }

        res[tid] = getGridCentroidIdx(closest_xt, closest_yt, closest_zt, lims_x, lims_y, lims_z);
    }
}

int Intersection_Points_and_Grid_Estimation(std::string, std::string);
std::vector<int> point_density (pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ, pcl::PointXYZ);
std::vector<pcl::PointXYZ> get_grid(pcl::PointXYZ, pcl::PointXYZ);

std::ofstream log_file;
std::ifstream vehicle_file;
Eigen::Matrix4f Vehicle_transform,vehicle_trans;

int main (int argc, char** argv)

{
    dataset_folder =argv[1];
    Map = argv[2];
    Vehicle_PCDs_Path=argv[3];
    Intersection_Points_and_Grid_Estimation(dataset_folder,Map);

    return (0);
}

int Intersection_Points_and_Grid_Estimation(std::string dataset_folder, std::string Map)
{

    // ******************Map reading*********************** 

    std::string Map_path = dataset_folder +"/"+ Map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (Map_path, *Map_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file  \n");
        return (-1);
    }

    // ******************Intersection Extraction***********************  

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(Map_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(xmin, xmax);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
    pass_x.filter(*cloud_filtered_x);
    pcl::PassThrough<pcl::PointXYZ> pass_z_x;
    pass_z_x.setInputCloud(cloud_filtered_x);
    pass_z_x.setFilterFieldName("z");
    pass_z_x.setFilterLimits(-0.1, 4);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>());
    pass_z_x.filter(*cloud_filtered_z);
    pcl::PassThrough<pcl::PointXYZ> pass_z_x_y;
    pass_z_x_y.setInputCloud(cloud_filtered_z);
    pass_z_x_y.setFilterFieldName("y");
    pass_z_x_y.setFilterLimits(-100, 100);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xz(new pcl::PointCloud<pcl::PointXYZ>());
    pass_z_x_y.filter(*cloud_filtered_xz);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(Map_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(ymin, ymax);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_yz(new pcl::PointCloud<pcl::PointXYZ>());
    pass_y.filter(*cloud_filtered_yz);
    pcl::PassThrough<pcl::PointXYZ> pass_z_y;
    pass_z_y.setInputCloud(cloud_filtered_yz);
    pass_z_y.setFilterFieldName("z");
    pass_z_y.setFilterLimits(-0.1, 4);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_map_xz(new pcl::PointCloud<pcl::PointXYZ>());
    pass_z_y.filter(*cloud_filtered_map_xz);
    pcl::PassThrough<pcl::PointXYZ> pass_y_y;
    pass_y_y.setInputCloud(cloud_filtered_map_xz);
    pass_y_y.setFilterFieldName("x");
    pass_y_y.setFilterLimits(-150, 50);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_map_x(new pcl::PointCloud<pcl::PointXYZ>());
    pass_y_y.filter(*cloud_filtered_map_x);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Combined(new pcl::PointCloud<pcl::PointXYZ>());
    *Combined=(*cloud_filtered_map_x)+(*cloud_filtered_xz);

                // Combined->width = Combined->points.size();  // Set width to the number of points
                // Combined->height = 1; 
                // pcl::io::savePCDFileASCII("Map.pcd", *Combined);

// ******************Grid Estimation for the Intersection***********************  

    pcl::PointXYZ minPt_vehicle, maxPt_vehicle;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*Combined, minPt_vehicle, maxPt_vehicle);

    minPt.x =  minPt_vehicle.x;
    minPt.y =  minPt_vehicle.y;
    minPt.z =  minPt_vehicle.z;

    maxPt.x =  maxPt_vehicle.x;
    maxPt.y =  maxPt_vehicle.y;
    maxPt.z =  maxPt_vehicle.z;

    std::vector<pcl::PointXYZ> grid_info = get_grid(minPt, maxPt);
    std::vector<pcl::PointXYZ> centroids = get_grid_centroids(grid_info);


                // std::cout << "Number of centroids: " << centroids.size() << std::endl;

                // pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                // for (const auto& centroid : centroids) {
                //     centroid_cloud->points.push_back(centroid);
                // }
                // centroid_cloud->width = centroid_cloud->points.size();
                // centroid_cloud->height = 1;  
                // centroid_cloud->is_dense = true;
                
                // if (pcl::io::savePCDFileASCII("Grid.pcd", *centroid_cloud) == -1) {
                //     PCL_ERROR("Failed to save point cloud.");
                //     return -1;
                // }

                // std::cout << "Saved " << centroid_cloud->points.size() << " grid centroids to " << filename << std::endl;

                
// ******************Vehicle Drivable Space Estimation***********************  

    std::string Vehicle_path = dataset_folder +"/"+ Vehicle_PCDs_Path +"/"+"100.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr Vehicle_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (Vehicle_path, *Vehicle_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file  \n");
        return (-1);
    }

    pcl::PassThrough<pcl::PointXYZ> v_x_pass_x;
    pcl::PointCloud<pcl::PointXYZ>::Ptr v_x_cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> v_x_pass_z_x;
    pcl::PointCloud<pcl::PointXYZ>::Ptr v_x_cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> v_x_pass_z_x_y;
    pcl::PointCloud<pcl::PointXYZ>::Ptr v_x_cloud_filtered_xz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> v_x_pass_y;
    pcl::PointCloud<pcl::PointXYZ>::Ptr v_x_cloud_filtered_yz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> v_x_pass_z_y;
    pcl::PointCloud<pcl::PointXYZ>::Ptr v_x_cloud_filtered_map_xz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> v_x_pass_y_y;
    pcl::PointCloud<pcl::PointXYZ>::Ptr v_x_cloud_filtered_map_x(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr v_x_Combined(new pcl::PointCloud<pcl::PointXYZ>());

    v_x_pass_x.setInputCloud(Vehicle_cloud);
    v_x_pass_x.setFilterFieldName("x");
    v_x_pass_x.setFilterLimits(xmin, xmax);

    v_x_pass_z_x.setInputCloud(v_x_cloud_filtered_x);
    v_x_pass_z_x.setFilterFieldName("z");
    v_x_pass_z_x.setFilterLimits(-0.1,4);

    v_x_pass_z_x_y.setInputCloud(v_x_cloud_filtered_z);
    v_x_pass_z_x_y.setFilterFieldName("y");
    v_x_pass_z_x_y.setFilterLimits(-100, 100);

    v_x_pass_y.setInputCloud(Vehicle_cloud);
    v_x_pass_y.setFilterFieldName("y");
    v_x_pass_y.setFilterLimits(ymin, ymax);

    v_x_pass_z_y.setInputCloud(v_x_cloud_filtered_yz);
    v_x_pass_z_y.setFilterFieldName("z");
    v_x_pass_z_y.setFilterLimits(-0.1, 4);

    v_x_pass_y_y.setInputCloud(v_x_cloud_filtered_map_xz);
    v_x_pass_y_y.setFilterFieldName("x");
    v_x_pass_y_y.setFilterLimits(-150, 50);

    run_time_start = clock();
    v_x_pass_x.filter(*v_x_cloud_filtered_x);
    v_x_pass_z_x.filter(*v_x_cloud_filtered_z);
    v_x_pass_z_x_y.filter(*v_x_cloud_filtered_xz);
    run_time_end = clock();

    run_time_start1 = clock();
    v_x_pass_y.filter(*v_x_cloud_filtered_yz);
    v_x_pass_z_y.filter(*v_x_cloud_filtered_map_xz);
    v_x_pass_y_y.filter(*v_x_cloud_filtered_map_x);
    run_time_end1 = clock();
    time1 = max((run_time_end - run_time_start),(run_time_end1 - run_time_start1));
    time1 = (((float)(time1)/CLOCKS_PER_SEC)*1000);
    accumulate_time=accumulate_time+time1;
    // cout<<"Time1: "<<(((float)(time1)/CLOCKS_PER_SEC)*1000)<<endl;
    run_time_start = clock();
    v_x_cloud_filtered_map_x->reserve(v_x_cloud_filtered_map_x->size() + v_x_cloud_filtered_xz->size());
    (*v_x_cloud_filtered_map_x) += (*v_x_cloud_filtered_xz);
    run_time_end = clock();
    time2 = run_time_end - run_time_start;
    time2 = (((float)(time2)/CLOCKS_PER_SEC)*1000);
    accumulate_time=accumulate_time+time2;
    // cout<<"Road Filter Time: "<<accumulate_time<<endl;
    Vehicle_cloud = v_x_cloud_filtered_map_x;


                // Vehicle_cloud->width =  Vehicle_cloud->points.size();  // Set width to the number of points
                //  Vehicle_cloud->height = 1; 
                // pcl::io::savePCDFileASCII("Vehicle_DS.pcd", * Vehicle_cloud);

// ******************Dynamic Objects Extraction*********************** 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>()); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr changes(new pcl::PointCloud<pcl::PointXYZ>());      
    // PointCloudT::Ptr changes(new PointCloudT);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(Combined);
    voxel_grid.setLeafSize(0.5f, 0.5f, 0.5f);  // Set the leaf size (voxel resolution)

    // Apply the filter
    voxel_grid.filter(*cloud_downsampled);

    double distance_threshold = 1.0; 
    detectChangesCUDA(cloud_downsampled, Vehicle_cloud, distance_threshold, changes);
    accumulate_time=accumulate_time+time3;

    std::vector<pcl::PointXYZ> positions_list = { 
    pcl::PointXYZ(-4.369153594970703125e+01,5.486972427368164062e+01,2.188285589218139648e+00),
    pcl::PointXYZ(-4.717868423461914062e+01, 5.212943649291992188e+01, 2.414839744567871094e+00),
    pcl::PointXYZ(-4.704954910278320312e+01, 3.341448593139648438e+01, 2.325795888900756836e+00),
    pcl::PointXYZ(-4.354092025756835938e+01,3.187811279296875000e+01,2.444107055664062500e+00),
    pcl::PointXYZ(-4.713151168823242188e+01, 1.282591152191162109e+01, 2.511302709579467773e+00),
    pcl::PointXYZ(-4.356395721435546875e+01, 1.832020187377929688e+01, 2.048950433731079102e+00),
    pcl::PointXYZ(-6.024911880493164062e+00,-9.418421983718872070e-01,1.994088411331176758e+00),
    pcl::PointXYZ(-4.311421871185302734e+00, -4.439491748809814453e+00, 2.536698341369628906e+00),
    pcl::PointXYZ(-2.126921272277832031e+01, -4.398709774017333984e+00, 2.081127166748046875e+00),
    pcl::PointXYZ(-2.315992546081542969e+01,-8.944885730743408203e-01,2.210667371749877930e+00),
    pcl::PointXYZ(-3.916044235229492188e+01, -4.354568004608154297e+00, 2.211443901062011719e+00),
    pcl::PointXYZ(-4.262006759643554688e+01, -8.472048044204711914e-01, 2.386896371841430664e+00),
    pcl::PointXYZ(-5.478238296508789062e+01,-5.361889648437500000e+01,2.414839506149291992e+00),
    pcl::PointXYZ(-5.128505325317382812e+01, -5.410670471191406250e+01, 2.188285350799560547e+00),
    pcl::PointXYZ(-5.469423294067382812e+01, -3.869682693481445312e+01, 2.699539661407470703e+00),
    pcl::PointXYZ(-5.118546295166015625e+01,-3.745829772949218750e+01,2.472491502761840820e+00),
    pcl::PointXYZ(-5.457848358154296875e+01, -1.924626541137695312e+01, 2.304422140121459961e+00),
    pcl::PointXYZ(-5.105447006225585938e+01, -1.573933696746826172e+01,  2.254399776458740234e+00),
    pcl::PointXYZ(-5.435709381103515625e+01,-4.922786951065063477e-01,2.417370319366455078e+00),
    pcl::PointXYZ(-9.675198364257812500e+01,2.790080547332763672e+00, 1.993329167366027832e+00),
    pcl::PointXYZ(-9.363206481933593750e+01, 6.277460098266601562e+00, 2.875283241271972656e+00),
    pcl::PointXYZ(-7.565912628173828125e+01,2.732823371887207031e+00,2.276685953140258789e+00),
    pcl::PointXYZ(-7.335550689697265625e+01,  6.227827548980712891e+00, 2.358937501907348633e+00),
    pcl::PointXYZ(-6.157320022583007812e+01, 2.699161529541015625e+00, 2.644088983535766602e+00),
    pcl::PointXYZ(-5.764266586303710938e+01, 6.189242362976074219e+00, 2.214265584945678711e+00)
};
int cloud_size = changes->points.size();
int positions_size = positions_list.size();

// Allocate memory on the device
pcl::PointXYZ* d_cloud_points;
pcl::PointXYZ* d_positions;
int* d_assignments;


cudaMalloc((void**)&d_cloud_points, cloud_size * sizeof(pcl::PointXYZ));
cudaMalloc((void**)&d_positions, positions_size * sizeof(pcl::PointXYZ));
cudaMalloc((void**)&d_assignments, cloud_size * sizeof(int));

// Copy data to device
cudaMemcpy(d_cloud_points, changes->points.data(), cloud_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
cudaMemcpy(d_positions, positions_list.data(), positions_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

// Launch CUDA kernel to assign points to positions
int threads_per_block = 256;
int num_blocks = (cloud_size + threads_per_block - 1) / threads_per_block;
run_time_start = clock();
assignPointsToPositions<<<num_blocks, threads_per_block>>>(d_cloud_points, cloud_size, d_positions, positions_size, d_assignments);
cudaDeviceSynchronize();
run_time_end = clock();
time4 = run_time_end - run_time_start;
time4 = (((float)(time4)/CLOCKS_PER_SEC)*1000);
accumulate_time = accumulate_time + time4;

std::cout << "Accumulated Time: " << accumulate_time << std::endl;
std::cout << "Road Filter Time: " << time1 + time2 << std::endl;
std::cout << "Diff Time: " << time3 << std::endl;
std::cout << "Vehicle Points Assignment Time: " << time4 << std::endl;

// Check for kernel execution errors
cudaError_t error = cudaGetLastError();
if (error != cudaSuccess) {
    std::cerr << "CUDA kernel failed: " << cudaGetErrorString(error) << std::endl;
    return -1;
}

// Copy the assignments back to host
std::vector<int> assignments(cloud_size);
cudaMemcpy(assignments.data(), d_assignments, cloud_size * sizeof(int), cudaMemcpyDeviceToHost);

// Free device memory
cudaFree(d_cloud_points);
cudaFree(d_positions);
cudaFree(d_assignments);

// Initialize a vector to store the count of points assigned to each position
std::vector<int> position_counts(positions_size, 0);  // Initialize with 0

// Count how many points are assigned to each position
for (int i = 0; i < cloud_size; ++i) {
    int pos_idx = assignments[i];  // Get the assigned position index for point i
    if (pos_idx >= 0 && pos_idx < positions_size) {  // Ensure valid position index
        position_counts[pos_idx]++;  // Increment the count for this position
    } else {
        std::cerr << "Invalid position index: " << pos_idx << std::endl;
    }
}

// Print the number of points assigned to each position
for (size_t i = 0; i < positions_size; ++i) {
    std::cout << "Position " << i << " has " << position_counts[i] << " points assigned." << std::endl;
}


    // accumulate_time=accumulate_time+time3;
    // cout<<"Accumulated Time: "<<accumulate_time<<endl;
    

    // pcl::PointXYZ min_centroid = grid_info[0];
    // pcl::PointXYZ max_centroid = grid_info[1];
    // pcl::PointXYZ lims = grid_info[2];

    // int gridSize = lims.x*lims.y*lims.z;

    // std::vector<int> result;

    // std::vector<int> vehicle_pd(gridSize, 0);
    // std::vector<bool> ipd(gridSize, 0); 

    // int cloudSize = vehicle_cloud->size();
    // result.resize(cloudSize);

    // pcl::PointXYZ *h_points = vehicle_cloud->points.data();  
    // pcl::PointXYZ *d_points;

    
    // cudaMalloc((void**)&d_points, cloudSize * sizeof(pcl::PointXYZ));
    // cudaMemcpy(d_points, h_points, cloudSize * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

    // int *res_ptr = result.data();
    // int *d_res_ptr;

    // cudaMalloc((void**)&d_res_ptr, cloudSize * sizeof(int));
    // cudaMemcpy(d_res_ptr, res_ptr, cloudSize * sizeof(int), cudaMemcpyHostToDevice);

    // int numBlocks = (cloudSize + threadsPerBlock - 1) / threadsPerBlock;

    // float run_time_start = clock();
    // getNN<<<numBlocks, threadsPerBlock>>>(cloudSize, d_points, d_res_ptr, min_centroid.x, min_centroid.y, min_centroid.z, lims.x, lims.y, lims.z);
    // cudaDeviceSynchronize();
    // float run_time_end = clock();

    // time1 = run_time_end - run_time_start;

    // cudaMemcpy(res_ptr, d_res_ptr, cloudSize * sizeof(int), cudaMemcpyDeviceToHost);

    // cudaFree(d_points);
    // cudaFree(d_res_ptr);
        
        
    // int cloudSize2 = result.size();
    // int gridSize2 = vehicle_pd.size();

    // int* d_result2 = nullptr;
    // int* d_vehicle_pd = nullptr;
    // cudaMalloc(&d_result2, cloudSize2 * sizeof(int));
    // cudaMalloc(&d_vehicle_pd, gridSize2 * sizeof(int));

    // cudaMemcpy(d_result2, result.data(), cloudSize2 * sizeof(int), cudaMemcpyHostToDevice);
    // cudaMemcpy(d_vehicle_pd, vehicle_pd.data(), gridSize2 * sizeof(int), cudaMemcpyHostToDevice);

    // int threadsPerBlock = 256;
    // int blocksPerGrid = (cloudSize2 + threadsPerBlock - 1) / threadsPerBlock;
    // float run_time_start = clock();
    // incrementKernel<<<blocksPerGrid, threadsPerBlock>>>(d_result2, d_vehicle_pd, cloudSize2, gridSize2);
    // cudaDeviceSynchronize();
    // cudaCheckError();
    // float run_time_end = clock();
    // cudaMemcpy(vehicle_pd.data(), d_vehicle_pd, gridSize2 * sizeof(int), cudaMemcpyDeviceToHost);

    // cudaFree(d_result2);
    // cudaFree(d_vehicle_pd);
        
    // time2 = run_time_end - run_time_start;
    // float accumulate_time=accumulate_time+(((float)(time1+time2+time3)/CLOCKS_PER_SEC)*1000);
    // cout<<"Accumulated Time: "<<accumulate_time<<endl;

     

//     int centroids_size = centroids.size();
//     int vehicle_pd_size = vehicle_pd.size();
//     int positions_size = positions_list.size();

//     std::vector<BoundingBox> bounding_boxes1(positions_size);
//     for (auto& box : bounding_boxes1) {
//         box.minX = std::numeric_limits<float>::max();
//         box.minY = std::numeric_limits<float>::max();
//         box.maxX = std::numeric_limits<float>::lowest();
//         box.maxY = std::numeric_limits<float>::lowest();
//     }

//     pcl::PointXYZ* d_centroids = nullptr;
//     int* d_vehicle_pd = nullptr;
//     pcl::PointXYZ* d_positions_list = nullptr;
//     BoundingBox* d_bounding_boxes = nullptr;
//     int* d_cells_count = nullptr;

//     cudaMalloc(&d_centroids, centroids_size * sizeof(pcl::PointXYZ));
//     cudaMalloc(&d_vehicle_pd, vehicle_pd_size * sizeof(int));
//     cudaMalloc(&d_positions_list, positions_size * sizeof(pcl::PointXYZ));
//     cudaMalloc(&d_bounding_boxes, positions_size * sizeof(BoundingBox)); // No memset needed here
//     cudaMalloc(&d_cells_count, positions_size * sizeof(int));

//     // Copy data to device
//     cudaMemcpy(d_centroids, centroids.data(), centroids_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
//     cudaMemcpy(d_vehicle_pd, vehicle_pd.data(), vehicle_pd_size * sizeof(int), cudaMemcpyHostToDevice);
//     cudaMemcpy(d_positions_list, positions_list.data(), positions_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
//     cudaMemcpy(d_bounding_boxes, bounding_boxes1.data(), positions_size * sizeof(BoundingBox), cudaMemcpyHostToDevice); // Copy initialized bounding boxes
//     cudaMemset(d_cells_count, 0, positions_size * sizeof(int)); // initialize counts to 0

//     // Set up kernel launch parameters
//     int threadsPerBlock = 256;
//     int blocksPerGrid = (vehicle_pd_size + threadsPerBlock - 1) / threadsPerBlock;
//     run_time_start = clock();
//     // Launch kernel
//     updateBoundingBoxesKernel<<<blocksPerGrid, threadsPerBlock>>>(
//         d_centroids,
//         d_vehicle_pd,
//         d_positions_list,
//         d_bounding_boxes,
//         d_cells_count,
//         vehicle_pd_size,
//         positions_size,
//         cell_x_size,
//         cell_y_size
//     );

//     // Wait for kernel to complete and copy results back
//     cudaDeviceSynchronize();
//     run_time_end = clock();
//     time3 = run_time_end - run_time_start;
//     std::vector<BoundingBox> bounding_boxes(positions_size);
//     std::vector<int> cells_count(positions_size);
//     cudaMemcpy(bounding_boxes.data(), d_bounding_boxes, positions_size * sizeof(BoundingBox), cudaMemcpyDeviceToHost);
//     cudaMemcpy(cells_count.data(), d_cells_count, positions_size * sizeof(int), cudaMemcpyDeviceToHost);

//     // Clean up device memory
//     cudaFree(d_centroids);
//     cudaFree(d_vehicle_pd);
//     cudaFree(d_positions_list);
//     cudaFree(d_bounding_boxes);
//     cudaFree(d_cells_count);
//     std::string filename2;
//     size_t filename1;
//     filename1 = current_frame_name.find_last_of(".");
//     filename2 = current_frame_name.substr(0, filename1);
//     filename2=dataset_folder+"/"+filename2+".txt";
//     cout<<"File Name: "<<filename2<<endl;
//     log_file.open(filename2);
//     for (int j = 0; j < positions_list.size(); ++j) {
//         std::cout << "Number of cells assigned to vehicle " << j + 1 << ": " << cells_count[j] << std::endl;
//         if (cells_count[j]!= 0)
//         {
//         log_file << positions_list[j] << " " <<bounding_boxes[j].minX<<" "<<bounding_boxes[j].minY<<" "<<bounding_boxes[j].maxX <<" " <<bounding_boxes[j].maxY<<endl;
//     //     std::cout << "minX: " << bounding_boxes[j].minX 
//     //             << ", minY: " << bounding_boxes[j].minY 
//     //             << ", maxX: " << bounding_boxes[j].maxX 
//     //             << ", maxY: " << bounding_boxes[j].maxY << std::endl;
//         }
//     }
//     log_file.close();

// accumulate_time=accumulate_time+(((float)(time1+time2+time3)/CLOCKS_PER_SEC)*1000);
// // std::cout << "BlindSpots Latency: " <<accumulate_time<< std::endl; 
// // log_file <<current_frame_name <<"," <<  ((float)(time1+time2+time3+time4+time5+time6)/CLOCKS_PER_SEC)*1000<<"," <<0<<std::endl;
// positions_list.clear();
return 0;
}


std::vector<pcl::PointXYZ> get_grid( pcl::PointXYZ min, pcl::PointXYZ max )
{
    std::vector<pcl::PointXYZ> ret;
    ret.reserve(3); 

    pcl::PointXYZ min_centroid;
    min_centroid.x = min.x + (cell_x_size/2);
    min_centroid.y = min.y + (cell_y_size/2);
    min_centroid.z = min.z + (cell_z_size/2);

    pcl::PointXYZ max_centroid;

    int t_x = int((max.x - min.x) / cell_x_size);
    int t_y = int((max.y - min.y) / cell_y_size);
    int t_z = int((max.z - min.z) / cell_z_size);

    max_centroid.x = ( (min.x + (t_x-1)*cell_x_size) + (min.x + (t_x)*cell_x_size) ) / 2;
    max_centroid.y = ( (min.y + (t_y-1)*cell_y_size) + (min.y + (t_y)*cell_y_size) ) / 2;
    max_centroid.z = ( (min.z + (t_z-1)*cell_z_size) + (min.z + (t_z)*cell_z_size) ) / 2;

    pcl::PointXYZ lims;
    lims.x = t_x+1;
    lims.y = t_y+1;
    lims.z = t_z+1;
    
    ret[0] = min_centroid;
    ret[1] = max_centroid;
    ret[2] = lims;

    return ret;
}

__device__ inline int getGridCentroidIdx( int x, int y, int z, int xSz, int ySz, int zSz ) {
    return x + y * xSz + z * xSz * ySz;
}

__device__ float computeDist ( float x1, float x2 ) {
    return (x1-x2)*(x1-x2);
}

float computeDist1 ( float x1, float x2 ) {
    return (x1-x2)*(x1-x2);
}

__device__ float computeDist ( float x1, float y1, float z1, float x2, float y2, float z2 ) {
    return computeDist( x1, x2 ) + computeDist( y1, y2 ) + computeDist( z1, z2 );
}


__device__ float computeDist ( pcl::PointXYZ p1, pcl::PointXYZ p2 ) {
    return computeDist( p1.x, p1.y, p1.z, p2.x, p2.y, p2.z );
}

__device__ inline bool isWithinBounds( int x, int y, int z, int lims_x, int lims_y, int lims_z ) {
    if ( x < 0 ) return false;
    if ( y < 0 ) return false;
    if ( z < 0 ) return false;
    if ( x >= lims_x ) return false;
    if ( y >= lims_y ) return false;
    if ( z >= lims_z ) return false;
    return true;
}

inline bool isWithinBoundsHost( pcl::PointXYZ p, pcl::PointXYZ min, pcl::PointXYZ max ) {
    if ( p.x < min.x ) return false;
    if ( p.y < min.y ) return false;
    if ( p.z < min.z ) return false;
    if ( p.x > max.x ) return false;
    if ( p.y > max.y ) return false;
    if ( p.z > max.z ) return false;
    return true;
}
        
