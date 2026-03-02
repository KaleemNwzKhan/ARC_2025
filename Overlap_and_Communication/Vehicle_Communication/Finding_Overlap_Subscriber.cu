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
#include <pcl/filters/extract_indices.h>
using namespace std;

//**************************************************Global Variables****************************************************************************************************
int number_of_vehicles=44;
int number_of_frames=100;
int Intersection_Points_and_Grid_Estimation();
std::vector<int> point_density (pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ, pcl::PointXYZ);
std::vector<pcl::PointXYZ> get_grid(pcl::PointXYZ, pcl::PointXYZ);
std::vector<std::vector<std::tuple<float, float, float>>> lists(number_of_vehicles, std::vector<std::tuple<float, float, float>>(number_of_frames));
std::vector<std::vector<Eigen::Matrix4f>> World_Transforms(2, std::vector<Eigen::Matrix4f>(number_of_frames));
std::ofstream log_file,BB_file;
std::ifstream vehicle_file,leader_file,Transform_file;
Eigen::Matrix4f Vehicle_transform,vehicle_trans;
int ipd_threshold = 10;



std::string dataset_folder, Map, Vehicle_PCDs_Path, Leader_PCD_Path,Leader_transform_file, Vehicle_transform_file, All_Vehicles_Transforms_Folder, Output_file;
float run_time_start,run_time_end,run_time_start1,run_time_end1;
float time1,time2, time3, time3_1, time4, time5, time6, time7, time8, time9, time10;
const int threadsPerBlock = 256;
const float cell_x_size = 2.5;
const float cell_y_size = 2.5;
const float cell_z_size = 4.0;
std::string filename = "grid_centroids.pcd";
std::vector<pcl::PointXYZ> positions_list;

float xmin = -57.0;
float xmax = -42.0;
float ymin = -6.8;
float ymax = 9.0;
std::string decide="";


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
        

//**************************************************CUDA Change Detection Part****************************************************************************************************

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

void detectChangesCUDA(const PointCloudT::Ptr& cloud1, const PointCloudT::Ptr& cloud2, const PointCloudT::Ptr& change_PC,
                       double threshold) {
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
    nearestNeighborSearch<<<numBlocks, blockSize, sharedMemSize>>>(d_cloud2, d_cloud1, cloud2_size, cloud1_size, threshold, d_change_indices, d_change_count);
    cudaDeviceSynchronize();
    run_time_end = clock();
    time3 = ((float(run_time_end - run_time_start) / CLOCKS_PER_SEC) * 1000);

    int change_count;
    cudaMemcpy(&change_count, d_change_count, sizeof(int), cudaMemcpyDeviceToHost);
    std::vector<int> change_indices(change_count);
    cudaMemcpy(change_indices.data(), d_change_indices, change_count * sizeof(int), cudaMemcpyDeviceToHost);

    // Now update the 'cloud2' point cloud with only the detected changes
    // cloud2->clear();  // Clear all points in cloud2 to store only the changes

    for (int i = 0; i < change_count; ++i) {
        // Add the changed points to cloud2
        change_PC->points.push_back(cloud2->points[change_indices[i]]);
    }

    // Free device memory
    cudaFree(d_cloud1);
    cudaFree(d_cloud2);
    cudaFree(d_change_indices);
    cudaFree(d_change_count);
}

//**************************************************CUDA Assignment Dynamic Points to Vehicle Positions*************************************************************

__global__ void assignPointsToPositions(pcl::PointXYZ* cloud_points, int cloud_size, pcl::PointXYZ* positions, int positions_size, int* assignments) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < cloud_size) {
       pcl::PointXYZ  point = cloud_points[idx];
        float min_distance = FLT_MAX;
        int nearest_position = -1;

        // Loop over each position to find the nearest one
        for (int i = 0; i < positions_size; ++i) {
            // Access position directly from memory
            float distance = (point.x - positions[i].x) * (point.x - positions[i].x) +
                             (point.y - positions[i].y) * (point.y - positions[i].y);

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

//**************************************************CUDA BlindSpots to Vehicles Assignment Part *************************************************************
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
    BoundingBox* bounding_boxes,  // Flat array for all bounding boxes
    int* bounding_box_offsets,   // Offset for each position in the flat array
    int* bounding_box_counts,    // Counter for bounding boxes of each position
    int vehicle_size,
    int positions_size,
    float cell_x_size,
    float cell_y_size,
    int max_bounding_boxes_per_position) {  // Upper limit to pre-allocate memory

    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= vehicle_size || vehicle_pd[i] != 0) return;

    pcl::PointXYZ centroid = centroids[i];
    float min_dist = FLT_MAX;
    int closest_position_idx = -1;

    // Find the closest position
    for (int pos_idx = 0; pos_idx < positions_size; ++pos_idx) {
        float dist = computeDist1(centroid.x, centroid.y, centroid.z,
                                  positions_list[pos_idx].x, positions_list[pos_idx].y, positions_list[pos_idx].z);
        if (dist < min_dist) {
            min_dist = dist;
            closest_position_idx = pos_idx;
        }
    }

    if (closest_position_idx != -1) {
        // Create a bounding box for the current cell
        CellInfo cell_info(centroid.x, centroid.y, centroid.z, cell_x_size, cell_y_size);
        BoundingBox box = {cell_info.minX, cell_info.minY, cell_info.maxX, cell_info.maxY};

        // Compute index for the new bounding box
        int position_offset = bounding_box_offsets[closest_position_idx];
        int index = atomicAdd(&bounding_box_counts[closest_position_idx], 1);

        if (index < max_bounding_boxes_per_position) {
            bounding_boxes[position_offset + index] = box;
        }
    }
}

//*************************************************************CUDA Counting High Point Density Cells*************************************************************

__global__ void incrementKernel(int* d_result2, int* d_vehicle_pd, int cloudSize, int gridSize) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < cloudSize) {
        int bin = d_result2[idx];
        if (bin < gridSize) { 
            atomicAdd(&d_vehicle_pd[bin], 1);
        }
    }
}


//**************************************************Centroids Assignment to the Points**************************************************


                        //***********************For filtered grid centroids (only drivable space)************************

__global__ void getNN_modified(int N, pcl::PointXYZ *points, int *res, pcl::PointXYZ *centroids, int num_centroids) {
    size_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid < N) {
        float curr_x = points[tid].x;
        float curr_y = points[tid].y;
        float curr_z = points[tid].z;

        float min_dist_to_centroid = FLT_MAX;  // Initialize with a large value
        int closest_centroid_idx = -1;

        // Iterate over all centroids
        for (int i = 0; i < num_centroids; i++) {
            float centroid_x = centroids[i].x;
            float centroid_y = centroids[i].y;
            float centroid_z = centroids[i].z;

            // Compute squared Euclidean distance
            float curr_dist = (curr_x - centroid_x) * (curr_x - centroid_x) +
                              (curr_y - centroid_y) * (curr_y - centroid_y) +
                              (curr_z - centroid_z) * (curr_z - centroid_z);

            // Update closest centroid if a smaller distance is found
            if (curr_dist < min_dist_to_centroid) {
                min_dist_to_centroid = curr_dist;
                closest_centroid_idx = i;
            }
        }

        // Store the index of the closest centroid
        res[tid] = closest_centroid_idx;
    }
}

                        //***********************Whole grid centroids************************

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

//*****************************************Centroids of the Grid**************************************************t*************************************************************
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

//*****************************************CUDA Finding Overlapping Region***************************************************************************************************************

// Kernel to find indices above the threshold
__global__ void findAboveThreshold(const int* leader_pd, const int* vehicle_pd, 
                                   int* indices_above_threshold, int* above_threshold, 
                                   int leader_size, int vehicle_size, int ipd_threshold) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < vehicle_size) {
        // Calculate the minimum pd value between infrastructure and vehicle
        int min_pd = min(leader_pd[idx], vehicle_pd[idx]);

        if (min_pd > ipd_threshold) {
            // Atomically increment the count of above-threshold indices
            int index = atomicAdd(above_threshold, 1);
            indices_above_threshold[index] = idx; // Store the index
        }
    }
}

// Kernel to populate final clouds for both vehicle and leader points based on threshold indices
__global__ void populateFinalCloud(const pcl::PointXYZ* vehicle_points, const pcl::PointXYZ* leader_points,
                                    const int* result2, const int* result1, const int* indices_above_threshold, int above_threshold, 
                                    pcl::PointXYZ* final_vehicle_points, pcl::PointXYZ* final_leader_points, 
                                    int* final_count_vehicle, int* final_count_leader, int vehicle_size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < vehicle_size) {
        for (int i = 0; i < above_threshold; ++i) {
            if (result2[idx] == indices_above_threshold[i] ) {
                int count_idx_vehicle = atomicAdd(final_count_vehicle, 1);
                final_vehicle_points[count_idx_vehicle] = vehicle_points[idx];
            }
             if (result1[idx] == indices_above_threshold[i] ) {
                int count_idx_leader = atomicAdd(final_count_leader, 1);
                final_leader_points[count_idx_leader] = leader_points[idx];
            }
        }
    }
}

//*****************************************Reading Transforms from a File and Applying to Point Clouds***************************************************************************************************************

std::vector<std::string> splitString(std::string str, char splitter){
    std::vector<std::string> result;
    std::string current = "";
    for(int i = 0; i < str.size(); i++){
        if(str[i] == splitter){
            if(current != ""){
                result.push_back(current);
                current = "";
            }
            continue;
        }
        current += str[i];
    }
    if(current.size() != 0)
        result.push_back(current);
    return result;
}

Eigen::Matrix4f Get_Transform()
{
  std::string line="";
  Eigen::Matrix4f init_guess;
  Eigen::Matrix4f temp;
  int k=0;
  //std::cout<<"Hello Hello"<<std::endl;
  for (int i=0;i<4;i++)
  {
    getline(Transform_file,line);
    //cout<<"Kaleem : "<<line<<endl;
    std::vector<std::string> result = splitString(line, ' ');
    std::vector<std::string> result_last = splitString(result[3],'\n');
    for (int j=0;j<3;j++)
    {
      init_guess(k)=std::stof(result[j]);
      k++;
    }
    init_guess(k)=std::stof(result_last[0]);
    k++;
  }
  temp=init_guess.transpose();
  init_guess=temp;
  return init_guess;
}

// void applyTransformations(pcl::PointCloud<pcl::PointXYZ>::Ptr& leader_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& vehicle_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& vehicle_cloud_temp) {
//     Eigen::Matrix4f leader_trans = Get_Transform(leader_file);
//     Eigen::Matrix4f vehicle_trans = Get_Transform(vehicle_file);
//         pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_leader_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_vehicle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_vehicle_cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::transformPointCloud(*leader_cloud, *leader_cloud, leader_trans);
//         pcl::transformPointCloud(*vehicle_cloud, *vehicle_cloud, vehicle_trans);
//         pcl::transformPointCloud(*vehicle_cloud_temp, *vehicle_cloud_temp, vehicle_trans);
//         std::cout << "Applied transformation to both leader and vehicle clouds.\n";
// }

void Fill_transforms() {
    Transform_file.open(Leader_transform_file);
    for (int i=0;i<number_of_frames;i++)
    {
       Eigen::Matrix4f lead_trans=Get_Transform();
       World_Transforms[0][i]=lead_trans;
    //    cout<<"Leader: "<<World_Transforms[0][i]<<endl;
    }
   Transform_file.close();

    Transform_file.open(Vehicle_transform_file);
    for (int i=0;i<number_of_frames;i++)
    {
       Eigen::Matrix4f veh_trans=Get_Transform();
       World_Transforms[1][i]=veh_trans;
    //    cout<<"Vehicle: "<<World_Transforms[1][i]<<endl;
    }
   Transform_file.close();
   
}


//*****************************************Host Part of CUDA Intersection_Points_and_Grid_Estimation***************************************************************************************************************


int Intersection_Points_and_Grid_Estimation()
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

    int Map_Points_Size = Combined->points.size();
    int Centroids_size = centroids.size();

    // Allocate memory on the device
    pcl::PointXYZ* d_Map_points;
    pcl::PointXYZ* d_Centroids;
    int* d_Centroids_assignments;
    cudaMalloc((void**)&d_Map_points, Map_Points_Size * sizeof(pcl::PointXYZ));
    cudaMalloc((void**)&d_Centroids, Centroids_size * sizeof(pcl::PointXYZ));
    cudaMalloc((void**)&d_Centroids_assignments, Map_Points_Size * sizeof(int));

    // Copy data to device
    cudaMemcpy(d_Map_points, Combined->points.data(), Map_Points_Size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
    cudaMemcpy(d_Centroids, centroids.data(), Centroids_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

    // Launch CUDA kernel to assign points to positions
    int threads_per_block = 256;
    int num_blocks = (Map_Points_Size + threads_per_block - 1) / threads_per_block;
    assignPointsToPositions<<<num_blocks, threads_per_block>>>(d_Map_points, Map_Points_Size,d_Centroids, Centroids_size, d_Centroids_assignments);
    cudaDeviceSynchronize();

    // Check for kernel execution errors
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
        std::cerr << "CUDA kernel failed: " << cudaGetErrorString(error) << std::endl;
        return -1;
    }

    // Copy the assignments back to host
    std::vector<int> assignments_Map(Map_Points_Size);
    cudaMemcpy(assignments_Map.data(), d_Centroids_assignments, Map_Points_Size* sizeof(int), cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree(d_Map_points);
    cudaFree(d_Centroids);
    cudaFree(d_Centroids_assignments);

    // Step 2: Remove positions with no points assigned
    std::vector<bool> position_has_points(Centroids_size, false);
    
    // Mark positions that have at least one point assigned
    for (int i = 0; i < Map_Points_Size; ++i) {
        int assigned_position = assignments_Map[i];
        if (assigned_position >= 0 && assigned_position < Centroids_size) {
            position_has_points[assigned_position] = true;
        }
    }

    // Create a new filtered list of positions that have points assigned
    std::vector<pcl::PointXYZ> filtered_positions;
    for (int i = 0; i < Centroids_size; ++i) {
        if (position_has_points[i]) {
            filtered_positions.push_back(centroids[i]);
        }
    }
    centroids=filtered_positions;

//                 // std::cout << "Number of centroids: " << centroids.size() << std::endl;

//                 // pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//                 // for (const auto& centroid : centroids) {
//                 //     centroid_cloud->points.push_back(centroid);
//                 // }
//                 // centroid_cloud->width = centroid_cloud->points.size();
//                 // centroid_cloud->height = 1;  
//                 // centroid_cloud->is_dense = true;
                
//                 // if (pcl::io::savePCDFileASCII("Grid.pcd", *centroid_cloud) == -1) {
//                 //     PCL_ERROR("Failed to save point cloud.");
//                 //     return -1;
//                 // }

//                 // std::cout << "Saved " << centroid_cloud->points.size() << " grid centroids to " << filename << std::endl;

                
// ******************Vehicle Drivable Space Estimation***********************  


    struct
    {
        bool operator()(std::filesystem::path a, std::filesystem::path b) const 
        {
            std::string a1 = a;
            std::string b1 = b;
            double af, bf;
            a1 = a1.substr(a1.find_last_of("/\\") + 1);
            b1 = b1.substr(b1.find_last_of("/\\") + 1);
            af = stof(a1.substr(0, a1.find_last_of(".\\") ));
            bf = stof(b1.substr(0, b1.find_last_of(".\\") ));
            return af < bf; 
        }
    }
    customLess;


    // std::vector<std::filesystem::path> files_in_directory;
    // std::copy(std::filesystem::directory_iterator(Vehicle_PCDs_Path), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
    // std::sort(files_in_directory.begin(), files_in_directory.end(), customLess);

    std::vector<std::filesystem::path> files_in_directory1, files_in_directory2;
    std::copy(std::filesystem::directory_iterator(Vehicle_PCDs_Path), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory1));
    std::sort(files_in_directory1.begin(), files_in_directory1.end(), customLess);
    std::copy(std::filesystem::directory_iterator(All_Vehicles_Transforms_Folder), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory2));
    std::sort(files_in_directory2.begin(), files_in_directory2.end(), customLess);
    Fill_transforms();
int files_count=0;
for (const std::string& filename : files_in_directory2)
    {
        //int files_count=0;
        // std::cout << filename << std::endl;
        std::string current_frame = filename;
        // std::cout << current_frame << std::endl;
        std::string current_frame_name = current_frame.substr(current_frame.find_last_of("/\\") + 1);
        // std::cout << current_frame_name << std::endl;
        Transform_file.open(All_Vehicles_Transforms_Folder+"/"+current_frame_name);
        for (int i=0;i<number_of_frames;i++)
        {
            Eigen::Matrix4f vehicle_trans = Get_Transform();
	    // cout<<"vehicle_transform: "<<vehicle_trans<<endl;
            Eigen::Vector3f top_three_last_column = vehicle_trans.col(3).head<3>();
            lists[files_count][i] = std::make_tuple(top_three_last_column(0), top_three_last_column(1), top_three_last_column(2));
            //cout<<"Path: "<<All_Vehicles_Transforms_Folder+"/"+current_frame_name<<endl;
	    //auto [x, y, z] =lists[files_count][i];
	    //cout<<"Values: "<< x<<" "<<y<<" "<<z<<endl;
       	}
        Transform_file.close();
        files_count++;
    }
	    cout<<"Completed: "<<endl;

int global_count=0;
for (const std::string& filename : files_in_directory1)
{

        float accumulate_time=0;
        // std::cout << filename << std::endl;
        std::string current_frame = filename;
        // std::cout << current_frame << std::endl;
        std::string current_frame_name = current_frame.substr(current_frame.find_last_of("/\\") + 1);
        // std::cout << Vehicle_PCDs_Path+"/"+current_frame_name << std::endl;
        // std::cout << Leader_PCD_Path+"/"+current_frame_name << std::endl;

    // std::string Vehicle_path = dataset_folder +"/"+ Vehicle_PCDs_Path +"/"+"100.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr Vehicle_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (Vehicle_PCDs_Path+"/" +current_frame_name, *Vehicle_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file  \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Vehicle_cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (Vehicle_PCDs_Path+"/" +current_frame_name, *Vehicle_cloud_temp) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file  \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Leader_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (Leader_PCD_Path+"/" +current_frame_name, *Leader_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file  \n");
        return (-1);
    }

        Eigen::Matrix4f leader_trans = World_Transforms[0][global_count];
        Eigen::Matrix4f vehicle_trans = World_Transforms[1][global_count];
        pcl::transformPointCloud(*Leader_cloud, *Leader_cloud, leader_trans);
        pcl::transformPointCloud(*Vehicle_cloud, *Vehicle_cloud, vehicle_trans);
        pcl::transformPointCloud(*Vehicle_cloud_temp, *Vehicle_cloud_temp, vehicle_trans);

    pcl::PointCloud<pcl::PointXYZ>::Ptr x_cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>()); 
    pcl::PassThrough<pcl::PointXYZ> x_pass_x;
    x_pass_x.setInputCloud(Vehicle_cloud);
    x_pass_x.setFilterFieldName("x");
    x_pass_x.setFilterLimits(xmin, xmax);

    pcl::PointCloud<pcl::PointXYZ>::Ptr x_cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> x_pass_y; 
    x_pass_y.setInputCloud(x_cloud_filtered_x);
    x_pass_y.setFilterFieldName("y");
    x_pass_y.setFilterLimits(-100, 100);

    pcl::PointCloud<pcl::PointXYZ>::Ptr y_cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> y_pass_y;
    y_pass_y.setInputCloud(Vehicle_cloud);
    y_pass_y.setFilterFieldName("y");
    y_pass_y.setFilterLimits(ymin, ymax);

    pcl::PointCloud<pcl::PointXYZ>::Ptr y_cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> y_pass_x;
    y_pass_x.setInputCloud(y_cloud_filtered_y);
    y_pass_x.setFilterFieldName("x");
    y_pass_x.setFilterLimits(-150, 50);

    run_time_start = clock();
    x_pass_x.filter(*x_cloud_filtered_x);
    x_pass_y.filter(*x_cloud_filtered_y);
    run_time_end = clock();

    run_time_start1=clock();
    y_pass_y.filter(*y_cloud_filtered_y);
    y_pass_x.filter(*y_cloud_filtered_x);
    run_time_end1 = clock();

    time1 = (run_time_end - run_time_start)>(run_time_end1 - run_time_start1)?(run_time_end - run_time_start):(run_time_end1 - run_time_start1);
    time1 = (((float)(time1)/CLOCKS_PER_SEC)*1000);
    accumulate_time=accumulate_time+time1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>());
    run_time_start = clock();
    x_cloud_filtered_y->reserve(x_cloud_filtered_y->size() + y_cloud_filtered_x->size());
    (*x_cloud_filtered_y) += (*y_cloud_filtered_x);

    pcl::PassThrough<pcl::PointXYZ> xy_pass_z;
    xy_pass_z.setInputCloud(x_cloud_filtered_y);
    xy_pass_z.setFilterFieldName("z");
    xy_pass_z.setFilterLimits(-0.1, 4);
    xy_pass_z.filter(*xy_cloud_filtered_z);

    run_time_end = clock();
    time2 = run_time_end - run_time_start;
    time2 = (((float)(time2)/CLOCKS_PER_SEC)*1000);
    accumulate_time=accumulate_time+time2;
    Vehicle_cloud = xy_cloud_filtered_z;

                // Vehicle_cloud->width =  Vehicle_cloud->points.size();  // Set width to the number of points
                //  Vehicle_cloud->height = 1; 
                // pcl::io::savePCDFileASCII("Vehicle_DS_"+current_frame_name, * Vehicle_cloud);


// ***************************************** Dynamic Objects and Non-Road Points Extraction**********************************************  

            // *************************** Dynamic Objects Extraction*********************

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled_map(new pcl::PointCloud<pcl::PointXYZ>()); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr changes(new pcl::PointCloud<pcl::PointXYZ>());      
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(Combined);
    voxel_grid.setLeafSize(0.7f, 0.7f, 0.7f); 
    voxel_grid.filter(*cloud_downsampled_map);
    double distance_threshold = 0.7; 
    detectChangesCUDA(cloud_downsampled_map, Vehicle_cloud, changes, distance_threshold);
    time3_1 = time3;

        // changes->width = changes->points.size();  // Set width to the number of points
        //  changes->height = 1;  // Unorganized point cloud
        //  pcl::io::savePCDFileASCII("Vehicles_Points_Only"+current_frame_name, *changes);

            // *************************** Non-Road Points Extraction*********************

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled_vehicle(new pcl::PointCloud<pcl::PointXYZ>()); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr Non_ground_points(new pcl::PointCloud<pcl::PointXYZ>()); 
    voxel_grid.setInputCloud(x_cloud_filtered_y);
    voxel_grid.setLeafSize(0.7f, 0.7f, 0.7f);  
    voxel_grid.filter(*cloud_downsampled_vehicle);
    detectChangesCUDA(cloud_downsampled_vehicle, Vehicle_cloud_temp, Non_ground_points, distance_threshold);
    time3= time3>time3_1?time3:time3_1;
    accumulate_time=accumulate_time+time3;

    // Non_ground_points->width = Non_ground_points->points.size();  // Set width to the number of points
    // Non_ground_points->height = 1;  // Unorganized point cloud
    // Eigen::Matrix4f vehicle_trans_inverse = vehicle_trans.inverse();
    // pcl::transformPointCloud(*Non_ground_points, *Non_ground_points, vehicle_trans_inverse);
    //   pcl::io::savePCDFileASCII("Vehicle_Non_Ground/"+current_frame_name, *Non_ground_points);
// std::vector<pcl::PointXYZ> positions_list;
//     for (int i=0;i<number_of_vehicles;i++)
//     {
//         auto [x, y, z] = lists[i][global_count];
// 	//cout<<"X and Y and Z Values: "<<x<<" "<< y<<" "<< z<<endl;
//         positions_list.push_back(pcl::PointXYZ(x,y,z));
//     }

//cout<<"List size: "<<lists.size()<<endl;
//     std::vector<pcl::PointXYZ> positions_list = { 
//     pcl::PointXYZ(-4.369153594970703125e+01,5.486972427368164062e+01,2.188285589218139648e+00),
//     pcl::PointXYZ(-4.717868423461914062e+01, 5.212943649291992188e+01, 2.414839744567871094e+00),
//     pcl::PointXYZ(-4.704954910278320312e+01, 3.341448593139648438e+01, 2.325795888900756836e+00),
//     pcl::PointXYZ(-4.354092025756835938e+01,3.187811279296875000e+01,2.444107055664062500e+00),
//     pcl::PointXYZ(-4.713151168823242188e+01, 1.282591152191162109e+01, 2.511302709579467773e+00),
//     pcl::PointXYZ(-4.356395721435546875e+01, 1.832020187377929688e+01, 2.048950433731079102e+00),
//     pcl::PointXYZ(-6.024911880493164062e+00,-9.418421983718872070e-01,1.994088411331176758e+00),
//     pcl::PointXYZ(-4.311421871185302734e+00, -4.439491748809814453e+00, 2.536698341369628906e+00),
//     pcl::PointXYZ(-2.126921272277832031e+01, -4.398709774017333984e+00, 2.081127166748046875e+00),
//     pcl::PointXYZ(-2.315992546081542969e+01,-8.944885730743408203e-01,2.210667371749877930e+00),
//     pcl::PointXYZ(-3.916044235229492188e+01, -4.354568004608154297e+00, 2.211443901062011719e+00),
//     pcl::PointXYZ(-4.262006759643554688e+01, -8.472048044204711914e-01, 2.386896371841430664e+00),
//     pcl::PointXYZ(-5.478238296508789062e+01,-5.361889648437500000e+01,2.414839506149291992e+00),
//     pcl::PointXYZ(-5.128505325317382812e+01, -5.410670471191406250e+01, 2.188285350799560547e+00),
//     pcl::PointXYZ(-5.469423294067382812e+01, -3.869682693481445312e+01, 2.699539661407470703e+00),
//     pcl::PointXYZ(-5.118546295166015625e+01,-3.745829772949218750e+01,2.472491502761840820e+00),
//     pcl::PointXYZ(-5.457848358154296875e+01, -1.924626541137695312e+01, 2.304422140121459961e+00),
//     pcl::PointXYZ(-5.105447006225585938e+01, -1.573933696746826172e+01,  2.254399776458740234e+00),
//     pcl::PointXYZ(-5.435709381103515625e+01,-4.922786951065063477e-01,2.417370319366455078e+00),
//     pcl::PointXYZ(-9.675198364257812500e+01,2.790080547332763672e+00, 1.993329167366027832e+00),
//     pcl::PointXYZ(-9.363206481933593750e+01, 6.277460098266601562e+00, 2.875283241271972656e+00),
//     pcl::PointXYZ(-7.565912628173828125e+01,2.732823371887207031e+00,2.276685953140258789e+00),
//     pcl::PointXYZ(-7.335550689697265625e+01,  6.227827548980712891e+00, 2.358937501907348633e+00),
//     pcl::PointXYZ(-6.157320022583007812e+01, 2.699161529541015625e+00, 2.644088983535766602e+00),
//     pcl::PointXYZ(-5.764266586303710938e+01, 6.189242362976074219e+00, 2.214265584945678711e+00)
// };
// int cloud_size =  changes->points.size();
// int positions_size = positions_list.size();

// // Allocate memory on the device
// pcl::PointXYZ* d_cloud_points;
// pcl::PointXYZ* d_positions;
// int* d_assignments;
// cudaMalloc((void**)&d_cloud_points, cloud_size * sizeof(pcl::PointXYZ));
// cudaMalloc((void**)&d_positions, positions_size * sizeof(pcl::PointXYZ));
// cudaMalloc((void**)&d_assignments, cloud_size * sizeof(int));

// // Copy data to device
// cudaMemcpy(d_cloud_points,  changes->points.data(), cloud_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
// cudaMemcpy(d_positions, positions_list.data(), positions_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

// // Launch CUDA kernel to assign points to positions
// threads_per_block = 256;
// num_blocks = (cloud_size + threads_per_block - 1) / threads_per_block;
// run_time_start = clock();
// assignPointsToPositions<<<num_blocks, threads_per_block>>>(d_cloud_points, cloud_size, d_positions, positions_size, d_assignments);
// cudaDeviceSynchronize();
// run_time_end = clock();
// time4 = run_time_end - run_time_start;
// time4 = (((float)(time4)/CLOCKS_PER_SEC)*1000);
// accumulate_time = accumulate_time + time4;

// // Check for kernel execution errors
// error = cudaGetLastError();
// if (error != cudaSuccess) {
//     std::cerr << "CUDA kernel failed: " << cudaGetErrorString(error) << std::endl;
//     return -1;
// }

// // Copy the assignments back to host
// std::vector<int> assignments(cloud_size);
// cudaMemcpy(assignments.data(), d_assignments, cloud_size * sizeof(int), cudaMemcpyDeviceToHost);

// // Free device memory
// cudaFree(d_cloud_points);
// cudaFree(d_positions);
// cudaFree(d_assignments);

// // Initialize a vector of point clouds to store points assigned to each position
// std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> position_clouds(positions_size);
// for (size_t i = 0; i < positions_size; ++i) {
//     position_clouds[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
// }

// run_time_start = clock();
// //Populate point clouds based on the assignment
// for (int i = 0; i < cloud_size; ++i) {
//     int pos_idx = assignments[i];  // Get the assigned position index for point i
//     if (pos_idx >= 0 && pos_idx < positions_size) {  // Ensure valid position index
//         pcl::PointXYZ point = changes->points[i];
//         position_clouds[pos_idx]->points.push_back(point);  // Correctly use changes->points[i]
//     } else {
//         std::cerr << "Invalid position index: " << pos_idx << std::endl;
//     }
// }
// run_time_end = clock();
// time5 = run_time_end - run_time_start;
// time5 = (((float)(time5)/CLOCKS_PER_SEC)*1000);
// accumulate_time = accumulate_time + time5;

// //  pcl::io::savePCDFileASCII("Kaleem.pcd", *v_x_cloud_filtered_map_x);


// // // //Save each position's assigned points as a separate PCD file
// // // for (size_t i = 0; i < positions_size; ++i) {
// // //     if (!position_clouds[i]->points.empty()) {
// // //         std::stringstream filename;
// // //         filename << "position_" << i << "_points.pcd";
// // //         position_clouds[i]->width = position_clouds[i]->points.size();  // Set width to the number of points
// // //         position_clouds[i]->height = 1;  // Unorganized point cloud
// // //         pcl::io::savePCDFileASCII(filename.str(), *position_clouds[i]);
// // //         std::cout << "Saved " << position_clouds[i]->points.size() << " points to " << filename.str() << std::endl;
// // //     }
// // // }


// //Save each position's assigned points as a separate PCD file
// for (size_t i = 0; i < positions_size; ) {
//     if (position_clouds[i]->points.empty()) {
//         positions_list.erase(positions_list.begin() + i);
//         cout<<"Errased: "<<positions_list[i]<<endl;
//         --positions_size;
//     } 
//     else {
//         ++i; 
//     }
// }
// // *****************************************BlindSpots Estimation*********************************************************************   
    
// {
//     int cloudSize = Vehicle_cloud->size();
//     int num_centroids = centroids.size();

//     pcl::PointXYZ *d_centroids;
//     cudaMalloc((void**)&d_centroids, num_centroids * sizeof(pcl::PointXYZ));
//     cudaMemcpy(d_centroids, centroids.data(), num_centroids * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
//     std::vector<int> result;

//     std::vector<int> vehicle_pd(num_centroids, 0);

//     cloudSize = Vehicle_cloud->size();
//     result.resize(cloudSize);

//     pcl::PointXYZ *h_points = Vehicle_cloud->points.data();  
//     pcl::PointXYZ *d_points;
        
//     cudaMalloc((void**)&d_points, cloudSize * sizeof(pcl::PointXYZ));
//     cudaMemcpy(d_points, h_points, cloudSize * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

//     int *res_ptr = result.data();
//     int *d_res_ptr;

//     cudaMalloc((void**)&d_res_ptr, cloudSize * sizeof(int));
//     cudaMemcpy(d_res_ptr, res_ptr, cloudSize * sizeof(int), cudaMemcpyHostToDevice);

//     int numBlocks = (cloudSize + threadsPerBlock - 1) / threadsPerBlock;
//     run_time_end = clock();
//     getNN_modified<<<numBlocks, threadsPerBlock>>>(cloudSize, d_points, d_res_ptr, d_centroids, num_centroids);
//     cudaDeviceSynchronize();
//     run_time_end = clock();
//     time6 = run_time_end - run_time_start;
//     time6 = (((float)(time6)/CLOCKS_PER_SEC)*1000);
//     accumulate_time = accumulate_time + time6;

//     cudaMemcpy(res_ptr, d_res_ptr, cloudSize * sizeof(int), cudaMemcpyDeviceToHost);

//     cudaFree(d_points);
//     cudaFree(d_res_ptr);
//     cudaFree(d_centroids);

//     int cloudSize2 = result.size();
//     int gridSize2 = vehicle_pd.size();

//     int* d_result2 = nullptr;
//     int* d_vehicle_pd = nullptr;
//     cudaMalloc(&d_result2, cloudSize2 * sizeof(int));
//     cudaMalloc(&d_vehicle_pd, gridSize2 * sizeof(int));

//     cudaMemcpy(d_result2, result.data(), cloudSize2 * sizeof(int), cudaMemcpyHostToDevice);
//     cudaMemcpy(d_vehicle_pd, vehicle_pd.data(), gridSize2 * sizeof(int), cudaMemcpyHostToDevice);

//     int threadsPerBlock = 256;
//     int blocksPerGrid = (cloudSize2 + threadsPerBlock - 1) / threadsPerBlock;
//     run_time_start = clock();
//     incrementKernel<<<blocksPerGrid, threadsPerBlock>>>(d_result2, d_vehicle_pd, cloudSize2, gridSize2);
//     cudaDeviceSynchronize();
//     cudaCheckError();
//     run_time_end = clock();
//     time7 = run_time_end - run_time_start;
//     time7 = (((float)(time7)/CLOCKS_PER_SEC)*1000);
//     accumulate_time = accumulate_time + time7;

//     cudaMemcpy(vehicle_pd.data(), d_vehicle_pd, gridSize2 * sizeof(int), cudaMemcpyDeviceToHost);

//     cudaFree(d_result2);
//     cudaFree(d_vehicle_pd);


// // *****************************************BlindSpots Assignment to Suppliers*********************************************************************   

//     int centroids_size = centroids.size();
//     int vehicle_pd_size = vehicle_pd.size();
//     positions_size = positions_list.size();


//      d_centroids = nullptr;
//      d_vehicle_pd = nullptr;
//     pcl::PointXYZ* d_positions_list = nullptr;

//     cudaMalloc(&d_centroids, centroids_size * sizeof(pcl::PointXYZ));
//     cudaMalloc(&d_vehicle_pd, vehicle_pd_size * sizeof(int));
//     cudaMalloc(&d_positions_list, positions_size * sizeof(pcl::PointXYZ));
//     cudaMemcpy(d_centroids, centroids.data(), centroids_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
//     cudaMemcpy(d_vehicle_pd, vehicle_pd.data(), vehicle_pd_size * sizeof(int), cudaMemcpyHostToDevice);
//     cudaMemcpy(d_positions_list, positions_list.data(), positions_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);


//     threadsPerBlock = 256;
//     blocksPerGrid = (vehicle_pd_size + threadsPerBlock - 1) / threadsPerBlock;


//     int max_bounding_boxes_per_position = centroids_size;  
//     int total_bounding_boxes = positions_size * max_bounding_boxes_per_position;

//     BoundingBox* d_bounding_boxes = nullptr;
//     int* d_bounding_box_offsets = nullptr;
//     int* d_bounding_box_counts = nullptr;

//     cudaMalloc(&d_bounding_boxes, total_bounding_boxes * sizeof(BoundingBox));
//     cudaMalloc(&d_bounding_box_offsets, positions_size * sizeof(int));
//     cudaMalloc(&d_bounding_box_counts, positions_size * sizeof(int));

//     // Initialize offsets and counts
//     std::vector<int> bounding_box_offsets(positions_size);
//     for (int i = 0; i < positions_size; ++i) {
//         bounding_box_offsets[i] = i * max_bounding_boxes_per_position;
//     }
//     cudaMemcpy(d_bounding_box_offsets, bounding_box_offsets.data(), positions_size * sizeof(int), cudaMemcpyHostToDevice);
//     cudaMemset(d_bounding_box_counts, 0, positions_size * sizeof(int));
//     run_time_start = clock();
//     updateBoundingBoxesKernel<<<blocksPerGrid, threadsPerBlock>>>(
//         d_centroids,
//         d_vehicle_pd,
//         d_positions_list,
//         d_bounding_boxes,
//         d_bounding_box_offsets,
//         d_bounding_box_counts,
//         vehicle_pd_size,
//         positions_size,
//         cell_x_size,
//         cell_y_size,
//         max_bounding_boxes_per_position
//     );
//     run_time_end = clock();
//     time8 = run_time_end - run_time_start;
//     time8 = (((float)(time8)/CLOCKS_PER_SEC)*1000);
//     accumulate_time = accumulate_time + time8;


//     std::vector<BoundingBox> bounding_boxes(total_bounding_boxes);
//     std::vector<int> bounding_box_counts(positions_size);
//     cudaMemcpy(bounding_boxes.data(), d_bounding_boxes, total_bounding_boxes * sizeof(BoundingBox), cudaMemcpyDeviceToHost);
//     cudaMemcpy(bounding_box_counts.data(), d_bounding_box_counts, positions_size * sizeof(int), cudaMemcpyDeviceToHost);
    
//     // int total=0;
//     // // Process results
//     // for (int i = 0; i < positions_size; ++i) {
//     //     // std::cout << "Position " << i << " has " << bounding_box_counts[i] << " bounding boxes:" << std::endl;
//     //     BB_file << i <<" "<< bounding_box_counts[i]<< std::endl;
//     //     // total=total+bounding_box_counts[i];
//     //     for (int j = 0; j < bounding_box_counts[i]; ++j) {
//     //         BoundingBox& box = bounding_boxes[i * max_bounding_boxes_per_position + j];
//     //         BB_file << box.minX <<" "<< box.minY<<" "<<box.maxX <<" "<< box.maxY << std::endl;
//     //         // std::cout << "  minX: " << box.minX << ", minY: " << box.minY
//     //         //           << ", maxX: " << box.maxX << ", maxY: " << box.maxY << std::endl;
//     //     }
//     // }

//         int total=0;
//     // Process results
//     for (int i = 0; i < positions_size; ++i) {
//         BB_file << positions_list[i] <<" "<< bounding_box_counts[i]<< std::endl;
//         for (int j = 0; j < bounding_box_counts[i]; ++j) {
//             BoundingBox& box = bounding_boxes[i * max_bounding_boxes_per_position + j];
//             float centerX = (box.minX + box.maxX) / 2.0f;
//             float centerY = (box.minY + box.maxY) / 2.0f;
//             BB_file << centerX <<" "<< centerY<<std::endl;
//         }
//     }


//     // // // Store centers as a point cloud
//     //  pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud(new pcl::PointCloud<pcl::PointXYZ>);

//     //  for (int i = 0; i < positions_size; ++i) {
//     //      for (int j = 0; j < bounding_box_counts[i]; ++j) {
//     //          BoundingBox& box = bounding_boxes[i * max_bounding_boxes_per_position + j];

//     //          // Calculate the center of the bounding box
//     //          float centerX = (box.minX + box.maxX) / 2.0f;
//     //          float centerY = (box.minY + box.maxY) / 2.0f;
//     //         float centerZ = 0.0f;  // Assuming 2D bounding boxes; set Z to 0 or another value if applicable.

//     //          // Add the center as a point in the point cloud
//     //          pcl::PointXYZ center_point(centerX, centerY, centerZ);
//     //          centers_cloud->points.push_back(center_point);
//     //      }
//     //  }

//     // // // Set cloud properties
//     // centers_cloud->width = centers_cloud->points.size();
//     //  centers_cloud->height = 1; // Unordered point cloud

//     //  // Optionally save to a PCD file
//     //  pcl::io::savePCDFileASCII("bounding_box_centers"+current_frame_name, *centers_cloud);
//     //  std::cout << "Saved " << centers_cloud->points.size() << " bounding box centers to bounding_box_centers.pcd" << std::endl;


//     // Clean up
//     cudaFree(d_bounding_boxes);
//     cudaFree(d_bounding_box_offsets);
//     cudaFree(d_bounding_box_counts);
// }

// // *****************************************Overlap Estimation*********************************************************************   

    float time_overlap=0.0;

    pcl::PointXYZ minPt_leader, maxPt_leader;
    minPt_vehicle, maxPt_vehicle;
    minPt, maxPt;
    pcl::getMinMax3D (*Leader_cloud, minPt_leader, maxPt_leader);
    pcl::getMinMax3D (*Non_ground_points, minPt_vehicle, maxPt_vehicle);

    minPt.x = std::min(minPt_leader.x, minPt_vehicle.x);
    minPt.y = std::min(minPt_leader.y, minPt_vehicle.y);
    minPt.z = std::min(minPt_leader.z, minPt_vehicle.z);

    maxPt.x = std::max(maxPt_leader.x, maxPt_vehicle.x);
    maxPt.y = std::max(maxPt_leader.y, maxPt_vehicle.y);
    maxPt.z = std::max(maxPt_leader.z, maxPt_vehicle.z);

    grid_info = get_grid(minPt, maxPt);

    pcl::PointXYZ min_centroid = grid_info[0];
    pcl::PointXYZ max_centroid = grid_info[1];
    pcl::PointXYZ lims = grid_info[2];

    int gridSize = lims.x*lims.y*lims.z+1;

    std::vector<int> result1;
    std::vector<int> result2;

    std::vector<int> leader_pd(gridSize, 0),vehicle_pd_o(gridSize, 0);
    std::vector<bool> ipd(gridSize, 0); 

    {
        int cloudSize = Leader_cloud->size();
        result1.resize(cloudSize);

        pcl::PointXYZ *h_points = Leader_cloud->points.data();  // Get pointer to the points array
        pcl::PointXYZ *d_points;

        // Allocate GPU memory for the point cloud
        cudaMalloc((void**)&d_points, cloudSize * sizeof(pcl::PointXYZ));
        cudaMemcpy(d_points, h_points, cloudSize * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

        int *res_ptr = result1.data();
        int *d_res_ptr;

        // Allocate GPU memory for the results
        cudaMalloc((void**)&d_res_ptr, cloudSize * sizeof(int));
        cudaMemcpy(d_res_ptr, res_ptr, cloudSize * sizeof(int), cudaMemcpyHostToDevice);

        int numBlocks = (cloudSize + threadsPerBlock - 1) / threadsPerBlock;
        run_time_start = clock();
        getNN<<<numBlocks, threadsPerBlock>>>(cloudSize, d_points, d_res_ptr, min_centroid.x, min_centroid.y, min_centroid.z, lims.x, lims.y, lims.z);
        cudaDeviceSynchronize();
        run_time_end = clock();
        time_overlap=time_overlap+(run_time_end-run_time_start);
        cudaMemcpy(res_ptr, d_res_ptr, cloudSize * sizeof(int), cudaMemcpyDeviceToHost);
        cudaFree(d_points);
        cudaFree(d_res_ptr);
                            
        int cloudSize1 = result1.size();
        int gridSize1 = leader_pd.size();

        int* d_result1 = nullptr;
        int* d_leader_pd = nullptr;
        cudaMalloc(&d_result1, cloudSize1 * sizeof(int));
        cudaMalloc(&d_leader_pd, gridSize1 * sizeof(int));

        cudaMemcpy(d_result1, result1.data(), cloudSize1 * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(d_leader_pd, leader_pd.data(), gridSize1 * sizeof(int), cudaMemcpyHostToDevice);

        int threadsPerBlock = 256;
        int blocksPerGrid = (cloudSize1 + threadsPerBlock - 1) / threadsPerBlock;
        run_time_start = clock();
        incrementKernel<<<blocksPerGrid, threadsPerBlock>>>(d_result1, d_leader_pd, cloudSize1, gridSize1);
        cudaDeviceSynchronize();
        cudaCheckError();
        run_time_end = clock();
        time_overlap=time_overlap+(run_time_end-run_time_start);
        cudaMemcpy(leader_pd.data(), d_leader_pd, gridSize1 * sizeof(int), cudaMemcpyDeviceToHost);

        cudaFree(d_result1);
        cudaFree(d_leader_pd);

    }

    {
        int cloudSize_v = Non_ground_points->size();
        result2.resize(cloudSize_v);

        pcl::PointXYZ *h_points = Non_ground_points->points.data();  // Get pointer to the points array
        pcl::PointXYZ *d_points;

        // Allocate GPU memory for the point cloud
        cudaMalloc((void**)&d_points, cloudSize_v * sizeof(pcl::PointXYZ));
        cudaMemcpy(d_points, h_points, cloudSize_v * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

         int *res_ptr = result2.data();
         int *d_res_ptr;

        // Allocate GPU memory for the results
        cudaMalloc((void**)&d_res_ptr, cloudSize_v * sizeof(int));
        cudaMemcpy(d_res_ptr, res_ptr, cloudSize_v * sizeof(int), cudaMemcpyHostToDevice);

        int numBlocks = (cloudSize_v + threadsPerBlock - 1) / threadsPerBlock;
        run_time_start = clock();
        getNN<<<numBlocks, threadsPerBlock>>>(cloudSize_v, d_points, d_res_ptr, min_centroid.x, min_centroid.y, min_centroid.z, lims.x, lims.y, lims.z);
        cudaDeviceSynchronize();
        run_time_end = clock();
        time_overlap=time_overlap+(run_time_end-run_time_start);
        cudaMemcpy(res_ptr, d_res_ptr, cloudSize_v * sizeof(int), cudaMemcpyDeviceToHost);

        cudaFree(d_points);
        cudaFree(d_res_ptr);
            
        
        int cloudSize2 = result2.size();
        int gridSize2 = vehicle_pd_o.size();

        // Allocate device memory
        int* d_result2 = nullptr;
        int* d_vehicle_pd_o = nullptr;
        cudaMalloc(&d_result2, cloudSize2 * sizeof(int));
        cudaMalloc(&d_vehicle_pd_o, gridSize2 * sizeof(int));

        // Copy data from host to device
        cudaMemcpy(d_result2, result2.data(), cloudSize2 * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(d_vehicle_pd_o, vehicle_pd_o.data(), gridSize2 * sizeof(int), cudaMemcpyHostToDevice);

        // Define the number of threads and blocks
        int threadsPerBlock = 256;
        int blocksPerGrid = (cloudSize2 + threadsPerBlock - 1) / threadsPerBlock;
        run_time_start = clock();
        incrementKernel<<<blocksPerGrid, threadsPerBlock>>>(d_result2, d_vehicle_pd_o, cloudSize2, gridSize2);
        cudaDeviceSynchronize();
        run_time_end = clock();
        time_overlap=time_overlap+(run_time_end-run_time_start);        
        cudaCheckError();
        cudaMemcpy(vehicle_pd_o.data(), d_vehicle_pd_o, gridSize2 * sizeof(int), cudaMemcpyDeviceToHost);

        // Free device memory
        cudaFree(d_result2);
        cudaFree(d_vehicle_pd_o);

    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud_Vehicle(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud_Leader(new pcl::PointCloud<pcl::PointXYZ>()); // Pointer for the final cloud
    int leader_size = leader_pd.size();
    int vehicle_size = vehicle_pd_o.size();

    // Device memory allocation
    int *d_leader_pd, *d_vehicle_pd_o, *d_indices_above_threshold, *d_above_threshold, *d_final_count_vehicle,*d_final_count_leader;
    pcl::PointXYZ *d_vehicle_points,*d_leader_points, *d_final_vehicle_points,*d_final_leader_points;

    int *res_ptr  = result2.data();
    int *d_res_ptr=nullptr; 

    int * res_ptr_leader = result1.data();
    int * d_res_ptr_leader; 
    cudaMalloc((void**)&d_res_ptr, Non_ground_points->points.size() * sizeof(int));
    cudaMemcpy(d_res_ptr, res_ptr, Non_ground_points->points.size() * sizeof(int), cudaMemcpyHostToDevice);

    cudaMalloc((void**)&d_res_ptr_leader, Leader_cloud->points.size() * sizeof(int));
    cudaMemcpy(d_res_ptr_leader, res_ptr_leader, Leader_cloud->points.size() * sizeof(int), cudaMemcpyHostToDevice);

    // Allocate device memory
    cudaMalloc(&d_leader_pd, leader_size * sizeof(int));
    cudaMalloc(&d_vehicle_pd_o, vehicle_size * sizeof(int));
    cudaMalloc(&d_indices_above_threshold, leader_size * sizeof(int)); // Assume worst-case size
    cudaMalloc(&d_above_threshold, sizeof(int));
    cudaMalloc(&d_final_count_vehicle, sizeof(int));
    cudaMalloc(&d_final_count_leader, sizeof(int));
    cudaMalloc(&d_vehicle_points, Non_ground_points->points.size() * sizeof(pcl::PointXYZ));
    cudaMalloc(&d_leader_points, Leader_cloud->points.size() * sizeof(pcl::PointXYZ));
    cudaMalloc(&d_final_vehicle_points, Non_ground_points->points.size() * sizeof(pcl::PointXYZ)); // Allocate for worst case
    cudaMalloc(&d_final_leader_points, Leader_cloud->points.size() * sizeof(pcl::PointXYZ)); // Allocate for worst case

    // Copy data to device
    cudaMemcpy(d_leader_pd, leader_pd.data(), leader_size * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_vehicle_pd_o, vehicle_pd_o.data(), vehicle_size * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_vehicle_points, Non_ground_points->points.data(), Non_ground_points->points.size() * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
    cudaMemcpy(d_leader_points, Leader_cloud->points.data(), Leader_cloud->points.size() * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

    cudaMemset(d_above_threshold, 0, sizeof(int)); // Initialize above_threshold to 0
    cudaMemset(d_final_count_vehicle, 0, sizeof(int)); // Initialize final count to 0
    cudaMemset(d_final_count_leader, 0, sizeof(int)); // Initialize final count to 0

    int blockSize = 256;
    int numBlocks = (leader_size + blockSize - 1) / blockSize;
    run_time_start = clock();
    findAboveThreshold<<<numBlocks, blockSize>>>(d_leader_pd, d_vehicle_pd_o, d_indices_above_threshold, d_above_threshold, leader_size, vehicle_size, ipd_threshold);
     cudaDeviceSynchronize();
    run_time_end = clock();
    time_overlap=time_overlap+(run_time_end-run_time_start);   
    int above_threshold;
    cudaMemcpy(&above_threshold, d_above_threshold, sizeof(int), cudaMemcpyDeviceToHost);

    // Launch kernel to populate final_cloud based on the result2 mapping and threshold indices
    numBlocks = (Leader_cloud->points.size() + blockSize - 1) / blockSize; // Adjust for the number of points in the vehicle cloud
    run_time_start = clock();
    populateFinalCloud<<<numBlocks, blockSize>>>(d_vehicle_points,d_leader_points,d_res_ptr,d_res_ptr_leader, d_indices_above_threshold, above_threshold, d_final_vehicle_points,d_final_leader_points, d_final_count_vehicle,d_final_count_leader,Leader_cloud->points.size());
    cudaDeviceSynchronize();
    run_time_end = clock();
    time_overlap=time_overlap+(run_time_end-run_time_start);   
    cudaCheckError();

    int final_count_leader;
    cudaMemcpy(&final_count_leader, d_final_count_leader, sizeof(int), cudaMemcpyDeviceToHost);
    final_cloud_Leader->points.resize(final_count_leader); // Resize to the actual number of valid points
    cudaMemcpy(final_cloud_Leader->points.data(), d_final_leader_points, final_count_leader * sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost);
    cudaCheckError();

    int final_count_vehicle;
    cudaMemcpy(&final_count_vehicle, d_final_count_vehicle, sizeof(int), cudaMemcpyDeviceToHost);
    final_cloud_Vehicle->points.resize(final_count_vehicle); // Resize to the actual number of valid points
    cudaMemcpy(final_cloud_Vehicle->points.data(), d_final_vehicle_points, final_count_vehicle * sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost);
    cudaCheckError();


//Assuming leader_trans and vehicle_trans are transformation matrices


    // Compute the inverse of the transformation matrices
    Eigen::Matrix4f leader_trans_inverse = leader_trans.inverse();
    Eigen::Matrix4f vehicle_trans_inverse = vehicle_trans.inverse();


     //std::cout << "Final Vehicle cloud size: " << final_cloud_Vehicle->points.size() << std::endl;
     final_cloud_Vehicle->width = final_cloud_Vehicle->points.size();
     final_cloud_Vehicle->height = 1; // Unordered point cloud

    pcl::transformPointCloud(*final_cloud_Vehicle, *final_cloud_Vehicle, vehicle_trans_inverse);

     pcl::io::savePCDFileASCII("V2_Overlapping/Vehicle/"+current_frame_name, *final_cloud_Vehicle);

     //std::cout << "Final Leader cloud size: " << final_cloud_Leader->points.size() << std::endl;
     final_cloud_Leader->width = final_cloud_Leader->points.size();
    final_cloud_Leader->height = 1; // Unordered point cloud


    // Transform the point clouds using the inverse transformation matrices
    pcl::transformPointCloud(*final_cloud_Leader, *final_cloud_Leader, leader_trans_inverse);

     pcl::io::savePCDFileASCII("V2_Overlapping/Leader/"+current_frame_name, *final_cloud_Leader);
    time_overlap=(((float)(time_overlap)/CLOCKS_PER_SEC)*1000);
    // accumulate_time=accumulate_time+time9; 
    global_count++;  

    // Free device memory
    cudaFree(d_leader_pd);
    cudaFree(d_vehicle_pd_o);
    cudaFree(d_indices_above_threshold);
    cudaFree(d_above_threshold);
    cudaFree(d_vehicle_points);
    cudaFree(d_leader_points);
    cudaFree(d_res_ptr);
    cudaFree(d_final_vehicle_points);
    cudaFree(d_final_leader_points);
    cudaFree(d_final_count_vehicle);
    cudaFree(d_final_count_leader);

    // log_file<<time1+time2<<","<<time3<<","<<time4+time5<<","<<time6+time7<<","<<time8<<","<<time9<<","<<accumulate_time<<endl;
    log_file<<time_overlap<<endl;
    std::cout << "Overlap Estimation Time: " << time_overlap << std::endl;
    // std::cout << "Accumulated Time: " << accumulate_time << std::endl;
    // std::cout << "Road Filter Time: " << time1 + time2 << std::endl;
    // std::cout << "Dynamic Objects Extraction Time: " << time3 << std::endl;
    // std::cout << "Vehicle Points Assignment Time: " << time4+time5 << std::endl;
    // std::cout << "BlindSPots Calculation Time: " << time6+time7 << std::endl;
    // std::cout << "BlindSpts Assignment to the Suppliers: "<<time8<<endl;
    // std::cout << "Estimating and Filtering Overlapping Regions: "<<time9<<endl;
}

return 0;
}


int main (int argc, char** argv)

{

    dataset_folder =argv[1];
    Map = argv[2]; 
    Leader_PCD_Path=argv[3];
    Vehicle_PCDs_Path=argv[4];
    Leader_transform_file = argv[5];
    Vehicle_transform_file = argv[6];
    All_Vehicles_Transforms_Folder=argv[7];
    Output_file = argv[8];
    
    Vehicle_PCDs_Path=dataset_folder +"/"+ Vehicle_PCDs_Path;
    Leader_PCD_Path=dataset_folder +"/"+ Leader_PCD_Path;
    Leader_transform_file=dataset_folder +"/"+Leader_transform_file+".txt";
    Vehicle_transform_file=dataset_folder +"/"+Vehicle_transform_file+".txt";
    All_Vehicles_Transforms_Folder=dataset_folder +"/"+All_Vehicles_Transforms_Folder;
    Output_file= dataset_folder +"/"+Output_file+".csv";
    leader_file.open(Leader_transform_file);
    vehicle_file.open(Vehicle_transform_file);
    log_file.open(Output_file);
    log_file<<"Overlap_Estimation_Latency"<<endl;
    // log_file<<"Road Filter"<<","<<"Dynamic Objects Extraction"<<","<<"Vehicle Points Assignment"<<","<<"BlindSPots Calculation"<<","<<"BlindSpts Assignment to the Suppliers"<<","<<"Estimating and Filtering Overlapping Regions"<<","<<"E2E_Latency"<<endl;

    // cout<<"Leader Path: "<<Leader_PCD_Path<<endl;
    // cout<<"Vehicle Path: "<<Vehicle_PCDs_Path<<endl;
    // cout<<"Leader_transform_file: "<<Leader_transform_file<<endl;
    // cout<<"Vehicle_transform_file: "<<Vehicle_transform_file<<endl;
    // cout<<"All Vehicles Transform Folder: "<<All_Vehicles_Transforms_Folder<<endl;
    // cout<<"Output File: "<< Output_file<<endl;
    //  BB_file.open("BB.txt"); // Open the second file
     Intersection_Points_and_Grid_Estimation();
    //  BB_file.close();

    return (0);
}

