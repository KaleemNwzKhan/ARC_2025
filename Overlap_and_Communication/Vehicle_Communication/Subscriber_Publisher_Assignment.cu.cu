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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <vector>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <ctime>

// Constants for grid size
const float CELL_LENGTH = 2.5;
const float CELL_WIDTH = 2.5;
const float CELL_HEIGHT = 4.0;

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_set>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_set>
#include <filesystem>
using namespace std;
namespace fs = std::filesystem;

namespace fs = std::filesystem;

const std::string DIR_SOURCE = "BlindSpots";   // Directory containing files to process
const std::string DIR_COMPARE = "Vehicles_Cells"; // Directory containing files for comparison
const std::string DIR_OUTPUT = "Required";   // Directory for writing matched results
const std::string BASE_DIR = "Required";   // Directory for writing matched results


const int NUM_VEHICLES = 40;  // Vehicle_0 to Vehicle_39
const int NUM_FRAMES = 150;   // Files from 0.txt to 150.txt

// Function to read values from a file into an unordered_set
std::unordered_set<float> readValuesFromFile(const std::string& filePath) {
    std::unordered_set<float> values;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return values;
    }

    float value;
    while (file >> value) {
        values.insert(value);
    }
    file.close();

    // std::cout << "Read " << values.size() << " values from " << filePath << std::endl;
    return values;
}

// Function to write matched values to an output file
void writeMatchesToFile(const std::string& outputFile, const std::vector<std::pair<float, std::string>>& matches) {
    if (matches.empty()) {
        // std::cout << "No matches found. Skipping file: " << outputFile << std::endl;
        return;
    }

    std::ofstream file(outputFile, std::ios::out);
    if (!file.is_open()) {
        // std::cerr << "Error opening output file: " << outputFile << std::endl;
        return;
    }

    for (const auto& match : matches) {
        file << match.first << " " << match.second << "\n";
    }
    file.close();

    // std::cout << "Wrote " << matches.size() << " matches to " << outputFile << std::endl;
}

// Function to write remaining unmatched values back to a file
void writeRemainingValuesToFile(const std::string& filePath, const std::unordered_set<float>& values) {
    std::ofstream file(filePath, std::ios::out);
    if (!file.is_open()) {
        // std::cerr << "Error opening file for writing: " << filePath << std::endl;
        return;
    }

    for (float value : values) {
        file << value << "\n";
    }
    file.close();

    // std::cout << "Updated " << values.size() << " remaining values in " << filePath << std::endl;
}



std::string removePCDExtension(const std::string& filename) {
    size_t pos = filename.find_last_of(".");
    if (pos != std::string::npos && filename.substr(pos) == ".pcd") {
        return filename.substr(0, pos); // Remove ".pcd"
    }
    return filename; // Return unchanged if ".pcd" not found
}


//**************************************************Global Variables****************************************************************************************************
float Instersection_x = 1;
float Intersection_y = 2;
float Intersection_z = 3;

int number_of_vehicles=40;
int number_of_frames=150;
int Intersection_Points_and_Grid_Estimation();
std::vector<int> point_density (pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ, pcl::PointXYZ);
std::vector<pcl::PointXYZ> get_grid(pcl::PointXYZ, pcl::PointXYZ);
std::vector<std::vector<std::tuple<float, float, float>>> lists(number_of_vehicles, std::vector<std::tuple<float, float, float>>(number_of_frames));
std::vector<std::vector<Eigen::Matrix4f>> World_Transforms(2, std::vector<Eigen::Matrix4f>(number_of_frames));
std::ofstream log_file,BB_file,BB_size_file;
std::ifstream V_leader_file,V_map_file,Transform_file;
Eigen::Matrix4f Vehicle_transform,vehicle_trans;
int ipd_threshold = 10;

std::string dataset_folder, Map, Vehicle_PCDs_Path,V_Leader_transform_file,BlindSpot_Size_Folder,V_Map_transform_file, All_Vehicles_Leader_Transforms_Folder, Output_file, BlindSpots_File;
int Vehicle_number;
float run_time_start,run_time_end,run_time_start1,run_time_end1;
float time1,time2, time3, time3_1, time4, time5, time6, time7, time8, time9, time10, time11, time12;
const int threadsPerBlock = 256;
const float cell_x_size = 2.5;
const float cell_y_size = 2.5;
const float cell_z_size = 4.0;
std::string filename = "grid_centroids.pcd";
std::vector<pcl::PointXYZ> positions_list;

float xmin = -56.95;
float xmax = -41.18;
float ymin = -6.8;
float ymax = 8.9;
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
    float minZ;
    float maxZ;
    float centerX;
    float centerY;
    float centerZ;

    __device__ CellInfo(float x, float y, float z, float cell_x_size, float cell_y_size,float cell_z_size) {
        centerX = x;
        centerY = y;
        centerZ = z;
        minX = centerX - cell_x_size / 2;
        maxX = centerX + cell_x_size / 2;
        minY = centerY - cell_y_size / 2;
        maxY = centerY + cell_y_size / 2;
        minZ = centerZ - cell_z_size / 2;
        maxZ = centerZ + cell_z_size / 2;
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

float computeDist3 ( float x1, float y1, float z1, float x2, float y2, float z2) {
    return computeDist1( x1, x2 ) + computeDist1( y1, y2 ) + computeDist1( z1, z2 );
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
    run_time_end = clock();
    time3 = ((float(run_time_end - run_time_start) / CLOCKS_PER_SEC) * 1000);
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
                             (point.y - positions[i].y) * (point.y - positions[i].y)+(point.z - positions[i].z) * (point.z - positions[i].z);

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
    float minX, minY, maxX, maxY,minZ,maxZ;
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
    float cell_z_size,
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
        CellInfo cell_info(centroid.x, centroid.y, centroid.z, cell_x_size, cell_y_size,cell_z_size);
        BoundingBox box = {cell_info.minX, cell_info.minY, cell_info.maxX, cell_info.maxY,cell_info.minZ, cell_info.maxZ};

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


__global__ void findZeroCentroids(int* d_vehicle_pd, int* d_zero_centroids, int gridSize) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < gridSize) {
        if (d_vehicle_pd[idx] == 0) {
            d_zero_centroids[idx] = 1;  // Mark centroid as having zero points
        } else {
            d_zero_centroids[idx] = 0;  // Mark centroid as having points
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
    Transform_file.open(V_Leader_transform_file);
    for (int i=0;i<number_of_frames;i++)
    {
       Eigen::Matrix4f lead_trans=Get_Transform();
       World_Transforms[0][i]=lead_trans;
    //    cout<<"Leader: "<<World_Transforms[0][i]<<endl;
    }
   Transform_file.close();

    Transform_file.open(V_Map_transform_file);
    for (int i=0;i<number_of_frames;i++)
    {
       Eigen::Matrix4f veh_trans=Get_Transform();
       World_Transforms[1][i]=veh_trans;
    //    cout<<"Vehicle: "<<World_Transforms[1][i]<<endl;
    }
   Transform_file.close();
   
}

// Compute affinity matrix using Euclidean distance and density similarity
Eigen::MatrixXf computeAffinityMatrix(
    const std::vector<pcl::PointXYZ> &prev_centroids,
    const std::vector<pcl::PointXYZ> &curr_centroids,
    const std::vector<float> &prev_densities,
    const std::vector<float> &curr_densities) {
    
    size_t N = prev_centroids.size();
    size_t M = curr_centroids.size();
    Eigen::MatrixXf affinityMatrix(N, M);

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < M; ++j) {
            float dist = std::sqrt(
                std::pow(prev_centroids[i].x - curr_centroids[j].x, 2) +
                std::pow(prev_centroids[i].y - curr_centroids[j].y, 2) +
                std::pow(prev_centroids[i].z - curr_centroids[j].z, 2)
            );
            float density_diff = std::abs(prev_densities[i] - curr_densities[j]);

            // Higher affinity for closer points and similar densities
            affinityMatrix(i, j) = -dist - density_diff;
        }
    }
    return affinityMatrix;
}

// Hungarian Algorithm (Stub: Replace with actual implementation)
std::vector<int> assignTracks(const Eigen::MatrixXf &affinityMatrix) {
    std::vector<int> assignments(affinityMatrix.rows(), -1);
    // Hungarian algorithm implementation should go here
    return assignments;
}

// Compute centroids & densities for each assigned grid cell
void computeCentroidsAndDensities(
    pcl::PointCloud<pcl::PointXYZ>::Ptr changes,
    pcl::PointCloud<pcl::PointXYZ>::Ptr assigned_centroids,
    std::vector<pcl::PointXYZ> &centroids,
    std::vector<float> &densities) {

    centroids.clear();
    densities.clear();

    std::vector<int> point_counts(assigned_centroids->points.size(), 0);

    for (const auto &point : changes->points) {
        float min_dist = std::numeric_limits<float>::max();
        int best_index = -1;

        for (size_t i = 0; i < assigned_centroids->points.size(); ++i) {
            float dist = std::sqrt(
                std::pow(point.x - assigned_centroids->points[i].x, 2) +
                std::pow(point.y - assigned_centroids->points[i].y, 2) +
                std::pow(point.z - assigned_centroids->points[i].z, 2)
            );

            if (dist < min_dist) {
                min_dist = dist;
                best_index = i;
            }
        }

        if (best_index != -1) {
            ++point_counts[best_index];
        }
    }

    for (size_t i = 0; i < assigned_centroids->points.size(); ++i) {
        if (point_counts[i] > 0) {  // Only keep occupied cells
            centroids.push_back(assigned_centroids->points[i]);
            densities.push_back(static_cast<float>(point_counts[i]) /
                                (CELL_LENGTH * CELL_WIDTH * CELL_HEIGHT));
        }
    }
}


void printPositions(const std::vector<pcl::PointXYZ>& positions, const std::string& title) {
    std::cout << title << ":\n";
    for (const auto& p : positions) {
        std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")\n";
    }
    std::cout << "--------------------------------\n";
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

                //  Combined->width = Combined->points.size();  // Set width to the number of points
                //  Combined->height = 1; 
                //  pcl::io::savePCDFileASCII("Map.pcd", *Combined);

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

    centroids=filtered_positions;   //centroids of the cells on the drivable space only



                std::cout << "Number of centroids: " << centroids.size() << std::endl;

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



    std::vector<std::filesystem::path> files_in_directory1, files_in_directory2;
    
    std::copy(std::filesystem::directory_iterator(Vehicle_PCDs_Path), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory1));
    std::sort(files_in_directory1.begin(), files_in_directory1.end(), customLess);
    
    std::copy(std::filesystem::directory_iterator(All_Vehicles_Leader_Transforms_Folder), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory2));
    std::sort(files_in_directory2.begin(), files_in_directory2.end(), customLess);
    
    Fill_transforms();



int files_count=0;
for (const std::string& filename : files_in_directory2)
    {
        std::string current_frame = filename;
        std::string current_frame_name = current_frame.substr(current_frame.find_last_of("/\\") + 1);
        std::string frame = current_frame_name.substr(0, current_frame_name.find_last_of('.'));
        Transform_file.open(All_Vehicles_Leader_Transforms_Folder+"/"+current_frame_name);
        for (int i=0;i<number_of_frames;i++)
        {
            Eigen::Matrix4f vehicle_trans = Get_Transform();
            Eigen::Vector3f top_three_last_column = vehicle_trans.col(3).head<3>();
            lists[files_count][i] = std::make_tuple(top_three_last_column(0), top_three_last_column(1), top_three_last_column(2));

       	}
        Transform_file.close();
        files_count++;
    }
	    cout<<"New Completed: "<<endl;

// int global_count=0;
// for (const std::string& filename : files_in_directory1)
// {

//         centroids=filtered_positions;
//         float accumulate_time=0;
//         std::string current_frame = filename;
//         std::string current_frame_name = current_frame.substr(current_frame.find_last_of("/\\") + 1);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr Vehicle_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     if (pcl::io::loadPCDFile<pcl::PointXYZ> (Vehicle_PCDs_Path+"/" +current_frame_name, *Vehicle_cloud) == -1) //* load the file
//     {
//         PCL_ERROR ("Couldn't read file  \n");
//         return (-1);
//     }

//     pcl::PointCloud<pcl::PointXYZ>::Ptr Vehicle_cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
//     if (pcl::io::loadPCDFile<pcl::PointXYZ> (Vehicle_PCDs_Path+"/" +current_frame_name, *Vehicle_cloud_temp) == -1) //* load the file
//     {
//         PCL_ERROR ("Couldn't read file  \n");
//         return (-1);
//     }

//     Eigen::Matrix4f leader_trans = World_Transforms[0][global_count];
//     Eigen::Matrix4f vehicle_trans = World_Transforms[1][global_count];
//     pcl::transformPointCloud(*Vehicle_cloud, *Vehicle_cloud, vehicle_trans);
//     pcl::transformPointCloud(*Vehicle_cloud_temp, *Vehicle_cloud_temp, vehicle_trans);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr x_cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>()); 
//     pcl::PassThrough<pcl::PointXYZ> x_pass_x;
//     x_pass_x.setInputCloud(Vehicle_cloud);
//     x_pass_x.setFilterFieldName("x");
//     x_pass_x.setFilterLimits(xmin, xmax);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr x_cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PassThrough<pcl::PointXYZ> x_pass_y; 
//     x_pass_y.setInputCloud(x_cloud_filtered_x);
//     x_pass_y.setFilterFieldName("y");
//     x_pass_y.setFilterLimits(-100, 100);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr y_cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PassThrough<pcl::PointXYZ> y_pass_y;
//     y_pass_y.setInputCloud(Vehicle_cloud);
//     y_pass_y.setFilterFieldName("y");
//     y_pass_y.setFilterLimits(ymin, ymax);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr y_cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PassThrough<pcl::PointXYZ> y_pass_x;
//     y_pass_x.setInputCloud(y_cloud_filtered_y);
//     y_pass_x.setFilterFieldName("x");
//     y_pass_x.setFilterLimits(-150, 50);

//     run_time_start = clock();
//     x_pass_x.filter(*x_cloud_filtered_x);
//     x_pass_y.filter(*x_cloud_filtered_y);
//     run_time_end = clock();

//     run_time_start1=clock();
//     y_pass_y.filter(*y_cloud_filtered_y);
//     y_pass_x.filter(*y_cloud_filtered_x);
//     run_time_end1 = clock();

//     time1 = (run_time_end - run_time_start)+(run_time_end1 - run_time_start1);
//     time1 = (((float)(time1)/CLOCKS_PER_SEC)*1000);
//     accumulate_time=accumulate_time+time1;

//     pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>());
//     run_time_start = clock();
//     x_cloud_filtered_y->reserve(x_cloud_filtered_y->size() + y_cloud_filtered_x->size());
//     (*x_cloud_filtered_y) += (*y_cloud_filtered_x);

//     pcl::PassThrough<pcl::PointXYZ> xy_pass_z;
//     xy_pass_z.setInputCloud(x_cloud_filtered_y);
//     xy_pass_z.setFilterFieldName("z");
//     xy_pass_z.setFilterLimits(-0.1, 4);
//     xy_pass_z.filter(*xy_cloud_filtered_z);

//     run_time_end = clock();
//     time2 = run_time_end - run_time_start;
//     time2 = (((float)(time2)/CLOCKS_PER_SEC)*1000);
//     accumulate_time=accumulate_time+time2;
//     Vehicle_cloud = xy_cloud_filtered_z;

//     // Vehicle_cloud->width =  Vehicle_cloud->points.size();  // Set width to the number of points
//     // Vehicle_cloud->height = 1; 
//     // pcl::io::savePCDFileASCII("Vehicles_Driviable_Space/Vehicle_"+std::to_string(static_cast<int>(Vehicle_number))+"/"+current_frame_name, * Vehicle_cloud);


// //***************************************** Dynamic Objects and Non-Road Points Extraction**********************************************  

//             //*************************** Dynamic Objects Extraction*********************

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled_map(new pcl::PointCloud<pcl::PointXYZ>()); 
//     pcl::PointCloud<pcl::PointXYZ>::Ptr changes(new pcl::PointCloud<pcl::PointXYZ>());      
//     pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
//     voxel_grid.setInputCloud(Combined);
//     voxel_grid.setLeafSize(0.25f, 0.25f, 0.25f); 
//     voxel_grid.filter(*cloud_downsampled_map);
//     double distance_threshold = 0.7; 
//     detectChangesCUDA(cloud_downsampled_map, Vehicle_cloud, changes, distance_threshold);

//     accumulate_time=accumulate_time+time3;
    
//     float leader_finding_time;
//     float min_distance_intersection=0;
//     run_time_start= clock();
//     for (int i=0;i<number_of_vehicles;i++)
//     {
//         auto [x, y, z] = lists[i][global_count];
//         auto temp=computeDist3(x,y,z,Instersection_x,Intersection_y, Intersection_z);
//         if (temp<min_distance_intersection)
//         {
//             min_distance_intersection=temp;
//         }
//     }
//     run_time_end= clock();
//     leader_finding_time= run_time_end - run_time_start;
//     leader_finding_time = (((float)(leader_finding_time)/CLOCKS_PER_SEC)*1000);
//     accumulate_time=accumulate_time+leader_finding_time;

//     std::vector<pcl::PointXYZ> positions_list;
//     for (int i=0;i<number_of_vehicles;i++)
//     {
//         auto [x, y, z] = lists[i][global_count];
//         positions_list.push_back(pcl::PointXYZ(x,y,z));
//     }

//     Eigen::Matrix4f vehicle_trans_inverse = vehicle_trans.inverse();

//     pcl::transformPointCloud(*changes, *changes, vehicle_trans_inverse);
//     pcl::transformPointCloud(*changes, *changes, leader_trans);

//     // changes->width = changes->points.size();  // Set width to the number of points
//     // changes->height = 1;  // Unorganized point cloud
//     // pcl::io::savePCDFileASCII("Vehicles_Only/Vehicle_"+std::to_string(static_cast<int>(Vehicle_number))+"/"+current_frame_name, *changes);


// Eigen::Matrix4f composite_transform = leader_trans * vehicle_trans_inverse;
// for (auto& point : centroids) {
//     Eigen::Vector4f homogenous_point(point.x, point.y, point.z, 1.0f);
//     Eigen::Vector4f transformed_point = composite_transform * homogenous_point;
//     point.x = transformed_point.x();
//     point.y = transformed_point.y();
//     point.z = transformed_point.z();
//     }


//**********************************************Dynamic Objects Cells Assignment**************************************************************

// float position_x=leader_trans(0,3);
// float position_y=leader_trans(1,3);
// float position_z=leader_trans(2,3);

// int cloud_size = changes->points.size();
// int centroids_size = centroids.size();

// // Allocate memory on the device
// pcl::PointXYZ* d_cloud_points;
// pcl::PointXYZ* d_centroids;
// int* d_assignments;
// cudaMalloc((void**)&d_cloud_points, cloud_size * sizeof(pcl::PointXYZ));
// cudaMalloc((void**)&d_centroids, centroids_size * sizeof(pcl::PointXYZ));
// cudaMalloc((void**)&d_assignments, cloud_size * sizeof(int));

// // Copy data to device
// cudaMemcpy(d_cloud_points, changes->points.data(), cloud_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
// cudaMemcpy(d_centroids, centroids.data(), centroids_size * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

// // Launch CUDA kernel to assign points to centroids
// threads_per_block = 256;
// num_blocks = (cloud_size + threads_per_block - 1) / threads_per_block;
// run_time_start = clock();
// assignPointsToPositions<<<num_blocks, threads_per_block>>>(d_cloud_points, cloud_size, d_centroids, centroids_size, d_assignments);
// cudaDeviceSynchronize();
// run_time_end = clock();
// time4 = run_time_end - run_time_start;
// time4 = (((float)(time4)/CLOCKS_PER_SEC)*1000);
// accumulate_time = accumulate_time + time4;

// error = cudaGetLastError();
// if (error != cudaSuccess) {
//     std::cerr << "CUDA kernel failed: " << cudaGetErrorString(error) << std::endl;
//     return -1;
// }

// std::vector<int> assignments(cloud_size);
// cudaMemcpy(assignments.data(), d_assignments, cloud_size * sizeof(int), cudaMemcpyDeviceToHost);

// // Free device memory
// cudaFree(d_cloud_points);
// cudaFree(d_centroids);
// cudaFree(d_assignments);


// pcl::PointCloud<pcl::PointXYZ>::Ptr assigned_centroids(new pcl::PointCloud<pcl::PointXYZ>());

// run_time_start= clock();

// std::vector<int> point_counts(centroids.size(), 0);
// for (int i = 0; i < cloud_size; ++i) {
//     int centroid_idx = assignments[i];  
//     if (centroid_idx >= 0 && centroid_idx < centroids.size()) {
//         point_counts[centroid_idx]++;  
//     }
// }

// for (size_t i = 0; i < centroids.size(); ++i) 
// {
//     if (point_counts[i] > 0) 
//     {  
//         assigned_centroids->points.push_back(centroids[i]);
//     }
// }

// run_time_end = clock();
// time5 = run_time_end - run_time_start;
// time5 = (((float)(time5)/CLOCKS_PER_SEC)*1000);
// accumulate_time = accumulate_time + time5;


// std::stringstream text_filename;
// std::string new_filename = removePCDExtension(current_frame_name);
// text_filename << "Vehicles_Driviable_Space/Vehicle_" << static_cast<int>(Vehicle_number) 
//               << "/" << new_filename << ".txt";
// std::ofstream assigned_centroids_file(text_filename.str());

// if (!assigned_centroids_file.is_open()) {
//     std::cerr << "Error: Could not open file " << text_filename.str() << " for writing." << std::endl;
// } 
// else {

// for (size_t i = 0; i < centroids.size(); ++i) {
//     if (point_counts[i] > 0) {
//         assigned_centroids_file << i << "\n"; 
//     }
// }

// assigned_centroids_file.close();
// }

//**********************************************Producer Function******************************************************************************

// std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> centroid_clouds(centroids.size());
// for (size_t i = 0; i < centroids.size(); ++i) {
//     centroid_clouds[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
// }

// for (int i = 0; i < cloud_size; ++i) {
//     int centroid_idx = assignments[i];  // Get assigned centroid index
//     if (centroid_idx >= 0 && centroid_idx < centroids.size()) {
//         centroid_clouds[centroid_idx]->push_back(changes->points[i]);
//     }
// }

// pcl::PointCloud<pcl::PointXYZ>::Ptr frame_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
// for (int vehicle = 0; vehicle < NUM_VEHICLES; ++vehicle) {
//         std::string current_vehicle_folder = "Vehicle_" + std::to_string(Vehicle_number);
//         std::string frame_file = std::to_string(global_count) + ".txt";
//         std::vector<int> requested_cells;  // Store requested cell numbers
//         if (vehicle == Vehicle_number) continue;
//         std::string vehicle_folder = "Vehicle_" + std::to_string(vehicle);
//         std::string file_path = BASE_DIR + "/" + vehicle_folder + "/" + frame_file;
//         std::ifstream infile(file_path);
//         if (!infile.is_open()) {
//             std::cerr << "Error opening file: " << file_path << std::endl;
//             continue;
//         }
//         std::string line;
//         while (std::getline(infile, line)) 
//         {
//             std::istringstream iss(line);
//             int cell;
//             std::string requesting_vehicle;

//             if (!(iss >> cell >> requesting_vehicle)) {
//                 std::cerr << "Error reading line in " << file_path << std::endl;
//                 continue;
//             }
//             std::cout<<"Checking the Data *********************** :"<<requesting_vehicle<<endl;

//             if (requesting_vehicle == current_vehicle_folder) {
//                 requested_cells.push_back(cell);
//             }
//         }
//         infile.close();

//                 for (int cell : requested_cells) {
//             if (cell >= 0 && cell < centroid_clouds.size()) {
//                 *frame_point_cloud += *centroid_clouds[cell]; // Add full point cloud of the requested cell
//             }
//         }
//     }

// if (!frame_point_cloud->empty()) 
// { 
//     std::string output_path = BASE_DIR + "/" + "Vehicle_" + std::to_string(Vehicle_number) + "/" + std::to_string(global_count) + ".pcd";
//     pcl::io::savePCDFileBinary(output_path, *frame_point_cloud);
//     std::cout << "Saved " << output_path << " with " << frame_point_cloud->size() << " points." << std::endl;
// } 

// else {
//     std::cout << "Skipped writing empty point cloud for Vehicle_" + std::to_string(Vehicle_number) + "  frame " << global_count<< "." << std::endl;
// }

   

//**********************************************Blindspots Estimation*************************************************************************

// pcl::transformPointCloud(*Vehicle_cloud, *Vehicle_cloud, vehicle_trans_inverse);
// pcl::transformPointCloud(*Vehicle_cloud, *Vehicle_cloud, leader_trans);

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

// {
//     std::vector<int> zero_centroids;
//     int gridSize = vehicle_pd.size();
    
//     int *d_vehicle_pd = nullptr, *d_zero_centroids = nullptr;
//     cudaMalloc(&d_vehicle_pd, gridSize * sizeof(int));
//     cudaMalloc(&d_zero_centroids, gridSize * sizeof(int));

//     cudaMemcpy(d_vehicle_pd, vehicle_pd.data(), gridSize * sizeof(int), cudaMemcpyHostToDevice);

//     int threadsPerBlock = 256;
//     int blocksPerGrid = (gridSize + threadsPerBlock - 1) / threadsPerBlock;
//     run_time_start = clock();
//     findZeroCentroids<<<blocksPerGrid, threadsPerBlock>>>(d_vehicle_pd, d_zero_centroids, gridSize);
//     cudaDeviceSynchronize();
//     run_time_end = clock();
//     time8 = run_time_end - run_time_start;
//     time8 = (((float)(time8)/CLOCKS_PER_SEC)*1000);
//     accumulate_time=accumulate_time+time8;

//     std::vector<int> zero_centroids_mask(gridSize);
//     cudaMemcpy(zero_centroids_mask.data(), d_zero_centroids, gridSize * sizeof(int), cudaMemcpyDeviceToHost);

//     cudaFree(d_vehicle_pd);
//     cudaFree(d_zero_centroids);
    
//     std::stringstream text_filename;
//     text_filename << "BlindSpots/Vehicle_" << static_cast<int>(Vehicle_number) << "/" << new_filename << ".txt";
//     std::ofstream zero_centroids_file(text_filename.str());

//     if (!zero_centroids_file.is_open()) {
//         std::cerr << "Error: Could not open file " << text_filename.str() << " for writing." << std::endl;
//     } 
//     else {
//         for (size_t i = 0; i < gridSize; ++i) {
//             if (zero_centroids_mask[i] == 1) {
//                 float dx = centroids[i].x - position_x;
//                 float dy = centroids[i].y - position_y;
//                 float dz = centroids[i].z - position_z;
//                 float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

//                 if (distance > 20.0f) {  // Keep only centroids beyond 5 meters
//                     zero_centroids_file << i << "\n";  // Write only the index
//                 }
//             }
//         }

//         zero_centroids_file.close();
//     }
// }

// }

//**********************************************Objects Tracking*************************************************************************

//     std::vector<pcl::PointXYZ> prev_centroids, curr_centroids;
//     std::vector<float> prev_densities, curr_densities;

//    run_time_start = clock();
//     computeCentroidsAndDensities(changes, assigned_centroids, prev_centroids, prev_densities);
//     computeCentroidsAndDensities(changes, assigned_centroids, curr_centroids, curr_densities);
//     Eigen::MatrixXf affinityMatrix = computeAffinityMatrix(prev_centroids, curr_centroids, prev_densities, curr_densities);
//     std::vector<int> assignments1 = assignTracks(affinityMatrix);
//     run_time_end = clock();
//     time9 = run_time_end - run_time_start;
//     time9 = (((float)(time9)/CLOCKS_PER_SEC)*1000);
//     accumulate_time=accumulate_time+time9;


//**********************************************Producers Sorting**************************************************************
// run_time_start = clock();
// auto distanceComparator = [&](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
//     float dist_a = std::pow(a.x - position_x, 2) + std::pow(a.y - position_y, 2) + std::pow(a.z - position_z, 2);
//     float dist_b = std::pow(b.x - position_x, 2) + std::pow(b.y - position_y, 2) + std::pow(b.z - position_z, 2);
//     return dist_a < dist_b;
// };

// std::sort(positions_list.begin(), positions_list.end(), distanceComparator);

// run_time_end = clock();
// time10 = run_time_end - run_time_start;
// time10 = (((float)(time10)/CLOCKS_PER_SEC)*1000);
// accumulate_time=accumulate_time+time10;


//***************************************************Producers Assignment******************************************************************

        for (int v = 0; v < NUM_VEHICLES; ++v) {
        std::string vehicleDir = "Vehicle_" + std::to_string(v);

        std::string latencies_file = "Producer_Assignment_Latencies/" + std::to_string(v)+ ".txt";
        std::ofstream file1(latencies_file);

        for (int frame = 1; frame <= NUM_FRAMES; ++frame) {
            std::string frameFile = std::to_string(frame) + ".txt";
            std::string vehicleFilePath = DIR_SOURCE + "/" + vehicleDir + "/" + frameFile;

            std::unordered_set<float> vehicleValues = readValuesFromFile(vehicleFilePath);
            if (vehicleValues.empty()) continue;

            std::vector<std::pair<float, std::string>> matches;
  
            double total_compute_time = 0.0;
            for (int v_compare = 0; v_compare < NUM_VEHICLES; ++v_compare) {
                if (v_compare == v) continue;

                std::string compareDir = "Vehicle_" + std::to_string(v_compare);
                std::string compareFilePath = DIR_COMPARE + "/" + compareDir + "/" + frameFile;

                std::unordered_set<float> compareValues = readValuesFromFile(compareFilePath);
                if (compareValues.empty()) continue;

                auto start_time = std::chrono::high_resolution_clock::now();
                // Check for matches and remove them from vehicleValues immediately
                for (auto it = vehicleValues.begin(); it != vehicleValues.end();) {
                    float val = *it;
                    if (compareValues.find(val) != compareValues.end()) {
                        matches.emplace_back(val, compareDir);  // Add match
                        it = vehicleValues.erase(it);           // Remove the matched value from vehicleValues
                    } 
                    else {
                        ++it;
                    }
                }
            auto end_time = std::chrono::high_resolution_clock::now();
            double frame_compute_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            total_compute_time += frame_compute_time;
            }
            file1<<total_compute_time<<endl;
            // Stop computation timer

            std::string outputFilePath = DIR_OUTPUT + "/" + vehicleDir + "/" + frameFile;
            fs::create_directories(DIR_OUTPUT + "/" + vehicleDir);
            writeMatchesToFile(outputFilePath, matches);
            writeRemainingValuesToFile(vehicleFilePath, vehicleValues);
        }
         file1.close();
    }


//*****************************************************************************************************************************************
    //log_file<<leader_finding_time<<","<<time1+time2<<","<<time3+time4+time5<<","<<time6+time7+time8<<","<<time9<<","<<time10<<","<<accumulate_time<<endl;
    // std::cout << "Leader Selection: " << leader_finding_time << std::endl;
    // std::cout << "Road Filter Time: " << time1 + time2 << std::endl;
    // std::cout << "Dynamic Objects Extraction Time: " << time3 << std::endl;
    // std::cout << "Vehicle Points Assignment Time: " << time4+time5 << std::endl;
    // std::cout << "BlindSPots Calculation Time: " << time6+time7+time8 << std::endl;
    // std::cout << "Tracking Time: "<<time9<<endl;
    // std::cout << "Accumulated Time: " << accumulate_time << std::endl;
//     global_count++;  
// }

return 0;
}


int main (int argc, char** argv)
{

    dataset_folder =argv[1];
    Map = argv[2]; 
    Vehicle_PCDs_Path=argv[3];
    V_Map_transform_file = argv[4];
    V_Leader_transform_file = argv[5];
    All_Vehicles_Leader_Transforms_Folder=argv[6];
    Output_file = argv[7];
    BlindSpots_File = argv[8];
    Vehicle_number = std::stoi(argv[9]);
    BlindSpot_Size_Folder = argv[10];
    
    Vehicle_PCDs_Path=dataset_folder +"/"+ Vehicle_PCDs_Path;
    V_Map_transform_file=dataset_folder +"/"+ V_Map_transform_file+".txt";
    V_Leader_transform_file=dataset_folder +"/"+V_Leader_transform_file+".txt";
    All_Vehicles_Leader_Transforms_Folder=dataset_folder +"/"+All_Vehicles_Leader_Transforms_Folder;
    Output_file= dataset_folder +"/"+Output_file+".csv";

    BlindSpots_File= dataset_folder +"/"+BlindSpots_File+".txt";
    BlindSpot_Size_Folder=dataset_folder +"/"+BlindSpot_Size_Folder+".txt";
   // V_leader_file.open(V_Leader_transform_file);
   // V_map_file.open(V_Map_transform_file);
    log_file.open(Output_file);
    BB_file.open(BlindSpots_File); // Open the second file
    BB_size_file.open(BlindSpot_Size_Folder);
    // log_file<<"Overlap_Estimation_Latency"<<endl;
    log_file<<"Leader_Selection"<<","<<"Drivable_Space_Estimation"<<","<<"Dynamic_Objects_Extraction"<<"BlindSPots_Estimation"<<","<<"Object_Tracking"<<","<<"Producers_Sorting"<<","<<"E2E_Latency"<<endl;

    // cout<<"Vehicle Path: "<<Vehicle_PCDs_Path<<endl;
    // cout<<"V_Map_transform_file: "<<V_Map_transform_file<<endl;
    // cout<<"V_Leader_transform_file: "<<V_Leader_transform_file<<endl;
    // cout<<"All_Vehicles_Leader_Transforms_Folder: "<<All_Vehicles_Leader_Transforms_Folder<<endl;
    // cout<<"Latencies File: "<< Output_file<<endl;
    // cout<<"BlindSpots File: "<< BlindSpots_File<<endl;
    // cout<<"Vehicle_Number: "<< Vehicle_number<<endl;
    //  BB_file.open("BB.txt"); // Open the second file
     Intersection_Points_and_Grid_Estimation();
     BB_file.close();
     log_file.close();
     BB_size_file.close();

    return (0);
}

