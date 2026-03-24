#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

namespace fs = boost::filesystem;

int main() {
    std::string folder_path = "./Leader";                // Path to folder containing PCD files
    std::string decompressed_folder = "./Decompressed"; // Path to save decompressed point clouds
    std::string output_txt = "compression_results.txt"; // Output file for compression sizes

    // Create decompressed folder if it doesn't exist
    if (!fs::exists(decompressed_folder)) {
        fs::create_directory(decompressed_folder);
    }

    // Open output file
    std::ofstream output_file(output_txt);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open the output file: " << output_txt << std::endl;
        return -1;
    }

    output_file << "PCD File, Compressed Size (KB), Compression Time (ms), Decompression Time (ms)" << std::endl;

    // Iterate through all PCD files in the folder
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (fs::is_regular_file(entry) && entry.path().extension() == ".pcd") {
            std::string file_path = entry.path().string();
            std::string file_name = entry.path().filename().string();

            // Load the point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
                std::cerr << "Failed to load PCD file: " << file_path << std::endl;
                continue;
            }

            std::cout << "Processing " << file_name << " with " << cloud->points.size() << " points." << std::endl;

            // Compression
            std::stringstream compressedData;
            pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression(
                pcl::io::MANUAL_CONFIGURATION, false, 0.1, 5.0, false, 1, false, 8);
            auto t1 = std::chrono::high_resolution_clock::now();
            octree_compression.encodePointCloud(cloud, compressedData);
            auto t2 = std::chrono::high_resolution_clock::now();

            // Calculate compression time in milliseconds
            auto compression_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;

            // Calculate compressed size in KB
            float compressed_size_kb = compressedData.str().size() / 1024.0f;

            // Decompression
            pcl::PointCloud<pcl::PointXYZ>::Ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            auto t3 = std::chrono::high_resolution_clock::now();
            octree_compression.decodePointCloud(compressedData, decompressed_cloud);
            auto t4 = std::chrono::high_resolution_clock::now();

            // Calculate decompression time in milliseconds
            auto decompression_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count() / 1e6;

            // Save decompressed point cloud
            std::string decompressed_file_path = decompressed_folder + "/" + file_name;
            if (pcl::io::savePCDFileBinary(decompressed_file_path, *decompressed_cloud) == -1) {
                std::cerr << "Failed to save decompressed PCD file: " << decompressed_file_path << std::endl;
                continue;
            }

            std::cout << "Decompressed point cloud saved to: " << decompressed_file_path << std::endl;

            // Write results to the output file
            output_file << file_name << ", " << compressed_size_kb << ", " << compression_time << ", " << decompression_time << std::endl;
        }
    }

    // Close the output file
    output_file.close();
    std::cout << "Compression and decompression results saved to " << output_txt << std::endl;

    return 0;
}

