#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>  
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>
#include <omp.h> 
#include <pcl/common/common.h> //for minmax3D


// dimensions of a cell 
const float cell_x_size = 2.5;
const float cell_y_size = 2.5;
const float cell_z_size = 2.5;


#pragma omp declare reduction(vec_int_plus : std::vector<int> : \
                              std::transform(omp_out.begin(), omp_out.end(), omp_in.begin(), omp_out.begin(), std::plus<float>())) \
                    initializer(omp_priv = omp_orig)



#define NUM_PROCS 8

inline int getGridCentroidIdx( int, int, int, int, int, int );

float computeDist ( float, float );
float computeDist ( float, float, float, float, float, float );
float computeDist ( pcl::PointXYZ, pcl::PointXYZ );

inline bool isWithinBounds( pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ );
int getNearestGridCentroid( pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ );

int additional_alignment(std::string, std::string,std::string);
std::vector<int> point_density (pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ, pcl::PointXYZ);
std::vector<pcl::PointXYZ> get_grid(pcl::PointXYZ, pcl::PointXYZ);


std::ofstream log_file;

int main (int argc, char** argv)

{
    std::cout << "infra_path" <<std::endl;
    std::string dataset_folder =argv[1];
    std::string vehicle_folder = argv[2];
    std::string infra_folder = argv[3];
    std::string output_file = argv[4];
    log_file.open(output_file+".csv");
    log_file << "frame_number" << ","<< "time" <<  ","<<"ipd" << std::endl;

    additional_alignment(dataset_folder,vehicle_folder, infra_folder);

    return (0);
}



int additional_alignment(std::string dataset_folder, std::string vehicle_folder, std::string infra_folder)
{
    std::string vehicle_path = dataset_folder +"/"+ vehicle_folder;
    std::string infra_path = dataset_folder +"/"+ infra_folder;
    std::cout << vehicle_path <<std::endl;
    std::cout << infra_path <<std::endl;


    pcl::PointCloud<pcl::PointXYZI>::Ptr complete_transformed (new pcl::PointCloud<pcl::PointXYZI>);

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
            //std::cout<<af<<std::endl;
            return af < bf; 
        }
    }
    customLess;


    std::vector<std::filesystem::path> files_in_directory;
    std::copy(std::filesystem::directory_iterator(infra_path), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
    std::sort(files_in_directory.begin(), files_in_directory.end(), customLess);

    float run_time_start, run_time_end;

    for (const std::string& filename : files_in_directory) {

        std::cout << filename << std::endl; // printed in alphabetical order
        std::string current_frame = filename;
        std::cout << current_frame << std::endl;
        std::string current_frame_name = current_frame.substr(current_frame.find_last_of("/\\") + 1);
        std::cout << current_frame_name << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr infra_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr vehicle_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> infra_pd, vehicle_pd; 
        int ipd_threshold = 10 ;
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (infra_path+"/" +current_frame_name , *infra_cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file  \n");
            return (-1);
        }
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (vehicle_path+"/" +current_frame_name , *vehicle_cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file  \n");
            return (-1);
        }
        
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D (*infra_cloud, minPt, maxPt);

        std::vector<pcl::PointXYZ> grid_info = get_grid(minPt, maxPt);

        pcl::PointXYZ min_centroid = grid_info[0];
        pcl::PointXYZ max_centroid = grid_info[1];
        pcl::PointXYZ lims = grid_info[2];
        
        // std::cout << "HERE1" << std::endl;

        run_time_start = clock();
        // std::cout << "HERE2" << std::endl;
         
        // std::cout << "HERE3" << std::endl;
        
        // std::cout << "HERE4" << std::endl;

        #pragma omp parallel for num_threads(2)
        for ( int i = 0 ; i < 2 ; i++ ) {
            if ( i == 0 ) {
                infra_pd = point_density (min_centroid, lims, max_centroid, infra_cloud, minPt, maxPt);
            } else {
                vehicle_pd = point_density (min_centroid, lims, max_centroid, vehicle_cloud, minPt, maxPt);   
            }
        }

        // int ipd[infra_pd.size()];

        // std::cout << "HERE5" << std::endl;

        // //#pragma omp parallel for schedule(dynamic) default(shared) num_threads(NUM_PROCS) shared(infra_pd, vehicle_pd, ipd)
        // for (int i=0; i < infra_pd.size(); ++i){
        //     int min_pd = std::min(infra_pd[i], vehicle_pd[i]);
        //     ipd[i]=min_pd;
        // }

        // // std::cout << "HERE6" << std::endl;

        int above_threshold = 0;
        // const int ipd_size = sizeof(ipd) / sizeof(ipd[0]); 
        
        // for (int i=0; i<ipd_size; ++i) {
        //     if(ipd[i]>ipd_threshold) {
        //         above_threshold++;
        //     }
        // }


        #pragma omp parallel for num_threads(8) default(shared) reduction(+:above_threshold)
        for (int i=0; i < infra_pd.size(); ++i){
            int min_pd = (infra_pd[i] < vehicle_pd[i]) ? infra_pd[i] : vehicle_pd[i] ;
            if ( min_pd > ipd_threshold ) {
                above_threshold++;
            }
        }

        run_time_end= clock() ;
        
        log_file <<current_frame_name <<"," <<  ((float)(run_time_end-run_time_start)/CLOCKS_PER_SEC)*1000<<"," << above_threshold<<std::endl;
        

    }


    return 0;
}




std::vector<int> point_density (pcl::PointXYZ min_centroid, pcl::PointXYZ lims, pcl::PointXYZ max_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
    pcl::PointXYZ minPt, pcl::PointXYZ maxPt){


    int SZ = lims.x*lims.y*lims.z;
    std::vector<int> result (SZ+1, 0);

    // #pragma omp parallel for schedule(dynamic) default(shared) num_threads(NUM_PROCS) reduction(vec_int_plus : result) 
    for (std::size_t point_i = 0; point_i < input_cloud->size(); ++ point_i) {
        pcl::PointXYZ curr_point = (*input_cloud)[point_i];

        if ( isWithinBounds( curr_point, minPt, maxPt ) ) {
            int nearestGridCentroid = getNearestGridCentroid( curr_point, min_centroid, max_centroid, lims );
            result[nearestGridCentroid]++;
        }
    }

    return result;
}

/*
    Returns:
    [0] the min centroid of the grid
    [1] the max centroid of the grid
    [2] the step size limits
*/
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


inline int getGridCentroidIdx( int x, int y, int z, int xSz, int ySz, int zSz ) {
    return x + y * xSz + z * xSz * ySz;
}

float computeDist ( float x1, float x2 ) {
    return (x1-x2)*(x1-x2);
}

float computeDist ( float x1, float y1, float z1, float x2, float y2, float z2 ) {
    return computeDist( x1, x2 ) + computeDist( y1, y2 ) + computeDist( z1, z2 );
}

float computeDist ( pcl::PointXYZ p1, pcl::PointXYZ p2 ) {
    return computeDist( p1.x, p1.y, p1.z, p2.x, p2.y, p2.z );
}

/*
Given a possible point p, check if it is within the bounds defined by the min and max points given
*/
inline bool isWithinBounds( pcl::PointXYZ p, pcl::PointXYZ min, pcl::PointXYZ max ) {
    if ( p.x < min.x ) return false;
    if ( p.y < min.y ) return false;
    if ( p.z < min.z ) return false;
    if ( p.x > max.x ) return false;
    if ( p.y > max.y ) return false;
    if ( p.z > max.z ) return false;
    return true;
}

/*
    return index of grid centroid to which point p is nearest
*/
int getNearestGridCentroid( pcl::PointXYZ p, pcl::PointXYZ min_centroid, pcl::PointXYZ max_centroid, pcl::PointXYZ lims ) {

    int xT = (p.x - min_centroid.x)/cell_x_size;
    int yT = (p.y - min_centroid.y)/cell_y_size;
    int zT = (p.z - min_centroid.z)/cell_z_size;

    const int step = 1;

    float min_dist_to_centroid = (cell_x_size+cell_y_size+cell_z_size)*(cell_x_size+cell_y_size+cell_z_size);
    int closest_xt;
    int closest_yt;
    int closest_zt;

    // std::cout << " checking point " << p.x << ", " << p.y << ", " << p.z <<". " << std::endl;
 
    for ( int xC = xT - step ; xC <= xT + step ; xC++  ) {
        for ( int yC = yT - step ; yC <= yT + step ; yC++  ) {
            for ( int zC = zT - step ; zC <= zT + step ; zC++  ) {
                pcl::PointXYZ curr_grid_point;
                curr_grid_point.x = min_centroid.x + xC*cell_x_size;
                curr_grid_point.y = min_centroid.y + yC*cell_y_size;
                curr_grid_point.z = min_centroid.z + zC*cell_z_size;

                // std::cout << "\t checking grid point " << curr_grid_point.x << ", " 
                // << curr_grid_point.y << ", " << curr_grid_point.z <<". " << std::endl;
                // std::cout << "\t with steps " << xC << ", " 
                // << yC << ", " << zC <<". " << std::endl;

                if ( isWithinBounds ( curr_grid_point, min_centroid, max_centroid ) ) {
                    float curr_dist = computeDist( p, curr_grid_point );
                    if ( curr_dist < min_dist_to_centroid ) {
                        closest_xt = xC;
                        closest_yt = yC;
                        closest_zt = zC;
                        min_dist_to_centroid = curr_dist;
                    }
                }

            }
        }
    }
    return getGridCentroidIdx( closest_xt, closest_yt, closest_zt, lims.x, lims.y, lims.z );
   
}




