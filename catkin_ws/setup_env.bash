#!/bin/bash

# Common parameters
# Load eaad package
source devel/setup.bash
# Set the mode of dataset [carla, real_world]
rosparam set /nd_mode carla

#### Point cloud publisher
# Set path to pcd files
rosparam set /pc_path_1 /dataset/Vehicle/
rosparam set /pc_path_2 /dataset/Infrastructure/
rosparam set /imu_path /dataset/V_IMU.txt
rosparam set /vehicle_timestamp_path /dataset/Vehicle_timestamps.txt

# parameters for V2V starts
rosparam set use_sim_time false

rosparam set /map_path /dataset/CarLA_Map_V2V_FASTLIO.pcd
rosparam set /vehicle_1_pc_topic /v2v/vehicle_1_points
rosparam set /vehicle_1_transform_topic /v2v/v1_transform
rosparam set /vehicle_1_initial_guess_path /dataset/Initial_guess_1.txt

rosparam set /vehicle_2_pc_topic /v2v/vehicle_2_points
rosparam set /updated_v2_pc_topic /v2v/v2_updated
rosparam set /vehicle_2_transform_topic /v2v/v2_transform
rosparam set /vehicle_2_initial_guess_path /dataset/Initial_guess_2.txt

# parameters for V2V ends



# # parameters added by ali start 


# dataset_path="/dataset/04-21-22"
# dataset_subset_path='V32_I32_120m'
# dataset_name='D4'
# vehicle_folder="vehicle_lidar_2"
# infra_folder="infra_lidar_3"
# rosparam set /vehicle_initial_guess_path $dataset_path/V_W.txt
# rosparam set /vehicle_ndt_transform_path $dataset_path/V_W.txt
# rosparam set /infra_initial_guess_path $dataset_path/I_W.txt


# rosparam set use_sim_time false


# #### Vehicle NDT alignment
# # Inputs
# rosparam set /map_path $dataset_path/basemap.pcd
# rosparam set /vehicle_pc_topic /autopass/vehicle_points
# # Outputs
# rosparam set /aligned_pc_topic /autopass/ndt_vehicle_points
# rosparam set /map_pc_topic /autopass/map_points
# rosparam set /vehicle_transform_topic /autopass/transform


# #### Infra 
# # Inputs
# rosparam set /infra_static_pc_path $dataset_path/infra_static.pcd
# rosparam set /infra_pc_topic /autopass/infra_points
# rosparam set /infra_pc_topic_2 /autopass/infra_points_2
# # Outputs
# rosparam set /diff_pc_topic /autopass/diff_points
# rosparam set /diff_compress_topic /autopass/diff_compress


# #### Vehicle Fusion  
# # Inputs
# # same as other two nodes
# # Outputs
# rosparam set /infra_reconstructed_topic /autopass/infra_reconstructed_points
# rosparam set /fused_pc_topic /autopass/fused_points
# rosparam set /fused_pc_topic1 /autopass/fused_points1


# # parameters added by ali end


#### PCD Transformer
# Input
rosparam set /input_infra_pcd_topic /ouster/lidar_3_points
# Output
rosparam set /output_infra_pcd_topic /eaad/aligned_infra_points
# rosparam set /pc_path_3 /dataset/Edge-Assist/vehicle_lidar_2/
# rosparam set /pc_path_4 /dataset/Edge-Assist/vehicle_lidar_4/

# Set prefix of point cloud
# rosparam set /pc_prefix 
# Set index of first lidar frame
rosparam set /start_index 0
# Set the number of clouds to process
rosparam set /number_of_clouds 1040
# Set the number of lidars
rosparam set /number_of_lidars 1
# Set the point cloud prefix to publish on
rosparam set /pc_topic_prefix ouster
# Set the point cloud suffix to subscribe to
rosparam set /pc_topic_suffix points
# prefix and suffix (below are combined like): pc_topic_prefix + _ + id_of_lidar + / + suffix
# IMU params
rosparam set /imu_topic ouster/imu
#rosparam set /imu_freq 100
rosparam set /imu_freq 10

#### Point cloud stitching
rosparam set vehicle_point_cloud /dataset/Edge-Assist/04-21-2022/vehicle_1/2022-04-21-22-22/2022-04-21-22-22-16-001-pcds/1650580926.047990528.pcd
rosparam set infra_point_cloud /dataset/Edge-Assist/04-21-2022/infra_1/2022-04-21-22-22/2022-04-21-22-22-21-006-pcds/1650580926.087735296.pcd

# # Offline mode parameters
# # Set id of first lidar
# rosparam set /lidar_id 96


# # Path to initialization components
# rosparam set /path_to_base_clouds /workspace/infra-based-sensing/catkin_ws/config/carla_config/
# # Path to transforms
# rosparam set /path_to_transforms /workspace/infra-based-sensing/catkin_ws/config/carla_config/transform.txt
# # Path to drivable space cloud
# rosparam set /drivable_space_path /workspace/infra-based-sensing/catkin_ws/config/carla_config/drivable_space_cloud.pcd

# # Control specific parameters
# # Control mode [full, partial, ego-planning]
# rosparam set /control_mode full

# #if [ ! -d "_out/" ]
# #then
# #	mkdir _out
# #fi

# #if [ ! -d "control_logs/" ]
# #then
# #	mkdir control_logs
# #fi
