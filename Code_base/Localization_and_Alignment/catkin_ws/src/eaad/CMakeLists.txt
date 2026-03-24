# %Tag(FULLTEXT)%
cmake_minimum_required ( VERSION 3.13 )
project ( eaad )

# Set flags
if(NOT MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
        add_definitions(-D_WIN32_WINNT=0x600)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lboost_system")
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17) 
find_package ( catkin REQUIRED COMPONENTS
				roscpp
				rospy
				std_msgs
				genmsg
				pcl_conversions
				pcl_ros
				rosbag
				message_generation
				cv_bridge
)

find_package( OpenCV REQUIRED )
find_package( CUDA 10 REQUIRED )
find_package ( PCL 1.11 REQUIRED)

# Use the following to add custom ROS messages
#add_message_files ( FILES
	# perception_msg.msg
	# control_msg.msg
	# Added for pedestrians
	# PoseStampedArray.msg
	# MapPointCloud.msg
	#	)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
## Declare a catkin package
catkin_package ( CATKIN_DEPENDS message_runtime )

include_directories ( 
					${PROJECT_SOURCE_DIR}/src/perception/include
					${PROJECT_SOURCE_DIR}/src/perception/src
					${PROJECT_SOURCE_DIR}/src/pc_publisher/src
                    ${PROJECT_SOURCE_DIR}/src/pc_publisher/include
					${PROJECT_SOURCE_DIR}/include
					${catkin_INCLUDE_DIRS} 
					${PCL_INCLUDE_DIRS}
					${CUDA_INCLUDE_DIRS} # Cuda libraries
					${PROJECT_SOURCE_DIR}/src/perception/cub-1.8.0 # Cuda cub files
					)

############################################
############################################
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# We need boost libraries
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
	message(FATAL_ERROR "Can't find libboost_serialization")
endif()

find_library(BOOST_SYSTEM boost_system)
if (NOT BOOST_SYSTEM)
	message(FATAL_ERROR "Can't find libboost_serialization")
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()



# Eaad Point Cloud Publisher Node
add_executable (point_cloud_publisher src/pc_publisher/src/point_cloud_publisher.cpp)
target_link_libraries(point_cloud_publisher ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

# Eaad Point Cloud Stitching Node
add_executable (point_cloud_stitcher src/pc_publisher/src/stitching_node.cpp)
target_link_libraries(point_cloud_stitcher ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

# Eaad Point Cloud Stitching Node
add_executable (ndt src/pc_publisher/src/NDT.cpp)
target_link_libraries(ndt ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (vfh src/pc_publisher/src/VFH.cpp)
target_link_libraries(vfh ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (icp_cpp src/pc_publisher/src/ICP_CPP.cpp)
target_link_libraries(icp_cpp ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (plane_segmentation src/pc_publisher/src/Plane_segmentation.cpp)
target_link_libraries(plane_segmentation ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (Pcd_croper src/pc_publisher/src/Pcd_croper.cpp)
target_link_libraries(Pcd_croper ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (Boundryextraction src/pc_publisher/src/Boundryextraction.cpp)
target_link_libraries(Boundryextraction ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (NormalEstimation src/pc_publisher/src/Estimate_Normals.cpp)
target_link_libraries(NormalEstimation ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (ndt_update src/pc_publisher/src/vvr_ndt.cpp)
target_link_libraries(ndt_update ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (Diff_PCD_Octree src/pc_publisher/src/Diff_PCD_Octree.cpp)
target_link_libraries(Diff_PCD_Octree ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (Diff_PCD_KDTree src/pc_publisher/src/Diff_PCD_KDTree.cpp)
target_link_libraries(Diff_PCD_KDTree ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (transform_I src/pc_publisher/src/transform.cpp)
target_link_libraries(transform_I ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (Final_merge src/pc_publisher/src/merge.cpp)
target_link_libraries(Final_merge ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (Downsample src/pc_publisher/src/Downsample.cpp)
target_link_libraries(Downsample ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (pc_imu_publisher src/pc_publisher/src/pc_imu_publisher.cpp)
target_link_libraries(pc_imu_publisher ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable (pcd_transformer src/pc_publisher/src/pcd_transformer.cpp)
target_link_libraries(pcd_transformer ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

add_executable(pcd_to_pointcloud2_pub src/pc_publisher/src/pcd_to_pointcloud2_pub.cpp)
target_link_libraries(pcd_to_pointcloud2_pub ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES})

# # NetDriving Perception Node
# cuda_add_executable ( perception 
# 			src/perception/src/main.cpp
# 			src/perception/src/cloud_filtering.cpp
# 			src/perception/src/cloud_association.cpp
# 			src/perception/src/cloud_stitcher.cpp
# 			src/perception/src/cloud_clustering.cpp
# 			src/perception/src/estimation_mv.cpp
# 			src/perception/src/estimation_hv.cpp 
# 			src/perception/src/estimation_nv.cpp
# 			src/perception/src/CudaICP.cu
# 			src/perception/src/background_subtraction.cpp
# 			src/perception/src/utils.cpp
# 			src/perception/src/obtain_background.cpp
# 		     )
# target_link_libraries ( perception 
# 			${catkin_LIBRARIES} 
# 			${BOOST_SYSTEM} 
# 			${PCL_LIBRARIES}
# 			${OpenCV_LIBRARIES}
# 			cublas cusolver
# 			)
# add_dependencies ( perception ${catkin_EXPORTED_TARGETS} )



### Node for Net-driving Planning and Control
# add_executable ( control
# 				src/control/src/main.cpp )
# target_link_libraries ( control ${catkin_LIBRARIES} ${BOOST_SYSTEM} ${PCL_LIBRARIES} )
# add_dependencies ( control ${catkin_EXPORTED_TARGETS} )

# catkin_install_python ( PROGRAMS src/low_level_controller/low_level_controller.py
    #    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )


### Node for Net-driving without Background Filtering
# # NetDriving Perception WO Background Filtering Node
# cuda_add_executable ( perception_wo_bf
# 		src/perception/src/main_wo_bf.cpp
# 		src/perception/src/cloud_association.cpp
# 		src/perception/src/cloud_stitcher.cpp
# 		src/perception/src/cloud_clustering.cpp
# 		src/perception/src/estimation_mv.cpp
# 		src/perception/src/estimation_hv.cpp
# 		src/perception/src/estimation_nv.cpp
# 		src/perception/src/CudaICP.cu
# 		)
# target_link_libraries ( perception_wo_bf
# 		${catkin_LIBRARIES}
# 		${BOOST_SYSTEM}
# 		${PCL_LIBRARIES}
# 		${OpenCV_LIBRARIES}
# 		cublas cusolver
# 		)
# add_dependencies ( perception_wo_bf ${catkin_EXPORTED_TARGETS} )


### Node for Net-driving with Background Filtering
# NetDriving Perception Background Filtering Node
# cuda_add_executable ( perception_bf
# 		src/perception/src/main_bf.cpp
# 		src/perception/src/cloud_filtering.cpp
# 		src/perception/src/background_subtraction.cpp
# 		src/perception/src/utils.cpp
# 		src/perception/src/obtain_background.cpp
# 		)
# target_link_libraries ( perception_bf
# 		${catkin_LIBRARIES}
# 		${BOOST_SYSTEM}
# 		${PCL_LIBRARIES}
# 		${OpenCV_LIBRARIES}
# 		cublas cusolver
# 		)
# add_dependencies ( perception_bf ${catkin_EXPORTED_TARGETS} )


### Online Net-driving Nodes
# NetDriving Perception Node for online Ouster perception
# cuda_add_executable ( online_ouster_perception
#                        src/perception/src/online_ouster_perception.cpp
#                        src/perception/src/cloud_filtering.cpp
#                        src/perception/src/cloud_association.cpp
#                        src/perception/src/cloud_stitcher.cpp
#                        src/perception/src/cloud_clustering.cpp
#                        src/perception/src/estimation_mv.cpp
#                        src/perception/src/estimation_hv.cpp
#                        src/perception/src/estimation_nv.cpp
#                        src/perception/src/CudaICP.cu
#                     )
# target_link_libraries ( online_ouster_perception
#                        ${catkin_LIBRARIES}
#                        ${BOOST_SYSTEM}
#                        ${PCL_LIBRARIES}
#                        ${OpenCV_LIBRARIES}
#                        cublas cusolver
#                        )
# add_dependencies ( online_ouster_perception ${catkin_EXPORTED_TARGETS} )





# add_executable(headset_pose_estimation src/pedestrians/src/headset_pose_estimation.cpp
# 				src/pedestrians/src/common.cpp
# 				src/pedestrians/src/point_matching.cpp
# 				src/pedestrians/src/trajectory.cpp
# 				src/pedestrians/src/trajectory_manager.cpp
# 				src/pedestrians/src/perception_manager.cpp
# 				src/pedestrians/src/imu_manager.cpp
# 				src/pedestrians/src/slam_map_manager.cpp
# 				src/pedestrians/src/image_manager.cpp
# 				src/pedestrians/src/stereo_matching.cpp
# 				src/pedestrians/src/dynamic_cloud_manager.cpp
# 				)
# target_link_libraries ( headset_pose_estimation
#                        ${catkin_LIBRARIES}
#                        ${BOOST_SYSTEM}
#                        ${PCL_LIBRARIES}
#                        ${OpenCV_LIBRARIES}
#                        )
# add_dependencies ( headset_pose_estimation ${catkin_EXPORTED_TARGETS} )


# find_package(realsense2 REQUIRED)
# add_executable(realsense_trajectory src/pedestrians/src/realsense_trajectory.cpp
# 				src/pedestrians/src/trajectory_manager.cpp
# 				)
# target_link_libraries ( realsense_trajectory
#                        ${catkin_LIBRARIES}
#                        ${BOOST_SYSTEM}
#                        ${PCL_LIBRARIES}
#                        ${OpenCV_LIBRARIES}
# 					   ${realsense2_LIBRARIES}
#                        )
# add_dependencies ( realsense_trajectory ${catkin_EXPORTED_TARGETS} )

# add_executable(camera_projection src/pedestrians/camera_projection.cpp)
# target_link_libraries ( camera_projection
#                        ${catkin_LIBRARIES}
#                        ${BOOST_SYSTEM}
#                        ${PCL_LIBRARIES}
#                        ${OpenCV_LIBRARIES}
#                        )
# add_dependencies ( camera_projection ${catkin_EXPORTED_TARGETS} )


# NetDriving Perception Node
# cuda_add_executable ( perception_peds 
# 			src/perception/src/perception_peds.cpp
# 			src/perception/src/cloud_filtering.cpp
# 			src/perception/src/cloud_association.cpp
# 			src/perception/src/cloud_stitcher.cpp
# 			src/perception/src/cloud_clustering.cpp
# 			src/perception/src/estimation_mv.cpp
# 			src/perception/src/estimation_hv.cpp 
# 			src/perception/src/estimation_nv.cpp
# 			src/perception/src/CudaICP.cu
# 		     )
# target_link_libraries ( perception_peds 
# 			${catkin_LIBRARIES} 
# 			${BOOST_SYSTEM} 
# 			${PCL_LIBRARIES}
# 			${OpenCV_LIBRARIES}
# 			cublas cusolver
# 			)
# add_dependencies ( perception_peds ${catkin_EXPORTED_TARGETS} )


