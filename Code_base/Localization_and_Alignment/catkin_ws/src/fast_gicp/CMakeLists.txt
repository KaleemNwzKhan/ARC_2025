cmake_minimum_required(VERSION 3.10.0)
project(fast_gicp)


option(BUILD_VGICP_CUDA "Build GPU-powered VGICP" ON)
option(BUILD_apps "Build application programs" ON)
option(BUILD_test "Build test programs" OFF)
option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)
add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17")

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
else()
  add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
  set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
  set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.11" "/usr/include/eigen3")
include_directories(${PCL_INCLUDE_DIRS})


find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Transform.msg
  CompressDiff.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
endif()

if(NOT BUILD_PYTHON_BINDINGS)
  if(${ROS_VERSION})
    if(${ROS_VERSION} EQUAL 1)
      find_package(catkin)
    elseif (${ROS_VERSION} EQUAL 2)
      find_package(ament_cmake)
    endif()
  endif()
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(BUILD_VGICP_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
  include_directories(${catkin_INCLUDE_DIRS})
endif()

###################################
## catkin specific configuration ##
###################################
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fast_gicp
    CATKIN_DEPENDS message_runtime
  )
endif()




###########
## Build ##
###########

add_library(fast_gicp SHARED
  src/fast_gicp/gicp/lsq_registration.cpp
  src/fast_gicp/gicp/fast_gicp.cpp
  src/fast_gicp/gicp/fast_gicp_st.cpp
  src/fast_gicp/gicp/fast_vgicp.cpp
)
target_link_libraries(fast_gicp
  ${PCL_LIBRARIES}
)
if (OPENMP_FOUND)
    if (TARGET OpenMP::OpenMP_CXX)
        target_link_libraries(fast_gicp OpenMP::OpenMP_CXX)
    endif ()
endif ()
target_include_directories(fast_gicp PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

### APPS ###
if(BUILD_apps)
  # add_executable(cuda_ndt_align src/align_CUDA_NDT.cpp)
  # add_dependencies(cuda_ndt_align fast_gicp)
  # target_link_libraries(cuda_ndt_align
  #   ${PCL_LIBRARIES}
  #   ${catkin_LIBRARIES}
  #   fast_gicp
  # )

  # add_executable(vehicle_ndt src/vehicle_ndt.cpp)
  # add_dependencies(vehicle_ndt fast_gicp)
  # target_link_libraries(vehicle_ndt
  #   ${PCL_LIBRARIES}
  #   ${catkin_LIBRARIES}
  #   fast_gicp
  # )



  # add_executable(vehicle_fusion src/vehicle_fusion.cpp)
  # add_dependencies(vehicle_fusion fast_gicp)
  # target_link_libraries(vehicle_fusion
  #   ${PCL_LIBRARIES}
  #   ${catkin_LIBRARIES}
  #   fast_gicp
  # )

  # add_executable(decode src/decode.cpp)
  # add_dependencies(decode fast_gicp)
  # target_link_libraries(decode
  #   ${PCL_LIBRARIES}
  #   ${catkin_LIBRARIES}
  #   fast_gicp
  # )

  # add_executable(receive_infra_raw src/receive_infra_raw.cpp)
  # add_dependencies(receive_infra_raw fast_gicp)
  # target_link_libraries(receive_infra_raw
  #   ${PCL_LIBRARIES}
  #   ${catkin_LIBRARIES}
  #   fast_gicp
  # )


  # add_executable(vehicle_fusion_test src/vehicle_fusion_test.cpp)
  # add_dependencies(vehicle_fusion_test fast_gicp)
  # target_link_libraries(vehicle_fusion_test
  #   ${PCL_LIBRARIES}
  #   ${catkin_LIBRARIES}
  #   fast_gicp
  # )


  # add_executable(vehicle_fusion_icp src/vehicle_fusion_icp.cpp)
  # add_dependencies(vehicle_fusion_icp fast_gicp)
  # target_link_libraries(vehicle_fusion_icp
  #   ${PCL_LIBRARIES}
  #   ${catkin_LIBRARIES}
  #   fast_gicp
  # )

  # add_executable(vehicle_fusion_icp_ndt src/vehicle_fusion_icp_ndt.cpp)
  # add_dependencies(vehicle_fusion_icp_ndt fast_gicp)
  # target_link_libraries(vehicle_fusion_icp_ndt
  #   ${PCL_LIBRARIES}
  #   ${catkin_LIBRARIES}
  #   fast_gicp
  # )

  #  add_executable(cuda_gicp_align src/align_CUDA_GICP.cpp)
  #  add_dependencies(cuda_gicp_align fast_gicp)
  #  target_link_libraries(cuda_gicp_align
  #    ${PCL_LIBRARIES}
  #    fast_gicp
  #  )

  # add_executable(gicp_kitti src/kitti.cpp)
  # add_dependencies(gicp_kitti fast_gicp)
  # target_link_libraries(gicp_kitti
  #   ${PCL_LIBRARIES}
  #   fast_gicp
  # )



  add_executable(fusion_pipeline_1 src/fusion_pipeline_1.cpp)
  add_dependencies(fusion_pipeline_1 fast_gicp)
  target_link_libraries(fusion_pipeline_1
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )


  add_executable(fusion_pipeline_2 src/fusion_pipeline_2.cpp)
  add_dependencies(fusion_pipeline_2 fast_gicp)
  target_link_libraries(fusion_pipeline_2
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )

  add_executable(fusion_pipeline_3 src/fusion_pipeline_3.cpp)
  add_dependencies(fusion_pipeline_3 fast_gicp)
  target_link_libraries(fusion_pipeline_3
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )

  add_executable(fusion_pipeline_4 src/fusion_pipeline_4.cpp)
  add_dependencies(fusion_pipeline_4 fast_gicp)
  target_link_libraries(fusion_pipeline_4
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )

  add_executable(fusion_pipeline_5 src/fusion_pipeline_5.cpp)
  add_dependencies(fusion_pipeline_5 fast_gicp)
  target_link_libraries(fusion_pipeline_5
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )


  add_executable(vehicle_localization_pipeline_1 src/vehicle_localization_pipeline_1.cpp)
  add_dependencies(vehicle_localization_pipeline_1 fast_gicp)
  target_link_libraries(vehicle_localization_pipeline_1
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )


  add_executable(vehicle_localization_pipeline_5 src/vehicle_localization_pipeline_5.cpp)
  add_dependencies(vehicle_localization_pipeline_5 fast_gicp)
  target_link_libraries(vehicle_localization_pipeline_5
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )



  add_executable(infra_pipeline_1 src/infra_pipeline_1.cpp)
  add_dependencies(infra_pipeline_1 fast_gicp)
  target_link_libraries(infra_pipeline_1
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )


  add_executable(infra_pipeline_2 src/infra_pipeline_2.cpp)
  add_dependencies(infra_pipeline_2 fast_gicp)
  target_link_libraries(infra_pipeline_2
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )


  add_executable(infra_pipeline_3 src/infra_pipeline_3.cpp)
  add_dependencies(infra_pipeline_3 fast_gicp)
  target_link_libraries(infra_pipeline_3
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )



  add_executable(vehicle_localization_pipeline_2 src/vehicle_localization_pipeline_2.cpp)
  add_dependencies(vehicle_localization_pipeline_2 fast_gicp)
  target_link_libraries(vehicle_localization_pipeline_2
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )

  add_executable(V2V_fusion src/V2V_fusion.cpp)
  add_dependencies(V2V_fusion fast_gicp)
  target_link_libraries(V2V_fusion
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )

  add_executable(Vehicle_Vehicle src/align_CUDA_GICP_Vehicle_Vehicle_with_Script.cpp)
  add_dependencies(Vehicle_Vehicle fast_gicp)
  target_link_libraries(Vehicle_Vehicle
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )

  add_executable(cuda_ndt_align_R2V src/align_CUDA_NDT_R2V.cpp)
  add_dependencies(cuda_ndt_align_R2V fast_gicp)
  target_link_libraries(cuda_ndt_align_R2V
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    fast_gicp
  )



endif()

### Python bindings ###
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(thirdparty/pybind11)
  pybind11_add_module(pygicp
    src/python/main.cpp
  )
  target_include_directories(pygicp PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(pygicp PRIVATE
    fast_gicp
  )
endif()

### CUDA ###
if(BUILD_VGICP_CUDA)
  set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")
  add_definitions(-DUSE_VGICP_CUDA)

  cuda_add_library(fast_vgicp_cuda SHARED
    src/fast_gicp/cuda/fast_vgicp_cuda.cu
    src/fast_gicp/cuda/brute_force_knn.cu
    src/fast_gicp/cuda/covariance_estimation.cu
    src/fast_gicp/cuda/covariance_estimation_rbf.cu
    src/fast_gicp/cuda/covariance_regularization.cu
    src/fast_gicp/cuda/gaussian_voxelmap.cu
    src/fast_gicp/cuda/find_voxel_correspondences.cu
    src/fast_gicp/cuda/compute_derivatives.cu
    src/fast_gicp/cuda/compute_mahalanobis.cu
    src/fast_gicp/cuda/ndt_cuda.cu
    src/fast_gicp/cuda/ndt_compute_derivatives.cu
  )
  target_include_directories(fast_vgicp_cuda PRIVATE
    include
    thirdparty/Eigen
    thirdparty/nvbio
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(fast_vgicp_cuda
    ${catkin_LIBRARIES}
  )
  cuda_add_cublas_to_target(fast_vgicp_cuda)

  # add vgicp_cuda to libfast_gicp
  target_sources(fast_gicp PRIVATE
    src/fast_gicp/gicp/fast_vgicp_cuda.cpp
    src/fast_gicp/ndt/ndt_cuda.cpp
  )
  target_link_libraries(fast_gicp
    fast_vgicp_cuda
  )
  add_dependencies(fast_gicp fast_vgicp_cuda)
  if(catkin_FOUND)
    install(TARGETS fast_vgicp_cuda  
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  endif()
endif()

### TEST ###
if(BUILD_test)
  find_package(GTest REQUIRED)

  add_executable(gicp_test src/test/gicp_test.cpp)
  add_dependencies(gicp_test fast_gicp)
  target_link_libraries(gicp_test ${GTEST_LIBRARIES} ${PCL_LIBRARIES} fast_gicp)
  gtest_add_tests(TARGET gicp_test WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} EXTRA_ARGS "${CMAKE_SOURCE_DIR}/data")
endif()

if(catkin_FOUND)
  ###################################
  ## catkin specific configuration ##
  ###################################
  install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  
  install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp")
elseif (ament_cmake_FOUND)
  ##################################
  ## ament specific configuration ##
  ##################################
  ament_export_include_directories(include)
  ament_export_libraries(fast_gicp)
  ament_package()

  install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib)

  install(
      DIRECTORY "include/"
      DESTINATION include
    )
endif()
