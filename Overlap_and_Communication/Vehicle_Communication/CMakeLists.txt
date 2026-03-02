cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(additional_alignment LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_ARCHITECTURES 60)

# Enable OpenMP
find_package(OpenMP REQUIRED)

# Find PCL
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (CUDA_Subscriber Subscriber.cu)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --forward-unknown-to-host-compiler -arch=sm_60 -std=c++17 ")
target_link_libraries (CUDA_Subscriber PRIVATE OpenMP::OpenMP_CXX ${PCL_LIBRARIES} )

add_executable (CUDA_Publisher Publisher.cu)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --forward-unknown-to-host-compiler -arch=sm_60 -std=c++17 ")
target_link_libraries (CUDA_Publisher PRIVATE OpenMP::OpenMP_CXX ${PCL_LIBRARIES} )

add_executable (CUDA_Publisher_Assignment Subscriber_Publisher_Assignment.cu.cu)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --forward-unknown-to-host-compiler -arch=sm_60 -std=c++17 ")
target_link_libraries (CUDA_Publisher_Assignment PRIVATE OpenMP::OpenMP_CXX ${PCL_LIBRARIES} )

add_executable (CUDA_Downsample Downsample.cu)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --forward-unknown-to-host-compiler -arch=sm_60 -std=c++17 ")
target_link_libraries (CUDA_Downsample PRIVATE OpenMP::OpenMP_CXX ${PCL_LIBRARIES} )




