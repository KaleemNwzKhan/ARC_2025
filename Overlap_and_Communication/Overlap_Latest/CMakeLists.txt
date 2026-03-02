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

add_executable (Finding_Overlap Finding_Overlap.cu)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --forward-unknown-to-host-compiler -arch=sm_60 -std=c++17 ")
target_link_libraries (Finding_Overlap PRIVATE OpenMP::OpenMP_CXX ${PCL_LIBRARIES} )

add_executable (Only_Non_Ground Only_Non_Ground.cu)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --forward-unknown-to-host-compiler -arch=sm_60 -std=c++17 ")
target_link_libraries (Only_Non_Ground PRIVATE OpenMP::OpenMP_CXX ${PCL_LIBRARIES} )




