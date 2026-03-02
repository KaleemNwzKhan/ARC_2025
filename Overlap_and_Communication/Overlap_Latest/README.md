# GPU-based implementation for assessing inter-point density within 3D point clouds

- [Motivation](#item-one)
- [Approach](#item-two)
- [Challenges](#item-three)
- [Contributions](#item-four)
- [How to calculate density of the overlapping region using IPD](#item-five)
- [How to run the code](#item-six)

 <a id="item-one"></a>
## Motivation
Autonomus vehicle (AV) uses 3D sensors such as LiDAR, RADAR and cameras.These sensors are prone to occlusion and suffers from line-of-sight limitations. One such example is given below, where a red bus occludes the red-ligt violating yellow cab from the gray AV equipped with LiDAR. 

<div style="display: flex; justify-content: center;">
    <img src="images/Red_light_violating_scenario.png" alt="Red Light Violating Scenario" style="width: 45%;">
    <img src="images/LiDAR_View.png" alt="LiDAR View" style="width: 45%;">
</div>

This can cause accidents. A system is requried that assists the AV to fill the occluded area in its sensor's point clouds.

 <a id="item-two"></a>
## Approach

To assist AVs and augment their percpetion we have proposes Vehicle Road-side Point Cloud Fusion (VRF). VRF leverages road-side unit (RSU) LiDARs to augment AVs perception in real-time with cm level accuracy. The RSU send it LiDAR's point cloud to the AV which the AV fused with its own LiDAR's point cloud to fill the blindspots and augment its perception of the environment. The following figure show an AV (blue) and RSU (red) fused point cloud. The yellow vehicle which was occlued previously in the AV's point cloud now clearly visible in the fused point cloud. Fusion consists of two steps; (1) Aligning both the point clouds (2) Stitching both the point clouds

<div style="display: flex; justify-content: center;">
    <img src="images/Fused_Point_Cloud.png" alt="Fused Point Cloud";>
</div>


 <a id="item-three"></a>
## Challenges

- For accurate registration, algorithms such as iterative closest point (ICP) require an initial coarse alignment and significant overlap between the point clouds otherwise they don't converge
- The registration should be very fast (less than 100ms)
- We need to find out whether if their is significant overlap and we can do registration.
- We need to make this decision in real-time

 <a id="item-four"></a>
## Contributions

- We align the both the vehicle and the RUS point clouds to a common reference, i.e. 3D map of the environment and this alignment is the initial coarse alignment. 
- After that we have developed an inter-point density (IPD) based algorithm that takes both the point clouds and calculates the density of the overlapping region between the two point clouds using IPD.
- Our CPU based implementation of this algorithm incurs latency less than 178ms which is fast but not real-time for AVs
- We have opitmized our implementation and developed GPU-based solution that reduced the latency from 178ms to 58ms.

 <a id="item-five"></a>
## How to calculate density of the overlapping region using IPD

- We create a grid with a specific cell size, the dimensions of the grid depends on the extreme values of x, y, z in one of the two point clouds 
- Find the center of each grid cell 
- Run a 1-nearest neighbor (1NN) search to assign RSU points to their nearst grids' centers
- Similarly run a 1-nearest neighbor (1NN) search to assign Vehicle points to their nearst grids' centers
- Now, calculate IPD for each cell, i.e., min (RSU_points, Vehicle_points)
- Set a threshold, if a cell has IPD greater than the threshold we will classify that cell as dense cell 
- Count the number of cells having IPD above the threshold, and if the value is 10% of the total number of cells then the overlap is dense and we can perform the ICP for the alignment otherwise not.
- The output of the algorithm is a csv file, that contains Frame #, Latency (ms), and count of cells having IPD greater than the threshold.

 <a id="item-six"></a>
## How to run the code

### Setup the environment
- First run, *pip install -r requirements.txt*
- Install point cloud library (PCL), *conda install conda-forge::pcl*
### Implementation
- InterPoint_Density.cpp file contains C++ CPU-based implementation (**Baseline**) to calculate IPD and find the overlapping region (number of cells whoes IPDs are greater than the given threshold)
- OMP_InterPoint_Density.cpp file contains C++ CPU OMP-based implementation (**OMP based Solution**) to calculate IPD and find the overlapping region (number of cells whoes IPDs are greater than the given threshold)
- CUDA_InterPoint_Density.cu file contains C++ GPU-based implementation (**CUDA based Solution**) to calculate IPD and find the overlapping region (number of cells whoes IPDs are greater than the given threshold)
### Tests
- Tests folder contains five different tests data (both RSU and Vehicle point clouds)
- test.sh file contains the end-to-end execution script that runs (**Baseline**), (**OMP based Solution**) and (**CUDA based Solution**) implementations for all the tests 
- Run *./test.sh* 
### Comparisons
- Latency_Comparisons.py file is python based implementation that takes results from (**Baseline**), (**OMP based Solution**) and (**CUDA based Solution**) implementations and compare the execution time
- Run *python3 Latency_Comparisons.py* and you will get a comparison table like the one below

<div style="display: flex; justify-content: center;">
    <img src="images/Comparisons.png" alt="Comparions";>
</div>
