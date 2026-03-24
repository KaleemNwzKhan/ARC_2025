# ARC: Accurate, Real-Time, and Scalable Multi-Vehicle Cooperative Perception

This repository contains the code for the paper [ARC: Accurate, Real-Time, and Scalable Multi-Vehicle Cooperative Perception](https://kaleemnwzkhan.github.io/assets/pdf/ARC.pdf).

# Table of Contents
- [Setup](#setup)
- [Dataset](#dataset)
- [Run Instructions](#run-instructions)
  
## Setup
For this project, we will use a Docker container to containerize our code.

### Pre-requisties
This document assumes that you already have [Docker](https://docs.docker.com/engine/install/ubuntu/) and [NVIDIA-Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed on your machine. 

### Docker Setup
Download the docker container for this project as follows:

```bash
docker pull kk5271/rapid_v2v:1.0
```

To check if you have pulled the Docker, run the following:

```bash
docker images
```

This should output all the Docker images you have on your machine, and it should contain `docker pull kk5271/rapid_v2v:1.0`. After starting Docker, ensure you run the docker file and define your path settings and volume mounts correctly to access datasets and code.

## Dataset
Download the dataset from [this Google Drive link](https://drive.google.com/drive/folders/1Q374JyFSiZ8mbfJZweNIv9sgqCm8VhnQ?usp=sharing) and extract its contents into the `Dataset` folder.

This repository relies on specific directory structures for dataset processing:
- **Vehicles Directory (`/dataset/Vehicles`)**: Contains the source data for each vehicle in the scene.
- **Leader Directory (`/dataset/Leader`)**: Contains the reference data (`PCDs` and `Leader_Poses.txt`) for the vehicle designated as the anchor/leader.

## Run Instructions
This section explains how to run the codebase, reproduce alignment outcomes, and plot the accuracy results. 

1. **Start the Docker Environment**: Ensure you are running inside the `kk5271/rapid_v2v:1.0` Docker.
2. **Vehicle Localization**: Run the localization script to generate poses and latencies.
   ```bash
   cd /workspace/Localization_and_Alignment/catkin_ws/
   bash Vehicle_Localization.sh
   ```
   *Output*: In each vehicle directory under `/dataset/Vehicles`, this script will output `Poses.txt` and `Latencies.txt`.

3. **Leader Definition**: 
   For simplicity, we will designate a single vehicle as the "leader" throughout a 15-second duration. 
   Copy the localization results and point clouds for that vehicle to the leader directory paths:
   - `/dataset/Leader/PCDs`
   - `/dataset/Leader/Leader_Poses.txt`

4. **Fast Overlap Extraction**: Next, execute the Fast Overlap Extraction Algorithm to identify the overlapping regions.

5. **Vehicle to Leader Ground Truth**: Calculate the vehicle to leader (anchor) ground-truth transformations.
   ```bash
   cd /workspace/Localization_and_Alignment
   bash Vehicle_to_Leader_Ground_Truth.sh
   ```

6. **Anchor-based Direct Alignment**: Run the direct alignment process using the anchor.
   ```bash
   bash Anchor_based_Direct_Alignment.sh
   ```

7. **Vehicle to Vehicle Ground Truth Alignment**: Generate the V2V ground truth and intermediate tracking directories.
   ```bash
   bash Vehicle_to_Vehicle_GT.sh
   ```
   *Output*: This command creates three folders inside every vehicle folder:
   - **`V2V_Direct`**: End-to-end vehicle-to-vehicle alignment through indirect alignment (map-based).
   - **`V2V_Leader`**: End-to-end vehicle-to-vehicle alignment through indirect anchor-based alignment.
   - **`V2V_GT`**: Vehicle-to-vehicle ground truth alignment.

8. **Alignment Error Generation**: Calculate alignment errors for both direct mapping and anchor-based mapping.
   ```bash
   bash V2V_E2E_Direct_Error.sh
   bash V2V_E2E_Lead_Error.sh
   ```
   *Output*: The scripts evaluate relative errors and yield four files:
   - `RTE_V2V_Direct.txt` (Relative Translation Error for Direct Mapping)
   - `RRE_V2V_Direct.txt` (Relative Rotation Error for Direct Mapping)
   - `RTE_V2V_Lead.txt` (Relative Translation Error for Leader/Anchor Mapping)
   - `RRE_V2V_Lead.txt` (Relative Rotation Error for Leader/Anchor Mapping)

9. **Accuracy Plots**: To visualize these error metrics, navigate to the results folder and execute the plotting script.
   ```bash
   cd results/
   python3 Plot.py
   ```
   *Output*: This will generate the final accuracy plots characterizing the alignment results.

## Citation
If you are using this code or dataset in your work, please consider citing our paper:

```bibtex
@inproceedings{khan2026arc,
  title={ARC: Accurate, Real-Time, and Scalable Multi-Vehicle Cooperative Perception},
  author={Khan, Kaleem Nawaz and Ahmad, Fawad},
  booktitle={ACM/IEEE International Conference on Embedded Artificial Intelligence and Sensing Systems},
  year={2026}
}
```
