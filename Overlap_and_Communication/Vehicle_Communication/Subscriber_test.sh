#!/bin/bash
cmake .
make

for Vehicle in $(seq 0 39)
do
    echo "Vehicle_Number: ${Vehicle}"
    
    BlindSpots_DIR="BlindSpots"
    Vehicles_Cell_DIR="Vehicles_Cells"
    
    rm -rf "${BlindSpots_DIR}/Vehicle_${Vehicle}"
    mkdir -p "${BlindSpots_DIR}/Vehicle_${Vehicle}"
    
    rm -rf "${Vehicles_Cell_DIR}/Vehicle_${Vehicle}"
    mkdir -p "${Vehicles_Cell_DIR}/Vehicle_${Vehicle}"

    ./CUDA_Subscriber Dataset \
        Intersection_32_Beam.pcd \
        Vehicle_PCDs/Vehicle_${Vehicle}/Pcds \
        Vehicle_PCDs/Vehicle_${Vehicle}/Poses \
        Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Direct_Alignment_Poses \
        All_Vehicles_Leader_Transforms_Folder \
        Subscriber_BlindSpot_Latencies/${Vehicle} \
        BlindSpots/Vehicle_${Vehicle} \
        ${Vehicle} \
        BlindSpots_Size/${Vehicle}
done

