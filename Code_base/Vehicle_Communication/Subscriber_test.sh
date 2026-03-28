#!/bin/bash
cmake .
make

for Vehicle in $(seq 0 39)
do
    echo "Vehicle_Number: ${Vehicle}"
    
    BlindSpots_DIR="/dataset/Communication/BlindSpots"
    Vehicles_Cell_DIR="/dataset/Communication/Vehicles_Cells"
    
    rm -rf "${BlindSpots_DIR}/Vehicle_${Vehicle}"
    mkdir -p "${BlindSpots_DIR}/Vehicle_${Vehicle}"
    
    rm -rf "${Vehicles_Cell_DIR}/Vehicle_${Vehicle}"
    mkdir -p "${Vehicles_Cell_DIR}/Vehicle_${Vehicle}"

    ./CUDA_Subscriber /dataset \
        Map/Intersection_32_Beam.pcd \
        Vehicles/Vehicle_${Vehicle}/Pcds \
        Vehicles/Vehicle_${Vehicle}/Poses \
        Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Direct_Alignment_Poses \
        Communication/All_Vehicles_Leader_Transforms_Folder \
        Communication/Subscriber_BlindSpot_Latencies/${Vehicle} \
        Communication/BlindSpots/Vehicle_${Vehicle} \
        ${Vehicle} \
        Communication/BlindSpots_Size/${Vehicle}
done

