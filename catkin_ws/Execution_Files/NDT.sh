#!/bin/bash
source setup_env.bash

for Vehicle in $(seq 0 39)
do
    echo "Processing Vehicle_Number: ${Vehicle}"
    VEHICLE_DIR="PCDs/Vehicle_${Vehicle}"
    rm -rf ${VEHICLE_DIR}/Poses.txt
    rm -rf ${VEHICLE_DIR}/Latencies.txt

    # Check if the directory exists
    if [ -d "${VEHICLE_DIR}" ]; then
        echo "Directory exists: ${VEHICLE_DIR}. Running rosrun..."

        # Run the rosrun command with the appropriate arguments
        rosrun fast_gicp cuda_ndt_align_R2V \
            ${VEHICLE_DIR}/GT.txt \
            ${VEHICLE_DIR}/Pcds \
            ${VEHICLE_DIR} \
            ${VEHICLE_DIR} || { echo "Error processing Vehicle_Number: ${Vehicle}. Exiting."; exit 1; }

    else
        echo "Directory ${VEHICLE_DIR} does not exist. Skipping..."
    fi
done
