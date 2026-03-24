#!/bin/bash
source setup_env.bash

for Vehicle in $(seq 0 39)
do
    echo "Processing Vehicle_Number: ${Vehicle}"
    VEHICLE_DIR="/dataset/Leader_Vehicle_Overlap/Vehicle_${Vehicle}"
    rm -rf ${VEHICLE_DIR}/Direct_Alignment_Poses.txt
    rm -rf ${VEHICLE_DIR}/Latencies.txt

    # Check if the directory exists
    if [ -d "${VEHICLE_DIR}" ]; then
        echo "Directory exists: ${VEHICLE_DIR}. Running rosrun..."

        # Run the rosrun command with the appropriate arguments
        rosrun fast_gicp Vehicle_Vehicle \
            ${VEHICLE_DIR} \
            ${VEHICLE_DIR}/Lead_Vehicle_NDT.txt

    else
        echo "Directory ${VEHICLE_DIR} does not exist. Skipping..."
    fi
done
