#!/bin/bash

for Vehicle in $(seq 0 39)
do
    echo "Processing Vehicle_Number: ${Vehicle}"
    VEHICLE_DIR="Leader_Vehicle_Overlap/Vehicle_${Vehicle}"
    rm -rf ${VEHICLE_DIR}/ASP.txt

    # Check if the directory exists
    if [ -d "${VEHICLE_DIR}" ]; then
        echo "Directory exists: ${VEHICLE_DIR}. Running rosrun..."

        # Run the rosrun command with the appropriate arguments
        python3 ASP.py \
            ${VEHICLE_DIR}/Cells_count.txt \
            ${VEHICLE_DIR}/Direct_Alignment_Poses.txt \
            ${VEHICLE_DIR}/Lead_Vehicle_NDT.txt \
            ${VEHICLE_DIR}/ASP.txt || { echo "Error processing Vehicle_Number: ${Vehicle}. Exiting."; exit 1; }

    else
        echo "Directory ${VEHICLE_DIR} does not exist. Skipping..."
    fi
done
