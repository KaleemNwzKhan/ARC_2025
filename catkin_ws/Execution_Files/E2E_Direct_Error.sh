#!/bin/bash

# Define the set of vehicles to skip
#SKIP_VEHICLES=(4 5 6 12 13 14 15 22 23 24)
#SKIP_VEHICLES=(0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38)
SKIP_VEHICLES=(100)

for Vehicle in $(seq 0 39); do
    # Check if the current vehicle is in the skip list
    if [[ " ${SKIP_VEHICLES[@]} " =~ " ${Vehicle} " ]]; then
        echo "Skipping Vehicle ${Vehicle}..."
        continue
    fi

    echo "Vehicle_Number: ${Vehicle}"
    VEHICLE_DIR="Leader_Vehicle_Overlap/Vehicle_${Vehicle}"

    if [ -d "${VEHICLE_DIR}" ]; then
        echo "Processing Vehicle ${Vehicle}..."

        # Define the paths for the input files
        GT_FILE="${VEHICLE_DIR}/Lead_Vehicle_GT.txt"
        POSES_FILE="${VEHICLE_DIR}/Direct_Alignment_Poses.txt"
        COUNT_FILE="${VEHICLE_DIR}/Cells_count.txt"

        if [ -f "${GT_FILE}" ] && [ -f "${POSES_FILE}" ]; then
            # Run the Python script with the input files
            python3 NDT_DT_Calculate_Transforms_Error.py "${GT_FILE}" "${POSES_FILE}" "${COUNT_FILE}"\
                "RTE_Direct.txt" "RRE_Direct.txt"
            echo "Processed Vehicle ${Vehicle}, output files saved in ${VEHICLE_DIR}"
        else
            echo "One or both files missing in ${VEHICLE_DIR}. Skipping..."
        fi
    else
        echo "Directory ${VEHICLE_DIR} does not exist. Skipping..."
    fi
done

