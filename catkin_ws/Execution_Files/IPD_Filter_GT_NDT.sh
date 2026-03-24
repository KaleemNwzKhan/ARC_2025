#!/bin/bash

# Source the base environment
source /home/kk5271/anaconda3/bin/activate base

# Debugging info
echo "Which Python: $(which python3)"
echo "Python version:"
python3 --version
echo "Checking if open3d is installed:"
python3 -c "import open3d"

for Vehicle in $(seq 0 39)
do
    echo "Vehicle_Number: ${Vehicle}"
    VEHICLE_DIR="PCDs/V_${Vehicle}"
    VEHICLE_DIR_Overlap="Leader_Vehicle_Overlap/V_${Vehicle}"
    Cells_Count=${VEHICLE_DIR_Overlap}/Cells_count.txt

    if [ -d "${VEHICLE_DIR}" ]; then
        echo "Processing Vehicle ${Vehicle}..."

        # Define the paths for the input files
        GT_FILE="${VEHICLE_DIR}/Lead_Vehicle_GT.txt"
        POSES_FILE="${VEHICLE_DIR}/Lead_Vehicle_NDT.txt"

        if [ -f "${GT_FILE}" ] && [ -f "${POSES_FILE}" ]; then
            # Run the Python script with the input files

            python IPD_Filter.py "${GT_FILE}" "${POSES_FILE}" \
                "${Cells_Count}" \
                "${VEHICLE_DIR_Overlap}/Lead_Vehicle_GT.txt" "${VEHICLE_DIR_Overlap}/Lead_Vehicle_NDT.txt" 
            # echo "Processed Vehicle ${Vehicle}, output files saved in ${VEHICLE_DIR}"
        else
            echo "One or both files missing in ${VEHICLE_DIR}. Skipping..."
        fi
    else
        echo "Directory ${VEHICLE_DIR} does not exist. Skipping..."
    fi
done
