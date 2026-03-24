#!/bin/bash

# Source the base environment
source /home/kk5271/anaconda3/bin/activate base

# Debugging info
echo "Which Python: $(which python3)"
echo "Python version:"
python3 --version
echo "Checking if open3d is installed:"
python3 -c "import open3d"
for Main_Vehicle in $(seq 0 39)
do
    for Vehicle in $(seq 0 39)
    do
        VEHICLE_DIR="/dataset/Vehicles/Vehicle_${Main_Vehicle}"

        if [ -d "${VEHICLE_DIR}" ]; then
            echo "Processing Vehicle ${Vehicle}..."

          if [ "$Vehicle" = "$Main_Vehicle" ]; then
                    continue
                fi

            # Define the paths for the input files
            GT_FILE="${VEHICLE_DIR}/V2V_GT/${Vehicle}_GT.txt"
            POSES_FILE="${VEHICLE_DIR}/V2V_Direct/${Vehicle}_Direct.txt"

            if [ -f "${GT_FILE}" ] && [ -f "${POSES_FILE}" ]; then
                # Run the Python script with the input files
                python3 Calculate_Transforms_Error.py "${GT_FILE}" "${POSES_FILE}"\
                    "/workspace/Results/Alignment_Results/RTE_V2V_Direct.txt" "/workspace/Results/Alignment_Results/RRE_V2V_Direct.txt"
                echo "Processed Vehicle ${Vehicle}, output files saved in /workspace/Results/Alignment_Results/"
            else
                echo "One or both files missing in ${VEHICLE_DIR}. Skipping..."
            fi
        else
            echo "Directory ${VEHICLE_DIR} does not exist. Skipping..."
        fi
    done
done

