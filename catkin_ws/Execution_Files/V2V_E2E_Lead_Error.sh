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
        VEHICLE_DIR="PCDs/Vehicle_${Main_Vehicle}"

        if [ -d "${VEHICLE_DIR}" ]; then
            echo "Processing Vehicle ${Vehicle}..."

          if [ "$Vehicle" = "$Main_Vehicle" ]; then
                    continue
                fi

            # Define the paths for the input files
            GT_FILE="${VEHICLE_DIR}/V2V_GT/${Vehicle}_GT.txt"
            POSES_FILE="${VEHICLE_DIR}/V2V_Leader/${Vehicle}_Lead.txt"

            if [ -f "${GT_FILE}" ] && [ -f "${POSES_FILE}" ]; then
                # Run the Python script with the input files
                python3 Calculate_Transforms_Error.py "${GT_FILE}" "${POSES_FILE}"\
                    "RTE_V2V_Lead.txt" "RRE_V2V_Lead.txt"
                echo "Processed Vehicle ${Vehicle}, output files saved in ${VEHICLE_DIR}"
            else
                echo "One or both files missing in ${VEHICLE_DIR}. Skipping..."
            fi
        else
            echo "Directory ${VEHICLE_DIR} does not exist. Skipping..."
        fi
    done
done

