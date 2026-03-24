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
    rm -r "PCDs/Vehicle_${Main_Vehicle}/V2V_Direct"
    rm -r "PCDs/Vehicle_${Main_Vehicle}/V2V_Leader"
    rm -r "PCDs/Vehicle_${Main_Vehicle}/V2V_GT"
    mkdir "PCDs/Vehicle_${Main_Vehicle}/V2V_Direct"
    mkdir "PCDs/Vehicle_${Main_Vehicle}/V2V_Leader"
    mkdir "PCDs/Vehicle_${Main_Vehicle}/V2V_GT"
    for Vehicle in $(seq 0 39)
    do
        if [ "$Vehicle" = "$Main_Vehicle" ]; then
                    continue
                fi
        echo "Main Vehicle: ${Main_Vehicle}"
        echo "Vehicle_Number: ${Vehicle}"
        Vehicle_Map_Pose="PCDs/Vehicle_${Vehicle}/Poses.txt"
        Main_Vehicle_Map_Pose="PCDs/Vehicle_${Main_Vehicle}/Poses.txt"
        Vehicle_Leader_Pose="Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Direct_Alignment_Poses.txt"
        Main_Vehicle_Leader_Pose="Leader_Vehicle_Overlap/Vehicle_${Main_Vehicle}/Direct_Alignment_Poses.txt"
        Vehicle_GT_Pose="PCDs/Vehicle_${Vehicle}/GT.txt"
        Main_Vehicle_GT_Pose="PCDs/Vehicle_${Main_Vehicle}/GT.txt"

        python3 V2V_Vehicle_and_Vehicle_calculate_V2V_transforms.py "${Vehicle_Map_Pose}" "${Main_Vehicle_Map_Pose}" \
            "${Vehicle_Leader_Pose}" "${Main_Vehicle_Leader_Pose}" "${Vehicle_GT_Pose}" "${Main_Vehicle_GT_Pose}" \
            "PCDs/Vehicle_${Main_Vehicle}/V2V_Direct/${Vehicle}_Direct.txt" "PCDs/Vehicle_${Main_Vehicle}/V2V_Leader/${Vehicle}_Lead.txt" "PCDs/Vehicle_${Main_Vehicle}/V2V_GT/${Vehicle}_GT.txt" 

    done
done
