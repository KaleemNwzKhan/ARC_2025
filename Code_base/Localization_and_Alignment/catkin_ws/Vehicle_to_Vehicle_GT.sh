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
    rm -r "/dataset/Vehicles/Vehicle_${Main_Vehicle}/V2V_Direct"
    rm -r "/dataset/Vehicles/Vehicle_${Main_Vehicle}/V2V_Leader"
    rm -r "/dataset/Vehicles/Vehicle_${Main_Vehicle}/V2V_GT"
    mkdir "/dataset/Vehicles/Vehicle_${Main_Vehicle}/V2V_Direct"
    mkdir "/dataset/Vehicles/Vehicle_${Main_Vehicle}/V2V_Leader"
    mkdir "/dataset/Vehicles/Vehicle_${Main_Vehicle}/V2V_GT"
    for Vehicle in $(seq 0 39)
    do
        if [ "$Vehicle" = "$Main_Vehicle" ]; then
                    continue
                fi
        echo "Main Vehicle: ${Main_Vehicle}"
        echo "Vehicle_Number: ${Vehicle}"
        Vehicle_Map_Pose="/dataset/Vehicles/Vehicle_${Vehicle}/Poses.txt"
        Main_Vehicle_Map_Pose="/dataset/Vehicles/Vehicle_${Main_Vehicle}/Poses.txt"
        Vehicle_Leader_Pose="/dataset/Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Direct_Alignment_Poses.txt"
        Main_Vehicle_Leader_Pose="/dataset/Leader_Vehicle_Overlap/Vehicle_${Main_Vehicle}/Direct_Alignment_Poses.txt"
        Vehicle_GT_Pose="/dataset/Vehicles/Vehicle_${Vehicle}/GT.txt"
        Main_Vehicle_GT_Pose="/dataset/Vehicles/Vehicle_${Main_Vehicle}/GT.txt"

        python3 Calculate_all_possible_transforms.py "${Vehicle_Map_Pose}" "${Main_Vehicle_Map_Pose}" \
            "${Vehicle_Leader_Pose}" "${Main_Vehicle_Leader_Pose}" "${Vehicle_GT_Pose}" "${Main_Vehicle_GT_Pose}" \
            "/dataset/Vehicles/Vehicle_${Main_Vehicle}/V2V_Direct/${Vehicle}_Direct.txt" "/dataset/Vehicles/Vehicle_${Main_Vehicle}/V2V_Leader/${Vehicle}_Lead.txt" "/dataset/Vehicles/Vehicle_${Main_Vehicle}/V2V_GT/${Vehicle}_GT.txt" 

    done
done
