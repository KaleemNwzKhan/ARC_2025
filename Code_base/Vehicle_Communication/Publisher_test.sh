cmake .
make
 for Vehicle in $(seq 0 39)
 do
     echo "Vehicle_Number: ${Vehicle}"

     ./CUDA_Publisher /dataset \
 	 Map/Intersection_32_Beam.pcd \
         Vehicles/Vehicle_${Vehicle}/Pcds \
         Vehicles/Vehicle_${Vehicle}/Poses \
         Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Direct_Alignment_Poses \
         Communication/All_Vehicles_Leader_Transforms_Folder \
         Communication/Producer_BlindSpot_Latencies/${Vehicle} \
         Communication/BlindSpots/${Vehicle} \
         ${Vehicle} \
 	 Communication/BlindSpots_Size/${Vehicle}
 done
