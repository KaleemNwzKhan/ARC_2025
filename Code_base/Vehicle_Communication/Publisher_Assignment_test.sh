cmake .
make
 for Vehicle in $(seq 0 0)
 do
     echo "Vehicle_Number: ${Vehicle}"

     ./CUDA_Publisher_Assignment /dataset \
 	 Map/Intersection_32_Beam.pcd \
         Vehicles/Vehicle_${Vehicle}/Pcds \
         Vehicles/Vehicle_${Vehicle}/Poses \
         Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Direct_Alignment_Poses \
         Communication/All_Vehicles_Leader_Transforms_Folder \
         Assignment_temp/${Vehicle} \
         Assignment_temp/${Vehicle} \
         ${Vehicle} \
 	 Assignment_temp/${Vehicle}
 done
