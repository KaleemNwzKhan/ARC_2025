cmake .
make
 for Vehicle in $(seq 0 0)
 do
     echo "Vehicle_Number: ${Vehicle}"

     ./CUDA_Publisher_Assignment Dataset \
 	 Intersection_32_Beam.pcd \
         Vehicle_PCDs/Vehicle_${Vehicle}/Pcds \
         Vehicle_PCDs/Vehicle_${Vehicle}/Poses \
         Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Direct_Alignment_Poses \
         All_Vehicles_Leader_Transforms_Folder \
         Subscriber_BlindSpot_Latencies/${Vehicle} \
         BlindSpots/${Vehicle} \
         ${Vehicle} \
 	 BlindSpots_Size/${Vehicle}
 done
# #for Vehicle in $(seq 0 39)
# #do
# #    echo "Vehicle_Number: ${Vehicle}"
# #
# #   ./CUDA_Publisher Dataset \
# #		   Intersection_32_Beam.pcd \
# #        Vehicle_PCDs/Vehicle_${Vehicle}/Pcds \
# #        Vehicle_PCDs/Vehicle_${Vehicle}/Poses \
# #        Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Direct_Alignment_Poses \
# #        All_Vehicles_Leader_Transforms_Folder \
# #        BlindSpot_Latencies/${Vehicle} \
# #        BlindSpot_Each_Frame \
# #        ${Vehicle} \
# #	BlindSpots_Size/${Vehicle}
# #done



# Subscriber_Publisher_Assignment
