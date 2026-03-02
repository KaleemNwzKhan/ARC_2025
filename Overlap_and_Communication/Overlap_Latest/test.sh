#!/bin/bash
cmake .
make
 for Vehicle in $(seq 0 39)
 do
       mkdir Dataset/Leader_Vehicle_Overlap/Vehicle_${Vehicle}
	   mkdir Dataset/Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Vehicle
	   mkdir Dataset/Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Leader
        ./Finding_Overlap Dataset \
 		Intersection_32_Beam.pcd \
        Leader_PCDs \
        Vehicle_PCDs/Vehicle_${Vehicle}/Pcds \
		Leader_Map_Transform/Leader \
        Vehicle_PCDs/Vehicle_${Vehicle}/Poses\
        Leader_Vehicle_Overlap/Vehicle_${Vehicle}/ \
		Overlap_Latencies/Overlap_${Vehicle} 
 done



# cmake .
# make

# for i in {1..1}
# do
# 	./Only_Non_Ground Dataset Intersection_32_Beam.pcd Leader_PCDs Leader_Map_Transform/Leader
# 	# ./Finding_Overlap Dataset Downsampled_0.3_Complete_Intersection_Map.pcd Decompressed Vehicle Leader Vehicle Overlap_Latency
# done
