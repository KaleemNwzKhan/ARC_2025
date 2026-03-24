#!/bin/bash
cmake .
make
mkdir /dataset/Leader_Vehicle_Overlap
mkdir /dataset/Leader_Vehicle_Overlap/Overlap_Latencies
 for Vehicle in $(seq 0 39)
 do
       mkdir /dataset/Leader_Vehicle_Overlap/Vehicle_${Vehicle}
	   mkdir /dataset/Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Vehicle
	   mkdir /dataset/Leader_Vehicle_Overlap/Vehicle_${Vehicle}/Leader
        ./Finding_Overlap /dataset \
 		Map/Intersection_32_Beam.pcd \
        Leader/PCDs \
        Vehicles/Vehicle_${Vehicle}/Pcds \
		Leader/Leader_Poses\
        Vehicles/Vehicle_${Vehicle}/Poses\
        Leader_Vehicle_Overlap/Vehicle_${Vehicle}/ \
		Leader_Vehicle_Overlap/Overlap_Latencies/Overlap_${Vehicle} 
 done



# cmake .
# make

# for i in {1..1}
# do
# 	./Only_Non_Ground Dataset Intersection_32_Beam.pcd Leader_PCDs Leader_Map_Transform/Leader
# 	# ./Finding_Overlap Dataset Downsampled_0.3_Complete_Intersection_Map.pcd Decompressed Vehicle Leader Vehicle Overlap_Latency
# done
