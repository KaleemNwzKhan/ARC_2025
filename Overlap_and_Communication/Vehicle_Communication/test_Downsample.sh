cmake .
make
./CUDA_Publisher Vehicles_Points_Only_1.pcd
# ./CUDA_Downsample
#  for Vehicle in $(seq 0 44)
#  do
#      echo "Vehicle_Number: ${Vehicle}"

#      ./CUDA_Subscriber Dataset \
#  		Downsampled_0.3_Complete_Intersection_Map.pcd \
#          Vehicle_PCDs/V_${Vehicle} \
#          Vehicle_Map_Transform/V_${Vehicle}_NDT \
#          Vehicle_Leader_Transform/${Vehicle} \
#          All_Vehicles_Leader_Transforms_Folder \
#          BlinSpot_Latencies/${Vehicle} \
#          BlindSpots/${Vehicle} \
#          ${Vehicle} \
#  	BlindSpots_Size/${Vehicle}
#  done
# #for Vehicle in $(seq 0 44)
# #do
# #    echo "Vehicle_Number: ${Vehicle}"
# #
# #   ./CUDA_Publisher Dataset \
# #		Downsampled_0.3_Complete_Intersection_Map.pcd \
# #        Vehicle_PCDs/V_${Vehicle} \
# #        Vehicle_Map_Transform/V_${Vehicle}_NDT \
# #        Vehicle_Leader_Transform/${Vehicle} \
# #        All_Vehicles_Leader_Transforms_Folder \
# #        BlinSpot_Latencies/${Vehicle} \
# #       BlindSpot_Each_Frame \
# #        ${Vehicle} \
# #	BlindSpots_Size/${Vehicle}
# #done
