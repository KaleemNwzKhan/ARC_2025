1. Put all the vehicle folder in PCDs directory
2. Run NDT.sh this will store Poses.txt and Latencies.txt in each vehicle folder
3. If you want to calcuate the Localization error you can run NDT_Error.sh
4. Now go to Overlap_Latest folder in /home/kk5271/RAPID_V2V/Intersection_DrivablePoints_and_Grid/ and run test.sh
5. Copy Leader_Vehicle_Overlap folder from the same directory to /home/kk5271/workspace_eaad/catkin_ws/
6. Then run V_L_Ground_Truth.sh
7. Then run Direct_ICP.sh
8. Then run V2V_GT.sh
9. Then run V2V_E2E_Direct_Error.sh
10. And lastly V2V_E2E_Lead_Error.sh
11. At the end, we will have four files "RTE_V2V_Direct.txt", "RRE_V2V_Direct.txt", "RTE_V2V_Lead.txt" and "RRE_V2V_Lead.txt"
12. Go to the Plot and and paste these four files there to get the plots for accuracy.
13. From here two kind of latencies can be captured. From the latencies paste the following to files here 
Leader_Alignment_Latencies.py and NDT_Latencies.py to get the following two latencies files
Leader_Alignment.txt and Localization.txt
14. Now go to Overlap_Latest/Dataset folder in /home/kk5271/RAPID_V2V/Intersection_DrivablePoints_and_Grid/ and paste the following file from the Plots Latencies
Overlap_Latencies.py and run this file you will get Overlap_Estimation.txt 
15. Now go to Consumer_Latest/Dataset folder in /home/kk5271/RAPID_V2V/Intersection_DrivablePoints_and_Grid/ and paste the following file from the Plots Latencies
Consumer_Latencies.py and run this file you will get BlindSpots_Estimation_and_Producer_Assignment.txt
16. Now go to Producer_Latest/Dataset folder in /home/kk5271/RAPID_V2V/Intersection_DrivablePoints_and_Grid/ and paste the following file from the Plots Latencies
Producer_Latencies.py and run this file you will get Providing_BlindSpots_Vehicles.txt
17. Now copy all these latencies files to the Plots/Latencies and run E2E_Latency.py to get the Latency_Components_and_E2E_Latency.pdf
18. From Producer_Latest/Dataset and Consumer_Latest/Dataset copy BlindSpots_Sizes into Plots/Network and rename them as Producer and Consumer. 
19. Also copy Leader_PCDs folder and Meta_Data_Sharing.txt file to the Plots/Network and first run Calculate_Leader_Size.py then Plot_Sizes.py and lastly Per_Vehicle_Plot_Sizes.py to 
get all the network related plots.


