import numpy as np
import copy
import sys
from scipy.spatial.transform import Rotation as R
import time

Vehicle_Map_Pose = sys.argv[1]
Main_Vehicle_Map_Pose = sys.argv[2]
Vehicle_Leader_Pose = sys.argv[3]
Main_Vehicle_Leader_Pose = sys.argv[4]
Vehicle_GT_Pose = sys.argv[5]
Main_Vehicle_GT_Pose= sys.argv[6]
E2E_Vehicle= sys.argv[7]
E2E_Lead= sys.argv[8]
E2E_GT= sys.argv[9]

I_G = open(Vehicle_Map_Pose, 'r').read().splitlines()
I_G = list(map(str.split, filter(lambda x: x != '4 4', I_G)))
I_G = [np.array(I_G[i:i+4]) for i in range(0, len(I_G), 4)]

V_G=open(Main_Vehicle_Map_Pose, 'r').read().splitlines()
V_G= list(map(str.split, filter(lambda x: x != '4 4', V_G)))
V_G = [np.array(V_G[i:i+4]) for i in range(0, len(V_G), 4)]

file = open(E2E_Vehicle, "a")  
for k in range(len(V_G)):
    Pose_I_G = I_G[k].astype(np.float)
    Pose_V_G=V_G[k].astype(np.float)
    Pose_V_G=np.linalg.inv(Pose_V_G)
    G_Alignment=np.dot(Pose_V_G, Pose_I_G)
    np.savetxt(file,G_Alignment)
file.close()


I_G = open(Vehicle_Leader_Pose, 'r').read().splitlines()
I_G = list(map(str.split, filter(lambda x: x != '4 4', I_G)))
I_G = [np.array(I_G[i:i+4]) for i in range(0, len(I_G), 4)]

V_G=open(Main_Vehicle_Leader_Pose, 'r').read().splitlines()
V_G= list(map(str.split, filter(lambda x: x != '4 4', V_G)))
V_G = [np.array(V_G[i:i+4]) for i in range(0, len(V_G), 4)]

file = open(E2E_Lead, "a")  
for k in range(len(V_G)):
    Pose_I_G = I_G[k].astype(np.float)
    Pose_V_G=V_G[k].astype(np.float)
    Pose_V_G=np.linalg.inv(Pose_V_G)
    G_Alignment=np.dot(Pose_V_G, Pose_I_G)
    np.savetxt(file,G_Alignment)
file.close()

I_G = open(Vehicle_GT_Pose, 'r').read().splitlines()
I_G = list(map(str.split, filter(lambda x: x != '4 4', I_G)))
I_G = [np.array(I_G[i:i+4]) for i in range(0, len(I_G), 4)]

V_G=open(Main_Vehicle_GT_Pose, 'r').read().splitlines()
V_G= list(map(str.split, filter(lambda x: x != '4 4', V_G)))
V_G = [np.array(V_G[i:i+4]) for i in range(0, len(V_G), 4)]

file = open(E2E_GT, "a")  
for k in range(len(V_G)):
    Pose_I_G = I_G[k].astype(np.float)
    Pose_V_G=V_G[k].astype(np.float)
    Pose_V_G=np.linalg.inv(Pose_V_G)
    G_Alignment=np.dot(Pose_V_G, Pose_I_G)
    np.savetxt(file,G_Alignment)
file.close()
