import numpy as np
import copy
import sys
from scipy.spatial.transform import Rotation as R
import time

GT_FILE = sys.argv[1]
POSES_FILE = sys.argv[2]
LEADER_POSES_FILE = sys.argv[3]
LEADER_GT_FILE = sys.argv[4]
Lead_Vehicle_GT = sys.argv[5]
Lead_Vehicle_NDT= sys.argv[6]

I_G = open(GT_FILE, 'r').read().splitlines()
I_G = list(map(str.split, filter(lambda x: x != '4 4', I_G)))
I_G = [np.array(I_G[i:i+4]) for i in range(0, len(I_G), 4)]

V_G=open(LEADER_GT_FILE, 'r').read().splitlines()
V_G= list(map(str.split, filter(lambda x: x != '4 4', V_G)))
V_G = [np.array(V_G[i:i+4]) for i in range(0, len(V_G), 4)]

file = open(Lead_Vehicle_GT, "a")  
for k in range(len(V_G)):
    Pose_I_G = I_G[k].astype(np.float)
    Pose_V_G=V_G[k].astype(np.float)
    Pose_V_G=np.linalg.inv(Pose_V_G)
    G_Alignment=np.dot(Pose_V_G, Pose_I_G)
    np.savetxt(file,G_Alignment)
file.close()


I_G = open(POSES_FILE, 'r').read().splitlines()
I_G = list(map(str.split, filter(lambda x: x != '4 4', I_G)))
I_G = [np.array(I_G[i:i+4]) for i in range(0, len(I_G), 4)]

V_G=open(LEADER_POSES_FILE, 'r').read().splitlines()
V_G= list(map(str.split, filter(lambda x: x != '4 4', V_G)))
V_G = [np.array(V_G[i:i+4]) for i in range(0, len(V_G), 4)]

file = open(Lead_Vehicle_NDT, "a")  
for k in range(len(V_G)):
    Pose_I_G = I_G[k].astype(np.float)
    Pose_V_G=V_G[k].astype(np.float)
    Pose_V_G=np.linalg.inv(Pose_V_G)
    G_Alignment=np.dot(Pose_V_G, Pose_I_G)
    np.savetxt(file,G_Alignment)
file.close()


