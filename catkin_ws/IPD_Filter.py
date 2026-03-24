import numpy as np
import copy
import sys
from scipy.spatial.transform import Rotation as R
import time

Lead_Vehicle_GT= sys.argv[1]
Lead_Vehicle_Pose= sys.argv[2]
IPD_Count = open(sys.argv[3], 'r')
IPD_GT    = open(sys.argv[4], "a")  
IPD_Pose  = open(sys.argv[5], "a")  

I_G = open(Lead_Vehicle_Pose, 'r').read().splitlines()
I_G = list(map(str.split, filter(lambda x: x != '4 4', I_G)))
I_G = [np.array(I_G[i:i+4]) for i in range(0, len(I_G), 4)]

V_G=open(Lead_Vehicle_GT, 'r').read().splitlines()
V_G= list(map(str.split, filter(lambda x: x != '4 4', V_G)))
V_G = [np.array(V_G[i:i+4]) for i in range(0, len(V_G), 4)]

for k in range(len(V_G)):
    line = IPD_Count.readline()
    value = line.strip()
    value = int(line.strip()) 
    if value >=20:
        np.savetxt(IPD_Pose,I_G[k].astype(np.float))
        np.savetxt(IPD_GT,V_G[k].astype(np.float))
