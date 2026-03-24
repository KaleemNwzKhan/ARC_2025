import numpy as np
import sys
from scipy.spatial.transform import Rotation as R

a = sys.argv[1]  # Ground truth pose file
b = sys.argv[2]  # Estimated pose file
f = open(sys.argv[3], "a")  # Output file for RTE
g = open(sys.argv[4], "a")  # Output file for RRE

# Read pose data
Pose = open(b, 'r').read().splitlines()
Pose = list(map(str.split, filter(lambda x: x != '4 4', Pose)))
Pose = [np.array(Pose[i:i+4]) for i in range(0, len(Pose), 4)]

G_Pose = open(a, 'r').read().splitlines()
G_Pose = list(map(str.split, filter(lambda x: x != '4 4', G_Pose)))
G_Pose = [np.array(G_Pose[i:i+4]) for i in range(0, len(G_Pose), 4)]



RTE = 0
RRE = 0
valid_count = 0

for k in range(len(G_Pose)):
    
    Pose_i = Pose[k].astype(np.float64)
    G_Pose_i = G_Pose[k].astype(np.float64)

    Pose_Trans = Pose_i[:-1, 3]
    Pose_Rotat = Pose_i[:-1, :-1]
    G_Trans = G_Pose_i[:-1, 3]
    G_Rotat = G_Pose_i[:-1, :-1]

    RTE_temp = np.linalg.norm(Pose_Trans - G_Trans)
    
    f.write(f"{k} {RTE_temp}\n")
    RTE += RTE_temp

    intermidate_RRE = R.from_matrix(np.dot(np.linalg.inv(G_Rotat), Pose_Rotat))
    a = intermidate_RRE.as_euler('zyx', degrees=True)
    RRE_temp = sum(abs(number) for number in a)

    g.write(f"{k} {RRE_temp}\n")
    RRE += RRE_temp

    valid_count += 1  # Keep track of valid entries

f.close()
g.close()

