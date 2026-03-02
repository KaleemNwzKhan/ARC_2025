#!/bin/bash

# Source the base environment
source /home/kk5271/anaconda3/bin/activate base
rm -rf RTE_Direct.txt rm -rf RRE_Direct.txt RTE_NDT.txt RRE_NDT.txt
bash E2E_NDT_Error.sh
bash E2E_Direct_Error.sh 
python3 Direct_V_L_Error_Plot.py 
