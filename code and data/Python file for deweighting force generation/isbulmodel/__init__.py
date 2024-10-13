"""
Created on Mon May 20 12:15:36 2024

@author: vcrocher
"""

%run ISBUL.py
%run Jerk.py
#%run MOCO_UL_ID.py
%run OpenSIMMoBLARMSUtils.py
%run Deweighting.py
import ISBUL
import Deweighting
import OpenSIMMoBLARMSUtils
import numpy as np
 
 
DE = Deweighting.Deweighting()
DE.ArmMassFromBodyMass(46.76471)
q = np.deg2rad([90.012,90,0,49,0,0,0])
#q = [0,0,0,0,0,0,0]
q_ry = np.array(q)
Arr = DE.DeweightForce(q_ry)
Nf = np.array([
    [-1, 0, 0],
    [0, 0, 1],
    [0, 1, 0]
])
NFC = np.dot(Nf, Arr)
print(NFC)