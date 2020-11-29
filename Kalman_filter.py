#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 29 18:13:50 2020

@author: kike
"""


import numpy as np
import scipy as scipy
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path

file = 'IMU_calibration.txt'

data    = pd.read_csv(file,sep="\t",names=['time', 'AccelX', 'AccelY','AccelZ','GyroX','GyroY','GyroZ','MagX','MagY','MagZ']) 
# data    = data.replace('(\w| )*=','',regex=True)
# data    = data.drop(['Mode'], axis=1)
# data    = data.apply(pd.to_numeric)


time = np.array((data['time'] - data['time'][0])/1e6)

AccelX = np.array(data['AccelX'])
AccelY = np.array(data['AccelY'])
AccelZ = np.array(data['AccelZ'])


GyroX = np.array(data['GyroX'])
GyroY = np.array(data['GyroY'])
GyroZ = np.array(data['GyroZ'])

MagX = np.array(data['MagX'])
MagY = np.array(data['MagY'])
MagZ = np.array(data['MagZ'])

MagXbias    = -7.5
MagXSF      = 1.121387

MagYbias    = 9
MagYSF      = 1.164484

MagZbias    = -20
MagZSF      = 0.824261


# GyroXbias   = 0.026320
# GyroYbias   = 0.02487275
# GyroZbias   = -0.017317


# MagX = (MagX - MagXbias)*MagXSF
# MagY = (MagY - MagYbias)*MagYSF
# MagZ = (MagZ - MagZbias)*MagZSF


plt.figure(1)
plt.plot(time, MagX, time, MagY, time, MagZ,  linewidth = 3)
# plt.semilogy(xf, yf, 'k', linewidth=3)
# plt.plot(xf, yf, 'k', linewidth=3)
# plt.axis('scaled')
plt.grid(True, which="both")
plt.show()
plt.xlabel('$t[s]$')
# plt.ylabel('$x[m]$')
# plt.ticklabel_format(axis="y", style="sci", scilimits=(0,0))
# plt.ylim(top=)  # adjust the top leaving bottom unchanged
# plt.ylim(bottom=0)  # adjust the bottom leaving top unchanged
plt.xlim(left=0)  # adjust the top leaving bottom unchanged
# plt.xlim(right=1500)  # adjust the bottom leaving top unchanged
plt.show()

plt.figure(2)
plt.plot(time, GyroX, time, GyroY, time, GyroZ,  linewidth = 3)
# plt.semilogy(xf, yf, 'k', linewidth=3)
# plt.plot(xf, yf, 'k', linewidth=3)
# plt.axis('scaled')
plt.grid(True, which="both")
plt.show()
plt.xlabel('$t[s]$')
# plt.ylabel('$x[m]$')
# plt.ticklabel_format(axis="y", style="sci", scilimits=(0,0))
# plt.ylim(top=)  # adjust the top leaving bottom unchanged
# plt.ylim(bottom=0)  # adjust the bottom leaving top unchanged
plt.xlim(left=0)  # adjust the top leaving bottom unchanged
# plt.xlim(right=1500)  # adjust the bottom leaving top unchanged
plt.show()