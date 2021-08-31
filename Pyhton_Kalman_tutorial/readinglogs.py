#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 15 19:21:17 2021

@author: kike
"""


from datetime import datetime
import serial
import struct
import numpy as np
import copy
import time
import serial.tools.list_ports
import matplotlib.pyplot as plt

plt.close('all')

# data = np.genfromtxt('../Flighttest_2021_8_16_9_50_43.csv', dtype = float, delimiter='\t')

data = np.genfromtxt('../Flighttest_2021_8_26_9_39_24.csv', dtype = float, delimiter='\t')

E1 = data[:,-5] -1000
E2 = data[:,-4] -1000
E3 = data[:,-3] -1000
E4 = data[:,-2] -1000

pitch = data[:,4]/100
roll = data[:,5]/100
height = data[:,6]


dt = data[:,2]/1000

time = data[:,-1]
    
# plt.close('all')
    
plt.figure()
# plt.plot(time, pitch, time, roll, time, height)
plt.plot(time, pitch, time, roll, '-o')
plt.grid(True)
plt.ylim([-25, 25])


plt.figure()
# plt.plot(time, pitch, time, roll, time, height)
plt.subplot(3,1,1)
plt.plot(time, E1, time, E2, time, E3, time, E4)
plt.grid(True)
plt.ylim([0,1000])
# plt.xlim([4000, 6000])
plt.subplot(3,1,2)
plt.plot(time, pitch, time, roll, '-o')
plt.grid(True)
plt.ylim([-45, 45])
# plt.xlim([4000, 6000])
plt.subplot(3,1,3)
plt.plot(time, height, '-o')
plt.grid(True)
plt.ylim([0, 1200])
# plt.xlim([4000, 6000])


u1 = E1+E2+E3+E4 - 4*400
PIDpitch = (E1 - E2)/2
PIDroll = (E3-E2)/2

plt.figure()
plt.plot(time, height, time, u1)
plt.grid(True)


plt.figure()
plt.subplot(2,1,1)
plt.plot(time, pitch, time, roll)
plt.grid(True)
plt.ylim([-45, 45])
plt.subplot(2,1,2)
plt.plot(time, PIDpitch, time, PIDroll)
plt.grid(True)