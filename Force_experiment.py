#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 22 18:24:14 2021

@author: kike
"""


import numpy as np
import scipy as scipy
from scipy.integrate import odeint, trapz
from scipy import special
import matplotlib.pyplot as plt
import pylab
from scipy.optimize import curve_fit
import matplotlib.animation as animation
from scipy import interpolate
# from cmath import sqrt
params = { 'backend': 'ps',
           'axes.labelsize': 24,
           'font.size': 28,
           'xtick.labelsize': 22,
           'ytick.labelsize': 22,
           'text.usetex': True }
pylab.rcParams.update(params)

plt.close('all')

inp = np.linspace(0,1000,21)
inp2 = np.linspace(0,800, 17)

grams = np.array([0,0, 22, 58, 102, 148, 193, 246, 310, 385, 464, 550, 640, 740, 800, 880, 960, 1025, 1070, 1100, 1130])
grams2 = np.array([0, 0, 19, 51, 86, 129, 171, 211, 280, 340, 426, 505, 608, 675, 770, 825, 940])




u = inp/1000
F = grams/1000*9.81
u = u[0:-4]
F = F[0:-4]
u2 = inp2/1000
F2 = grams2/1000*9.81

FF = np.array([F, F2])

FF = np.mean(FF, axis = 0)


A = np.polyfit(u, F, 2)
A2 = np.polyfit(u2, F2, 2)
def func(x, k):
    return k*x**2


popt, pcov = curve_fit(func, u2, FF)

AA = np.polyfit(u2, FF, 2)
ufit = np.linspace(0,1,1001)

Ffit = np.polyval(A, ufit)
Ffit2 = np.polyval(A2, ufit)
Ffitmean = np.polyval(AA, ufit)
Parabola = popt*ufit**2

plt.figure(figsize = (15,15))
plt.plot(ufit, Ffit, 'r--', linewidth = 2, label = '$Fit$')
plt.plot(u, F, 'o', color = 'b', markersize = 3, label = '$Data$')
plt.plot(ufit, Ffit2, 'r--', linewidth = 2, label = '$Fit$')
plt.plot(ufit, Parabola, 'k', linewidth = 4, label = '$Parabola$')
plt.plot(u2, F2, 'o', color = 'b', markersize = 3, label = '$Data$')
plt.plot(ufit, Ffitmean, 'k-', linewidth = 2, label = '$Fit$')
plt.plot(u2, FF, 'o', color = 'b', markersize = 10, label = '$Data$')
plt.plot(ufit, 10*ufit**2, color = 'b', linewidth = 4,  label = '$Model$')
plt.xlabel('$u$')
plt.ylabel('$F[N]$')
plt.grid(True)
plt.tight_layout()


