#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 16 17:44:46 2021

@author: kike
"""

import numpy as np
import scipy as scipy
import matplotlib.pyplot as plt
import pylab
from scipy import signal
import os
from scipy.optimize import curve_fit
from scipy.io import wavfile


path = '../audiosmotor/Noprop/'

filelist = os.listdir(path)

u = 0.1*np.arange(len(filelist) + 1)


F = np.zeros(u.size)

for i in range(1, len(u)):
    samplerate, Y = wavfile.read(path + filelist[i-1])
    t = np.arange(Y.shape[0])/samplerate
    yl = Y[:,0]
    yr = Y[:,1]
    y               = np.array(yr, dtype = 'float')
    x = y - np.mean(y)
    xf = np.fft.rfft(x)
    sx = (xf*xf.conj()).real
    f = np.arange(0, sx.size)*samplerate/sx.size/2
    df = f[1] - f[0]
    sxf = (xf*xf.conj()).real*f
    fmax = f[np.argmax(sxf[(f<50+40*i)])]
    F[i] = fmax


F = np.array([0, 39., 113.1, 152.8, 175.5, 187.7, 197.4, 202.8, 207.1, 213.6, 216.1])

plt.figure()
plt.plot(u, F)
plt.xlim([0, 1])



#%%
plt.close('all')
    
root = path + 'STE-007.wav'
samplerate, Y = wavfile.read(root)
t = np.arange(Y.shape[0])/samplerate
 
yl = Y[:,0]
yr = Y[:,1]

plt.close('all')


y               = np.array(yr, dtype = 'float')

plt.figure()
plt.plot(t,y)
plt.grid(True)
plt.ylim([-2**15, 2**15])

f               = samplerate
overlapdegree   = 0.95
windowsize      = 2**14
win             = scipy.signal.get_window('blackman', windowsize, True)
F, T, Sxx       = scipy.signal.spectrogram(y, fs=f, window = win, 
                                            noverlap = int(overlapdegree*windowsize), 
                                            detrend = 'linear', scaling = 'density', )
Sxx             = 20*np.log10(Sxx)

plt.figure()
plt.pcolormesh(T,F, Sxx, rasterized = True)
plt.ylim([0, 300])

#%%

plt.close('all')
x = y - np.mean(y)
xf = np.fft.rfft(x)
sx = (xf*xf.conj()).real
f = np.arange(0, sx.size)*samplerate/sx.size/2
df = f[1] - f[0]
sxf = (xf*xf.conj()).real*f

plt.figure()
plt.semilogy(f, sxf)
plt.grid(True)
plt.xlim([0,1000])
plt.ylim([1e4, 10*np.max(sx)])

fmax = f[np.argmax(sxf)]


from scipy.signal import butter, lfilter
from scipy.signal import freqz

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=3):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y


yfilt = butter_bandpass_filter(y, 10, 1500, samplerate)

plt.figure()
plt.plot(t, y, t, yfilt)
plt.ylim([np.min(y), np.max(y)])
plt.grid(True)




#%%


a = 1
c = windowsize
g = a*np.exp(-(f/c)**2/2)

y = np.fft.irfft(g*xf)
y = y - np.mean(y)
yf = np.fft.fft(y)
yf[-yf.size//2:] = 0
env = 2*np.abs(np.fft.ifft(yf))
tmod = t[0:len(env)]
envf = np.fft.rfft(env)
senv = (envf*envf.conj()).real
fenv =  np.arange(0, senv.size)*samplerate/senv.size/2
h   = a*np.exp(-(fenv*10/c)**2/2)
envfilt = np.fft.irfft(envf*h)
tmodmod = tmod[0:len(envfilt)]