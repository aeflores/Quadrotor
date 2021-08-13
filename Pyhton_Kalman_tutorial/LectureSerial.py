#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 11 10:43:38 2021

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


class serialPlot:
    def __init__(self, serialPort='/dev/ttyUSB0', serialBaud=38400, dataNumBytes=2, numVariables=1):
    # def __init__(self, serialPort='/dev/ttyACM0', serialBaud=115200, dataNumBytes=2, numVariables=1):
        self.port = serialPort
        self.baud = serialBaud
        self.dataNumBytes = dataNumBytes
        self.numVariables = numVariables
        self.rawData = bytearray(numVariables * dataNumBytes)
        self.dataType = None
        if dataNumBytes == 2:
            self.dataType = 'h'     # 2 byte integer
        elif dataNumBytes == 4:
            self.dataType = 'f'     # 4 byte float
        self.dataBlock = []
        self.data = []
        for i in range(numVariables):
            self.data.append([])
        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    def getSerialData(self):
        self.serialConnection.reset_input_buffer()
        startTime = datetime.now()
       
        csv_file = open('Flighttest_{}_{}_{}_{}_{}_{}.csv'.format(datetime.now().year,
                                                             datetime.now().month,
                                                             datetime.now().day,
                                                             datetime.now().hour,
                                                             datetime.now().minute,
                                                             datetime.now().second), "w")
        available = True
        while available:
            try:
                self.serialConnection.isOpen()
                self.serialConnection.readinto(self.rawData)      
                dataline = np.array([])
                for j in range(self.numVariables):
                    byteData = self.rawData[(j * self.dataNumBytes):((j + 1) * self.dataNumBytes)]
                    value, = struct.unpack(self.dataType, byteData)
                    dataline = np.append(dataline, value)
                    
                    self.data[j].append(copy.copy(value))
                    csv_file.write('{}\t'.format(dataline[j]))
                csv_file.write('\n')
                # print(dataline)
                csvData = np.array(self.data).transpose()
                print(csvData[-1][:])
            except:
                available = False
        t = (datetime.now() - startTime).total_seconds()
        print("Captured data for %d seconds" % t)
        self.close()
        return csvData

    def close(self):
        self.serialConnection.close()
        print('Disconnected...')


# def main():
#     # portName = 'COM6'
#     portName = '/dev/ttyACM0'
#     baudRate = 115200
#     dataNumBytes = 4        # number of bytes of 1 data point
#     numVariables = 9        # number of plots in 1 graph
#     s = serialPlot(portName, baudRate, dataNumBytes, numVariables)   # initializes all required variables
#     time.sleep(2)
#     s.getSerialData(10)


# if __name__ == '__main__':
#     main()

# portName = 'COM6'
# portName = '/dev/ttyACM1'
ports = serial.tools.list_ports.comports()
portlist = []
for port, desc, hwid in sorted(ports):
        portlist.append(port)
portName = portlist[0]
baudRate = 115200
dataNumBytes = 2        # number of bytes of 1 data point
numVariables = 9        # number of plots in 1 graph



s = serialPlot(portName, baudRate, dataNumBytes, numVariables)   # initializes all required variables
time.sleep(2)
Data = s.getSerialData()
# np.savetxt('MagData05August.csv', Data, delimiter=',', fmt='%f')

# data = np.genfromtxt('Flighttest_2021_8_13_10_36_18.csv', dtype = float, delimiter='\t')

