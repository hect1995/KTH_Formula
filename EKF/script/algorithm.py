#! /usr/bin/env python

from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt
import os

class Extract_data():

    def __init__(self, directory):
        self.directory = directory

    def extract_data(self):
        D = loadmat(os.path.join(self.directory,"data.mat"))

        pos_ned = D['in_data'][0,0]['GNSS'][0,0]['pos_ned']
        hdop = D['in_data'][0,0]['GNSS'][0,0]['HDOP']
        vdop = D['in_data'][0,0]['GNSS'][0,0]['VDOP']
        t_gnss = D['in_data'][0,0]['GNSS'][0,0]['t']
        t_speedometer = D['in_data'][0,0]['SPEEDOMETER'][0,0]['t']
        speed = D['in_data'][0,0]['SPEEDOMETER'][0,0]['speed']
        t_imu = D['in_data'][0,0]['IMU'][0,0]['t']
        acc_imu = D['in_data'][0,0]['IMU'][0,0]['acc']
        gyro_imu = D['in_data'][0,0]['IMU'][0,0]['gyro']

        return pos_ned, hdop, vdop, t_gnss, t_speedometer, speed, t_imu, acc_imu, gyro_imu



if __name__ == '__main__':
    absFilePath = os.path.abspath(__file__)                # Absolute Path of the module
    fileDir = os.path.dirname(os.path.abspath(__file__))   # Directory of the Module
    parentDir = os.path.dirname(fileDir)                   # Directory of the Module directory
    newPath = os.path.join(parentDir, 'data')
    publisher = Extract_data(directory=newPath)
    pos_ned, hdop, vdop, t_gnss, t_speedometer, speed, t_imu, acc_imu, gyro_imu = publisher.extract_data()

    fig = plt.figure(figsize=(16, 5))
    plt.plot(t_speedometer, speed[0])
    plt.ylabel(r'Velocity $m/s$')
    plt.title('Speed vs Time')
    plt.legend(loc='best', prop={'size': 18})
    t = 3
    mpx = np.array(pos_ned[0, :])  # x position from GNSS
    mpy = np.array(pos_ned[1, :])  # y position from GNSS

    # Generate GPS Trigger
    counter = len(t_imu[11:])

    GPS = np.ndarray((counter), dtype='bool')
    new_mpx = np.ndarray(counter)  # I want to start at 0.11 (same as the first GNSS signal)
    new_mpy = np.ndarray(counter)

    GPS[0] = True
    new_mpx[0] = mpx[0]
    new_mpy[0] = mpy[0]

    # Less new position updates
    for i in range(1,counter):
        if i % 100 == 0:
            GPS[i] = True
        else:
            new_mpx[i] = mpx[i // 100]
            new_mpy[i] = mpy[i // 100]
            GPS[i] = False