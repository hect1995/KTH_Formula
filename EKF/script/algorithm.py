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