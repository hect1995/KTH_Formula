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

    new_t_imu = t_imu[11:]
    new_acc_imu = acc_imu[:, 11:]
    new_gyro_imu = gyro_imu[:, 11:]

    GPS = np.ndarray((counter), dtype='bool')
    new_mpx = np.ndarray(counter)  # I want to start at 0.11 (same as the first GNSS signal)
    new_mpy = np.ndarray(counter)

    GPS[0] = True
    new_mpx[0] = mpx[0]
    new_mpy[0] = mpy[0]

    # Less new position updates
    for i in range(1, counter):
        if i % 100 == 0:
            GPS[i] = True
        else:
            new_mpx[i] = mpx[i // 100]
            new_mpy[i] = mpy[i // 100]
            GPS[i] = False
    mx = new_acc_imu[0, :]  # acceleration x axis
    my = new_acc_imu[1, :]  # acceleration y axis
    measurements = np.vstack((new_mpx, new_mpy, mx, my))  # stack all of them in kind of a matrix
    #print(measurements.shape)
    x = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T
    P = np.diag(
        [100.0, 100.0, 10.0, 10.0, 1.0, 1.0])  # I assign more uncertainty to the position > velocity > acceleration
    dt = 0.01  # Time Step between Filter Steps (100 Hz)

    A = np.matrix([[1.0, 0.0, dt, 0.0, 1 / 2.0 * dt ** 2, 0.0],
                   [0.0, 1.0, 0.0, dt, 0.0, 1 / 2.0 * dt ** 2],
                   [0.0, 0.0, 1.0, 0.0, dt, 0.0],
                   [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                   [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
    H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]])
    rp = 100.0 ** 2  # Noise of Position Measurement
    R = np.matrix([[rp, 0.0],
                   [0.0, rp]])
    sa = 0.001  # standard deviation acceleration process noise
    G = np.matrix([[1 / 2.0 * dt ** 2],
                   [1 / 2.0 * dt ** 2],
                   [dt],
                   [dt],
                   [1.0],
                   [1.0]])
    Q = G * G.T * sa ** 2
    # Preallocation for Plotting
    xt = []
    yt = []
    Zx = []
    Zy = []
    Px = []
    Py = []
    Kx = []
    Ky = []


    def savestates(x, Z, P, K):
        xt.append(float(x[0]))
        yt.append(float(x[1]))
        Zx.append(float(Z[0]))
        Zy.append(float(Z[1]))
        Px.append(float(P[0, 0]))
        Py.append(float(P[1, 1]))
        Kx.append(float(K[0, 0]))
        Ky.append(float(K[1, 0]))


    I = np.eye(6)
    for filterstep in range(counter):

        # Time Update (Prediction)
        # ========================
        # Project the state ahead
        x = A * x

        # Project the error covariance ahead
        P = A * P * A.T + Q

        # Measurement Update (Correction)
        # ===============================
        # if there is a GPS Measurement
        if GPS[filterstep]:
            # Compute the Kalman Gain
            S = H * P * H.T + R
            K = (P * H.T) * np.linalg.pinv(S)

            # Update the estimate via z
            Z = measurements[:2, filterstep]  # just get the data regarding the position
            ## IN THEORY HAS TO BE (2X1) but does not make sense because I would miss the
            ## contribution of the accelerations if I remove their rows in the measurements
            auxiliar_prod = H * x
            y = Z.T - auxiliar_prod[0,:]
            x = x + (K * y.T)

            # Update the error covariance
            P = (I - (K * H)) * P

        # Save states for Plotting
        savestates(x, Z, P, K)
