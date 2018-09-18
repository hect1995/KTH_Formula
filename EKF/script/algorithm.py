#! /usr/bin/env python

from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt
import os

class Process():

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

    def kalman_filter(self):
        pos_ned, hdop, vdop, t_gnss, t_speedometer, speed, t_imu, acc_imu, gyro_imu = self.extract_data()
        # Plot of the speed
        fig = plt.figure(figsize=(16, 5))
        plt.plot(t_speedometer, speed[0])
        plt.ylabel(r'Velocity $m/s$')
        plt.title('Speed vs Time')
        plt.legend(loc='best', prop={'size': 18})

        # Preprocessing
        measurements = self.preprocessing(pos_ned, t_imu, acc_imu)
        self.obtain_variables()

        # Preallocation for Plotting
        self.xt = []
        self.yt = []
        self.Zx = []
        self.Zy = []
        self.Px = []
        self.Py = []
        self.Kx = []
        self.Ky = []

        self.implement_algorithm(measurements)
        self.plot_P(measurements)
        self.plot_K(measurements)


    def preprocessing(self, pos_ned, t_imu, acc_imu ):
        mpx = np.array(pos_ned[0, :])  # x position from GNSS
        mpy = np.array(pos_ned[1, :])  # y position from GNSS

        # Generate GPS Trigger -- The data
        self.counter = len(t_imu[11:])

        new_t_imu = t_imu[11:]
        new_acc_imu = acc_imu[:, 11:]

        self.GPS = np.ndarray((self.counter), dtype='bool')
        new_mpx = np.ndarray(self.counter)  # I want to start at 0.11 (same as the first GNSS signal)
        new_mpy = np.ndarray(self.counter)

        self.GPS[0] = True
        new_mpx[0] = mpx[0]
        new_mpy[0] = mpy[0]

        # Less new position updates
        for i in range(1, self.counter):
            if i % 100 == 0:
                self.GPS[i] = True
            else:
                new_mpx[i] = mpx[i // 100]
                new_mpy[i] = mpy[i // 100]
                self.GPS[i] = False

        mx = new_acc_imu[0, :]  # acceleration x axis
        my = new_acc_imu[1, :]

        self.plot_m(new_mpx, new_mpy, mx, my)
        measurements = np.vstack((new_mpx, new_mpy, mx, my))  # stack all the data I will use for the KF
        return measurements

    def obtain_variables(self):
        self.x = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T # Initial state
        self.P = np.diag(
            [100.0, 100.0, 10.0, 10.0, 1.0, 1.0])  # I assign more uncertainty to the position > velocity > acceleration
        dt = 0.01  # Time Step between Filter Steps (100 Hz)

        self.A = np.matrix([[1.0, 0.0, dt, 0.0, 1 / 2.0 * dt ** 2, 0.0],
                       [0.0, 1.0, 0.0, dt, 0.0, 1 / 2.0 * dt ** 2],
                       [0.0, 0.0, 1.0, 0.0, dt, 0.0],
                       [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                       [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]) # Dynamic matrix

        self.H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]]) # Measurement matrix
        rp = 100.0 ** 2  # Noise of Position Measurement
        self.R = np.matrix([[rp, 0.0],
                       [0.0, rp]]) # Measurement noise variable
        sa = 0.001  # standard deviation acceleration process noise
        self.G = np.matrix([[1 / 2.0 * dt ** 2],
                       [1 / 2.0 * dt ** 2],
                       [dt],
                       [dt],
                       [1.0],
                       [1.0]])
        self.Q = self.G * self.G.T * sa ** 2 # Process noise covariance

    def savestates(self, Z, K):
        self.xt.append(float(self.x[0]))
        self.yt.append(float(self.x[1]))
        self.Zx.append(float(Z[0]))
        self.Zy.append(float(Z[1]))
        self.Px.append(float(self.P[0, 0]))
        self.Py.append(float(self.P[1, 1]))
        self.Kx.append(float(K[0, 0]))
        self.Ky.append(float(K[1, 0]))

    def implement_algorithm(self, measurements):
        I = np.eye(6)
        for filterstep in range(self.counter):
            # Time Update (Prediction)
            # ========================
            # Project the state ahead
            self.x = self.A * self.x

            # Project the error covariance ahead
            self.P = self.A * self.P * self.A.T + self.Q

            # Measurement Update (Correction)
            # ===============================
            # if there is a GPS Measurement
            if self.GPS[filterstep]:
                # Compute the Kalman Gain
                S = self.H * self.P * self.H.T + self.R
                K = (self.P * self.H.T) * np.linalg.pinv(S)

                # Update the estimate via z
                Z = measurements[:2, filterstep]  # just get the data regarding the position
                auxiliar_prod = self.H * self.x
                self.y = Z.T - auxiliar_prod[0, :]
                self.x = self.x + (K * self.y.T)

                # Update the error covariance
                self.P = (I - (K * self.H)) * self.P

            # Save states for Plotting
            self.savestates(Z, K)

    def plot_m(self, new_mpx, new_mpy, mx, my):
        fig = plt.figure(figsize=(16, 12))
        plt.subplots_adjust(hspace=1)
        plt.subplot(411)
        plt.step(range(self.counter), new_mpx, label='$x$')
        plt.ylabel(r'X Position $m$')
        plt.title('Measurements')
        plt.ylim([-100, 100])
        plt.legend(loc='best', prop={'size': 18})

        plt.subplot(412)
        plt.step(range(self.counter), new_mpy, label='$y$')
        plt.ylabel(r'Y Position $m$')
        plt.title('Measurements')
        plt.ylim([-100, 100])
        plt.legend(loc='best', prop={'size': 18})

        plt.subplot(413)
        plt.step(range(self.counter), mx, label='$y$')
        plt.ylabel(r'X Acceleration $m$')
        plt.title('Measurements')
        plt.ylim([-10, 10])
        plt.legend(loc='best', prop={'size': 18})

        plt.subplot(414)
        plt.step(range(self.counter), my, label='$a_x$')
        plt.ylabel(r'Y Acceleration $m/s^2$')
        plt.ylim([-10, 10])
        plt.legend(loc='best', prop={'size': 18})

    def plot_P(self, measurements):
        fig = plt.figure(figsize=(16, 9))
        # plt.subplot(211)
        plt.plot(range(len(measurements[0])), self.Px, label='$x$')
        plt.plot(range(len(measurements[0])), self.Py, label='$y$')
        plt.title('Uncertainty (Elements from Matrix $P$)')
        plt.legend(loc='best', prop={'size': 22})
        plt.xlabel('Filter Step')
        plt.ylabel('')
        plt.legend(loc='best', prop={'size': 22})

    def plot_K(self, measurements):
        fig = plt.figure(figsize=(16, 9))
        plt.plot(range(len(measurements[0])), self.Kx, label='Kalman Gain for $x$')
        plt.plot(range(len(measurements[0])), self.Ky, label='Kalman Gain for $y$')

        plt.xlabel('Filter Step')
        plt.ylabel('')
        plt.title('Kalman Gain (the lower, the more the measurement fullfill the prediction)')
        plt.legend(loc='best', prop={'size': 18})


if __name__ == '__main__':
    absFilePath = os.path.abspath(__file__)                # Absolute Path of the module
    fileDir = os.path.dirname(os.path.abspath(__file__))   # Directory of the Module
    parentDir = os.path.dirname(fileDir)                   # Directory of the Module directory
    newPath = os.path.join(parentDir, 'data')
    publisher = Process(directory=newPath)
    publisher.kalman_filter()
