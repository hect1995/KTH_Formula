#!/usr/bin/env python
import rospy
import os
from scipy.io import loadmat
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Point, Pose


if __name__ == '__main__':
    try:
        absFilePath = os.path.abspath(__file__)                # Absolute Path of the module
        fileDir = os.path.dirname(os.path.abspath(__file__))   # Directory of the Module
        parentDir = os.path.dirname(fileDir)                   # Directory of the Module directory
        newPath = os.path.join(parentDir, 'data')

        D = loadmat(os.path.join(newPath,"data.mat"))
        pos_ned = D['in_data'][0,0]['GNSS'][0,0]['pos_ned']
        hdop = D['in_data'][0,0]['GNSS'][0,0]['HDOP']
        vdop = D['in_data'][0,0]['GNSS'][0,0]['VDOP']
        t_gnss = D['in_data'][0,0]['GNSS'][0,0]['t']
        t_speedometer = D['in_data'][0,0]['SPEEDOMETER'][0,0]['t']
        speed = D['in_data'][0,0]['SPEEDOMETER'][0,0]['speed']
        t_imu = D['in_data'][0,0]['IMU'][0,0]['t']
        acc_imu = D['in_data'][0,0]['IMU'][0,0]['acc']
        gyro_imu = D['in_data'][0,0]['IMU'][0,0]['gyro']
        

        mpx = np.array(pos_ned[0, :])  # x position from GNSS
        mpy = np.array(pos_ned[1, :])  # y position from GNSS
        mpz = np.array(pos_ned[2, :])  # y position from GNSS

        # Generate GPS Trigger -- The data
        counter = len(t_imu[11:])

        new_t_imu = t_imu[11:]
        new_acc_imu = acc_imu[:, 11:]

        GPS = np.ndarray((counter), dtype='bool')
        new_mpx = np.ndarray(counter)  # I want to start at 0.11 (same as the first GNSS signal)
        new_mpy = np.ndarray(counter)
        new_mpz = np.ndarray(counter)


        GPS[0] = True
        new_mpx[0] = mpx[0]
        new_mpy[0] = mpy[0]
        new_mpz[0] = mpz[0]

        # Less new position updates
        for i in range(1, counter):
            if i % 100 == 0:
                GPS[i] = True
            else:
                new_mpx[i] = mpx[i // 100]
                new_mpy[i] = mpy[i // 100]
                new_mpz[i] = mpz[i // 100]

                GPS[i] = False

        mx = new_acc_imu[0, :]  # acceleration x axis
        my = new_acc_imu[1, :]
        mz = new_acc_imu[2, :]

        measurements = np.vstack((new_mpx, new_mpy, new_mpz, mx, my, mz))
        imu_pub = rospy.Publisher('imu_hector', Imu, queue_size=20)
        pose_pub = rospy.Publisher('pose_hector', Pose, queue_size=20)

        rospy.init_node('nodeA', anonymous=True)
        rate = rospy.Rate(100) #100hz
        
        i = 0
        while (not rospy.is_shutdown()) and i<counter:
            imu_message = Imu()
            acc_message = Vector3()
            coordinates_gnss = Point()
            position_gnss = Pose()
            
            acc_message.x = mx[i]
            acc_message.y = my[i]
            acc_message.z = mz[i]
            imu_message.linear_acceleration = acc_message
            coordinates_gnss.x = new_mpx[i]
            coordinates_gnss.y = new_mpy[i]
            coordinates_gnss.z = new_mpz[i]
            position_gnss.position = coordinates_gnss

            imu_pub.publish(imu_message)
            pose_pub.publish(position_gnss)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass