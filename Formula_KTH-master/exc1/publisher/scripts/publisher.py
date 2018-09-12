#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

class Publisher():

    def __init__(self, K, N, F):
        self.K = K
        self.N = N
        self.F = F

    def talker(self):
        pub = rospy.Publisher('esteban', Float32, queue_size=10)
        rospy.init_node('nodeA', anonymous=True)
        rate = rospy.Rate(self.F) 
        while not rospy.is_shutdown():
            pub.publish(self.K)
            self.K = self.K + self.N
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = Publisher(K= 2, N= 4, F= 0.05)
        publisher.talker()
    except rospy.ROSInterruptException:
        pass