#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int32, String

class Subscriber():

    def __init__(self, Q, F):
        self.Q = Q
        self.F = F
        self.publish_result = rospy.Publisher('/kthfs/result', Float32, queue_size=10)


    def callback(self, data):
        rospy.loginfo("{}".format(data.data))
        new_result = data.data / self.Q
        self.publish_result.publish(new_result)

        
    def listener(self):
        rospy.init_node('nodeB', anonymous=True)
        rospy.Rate(self.F)
        rospy.Subscriber("esteban", Float32, self.callback)

        rospy.spin()

if __name__ == '__main__':
    try:
        subscriber = Subscriber(Q= 0.15, F= 0.05)
        subscriber.listener()
    except rospy.ROSInterruptException:
        pass