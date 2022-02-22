#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32

import random
from math import sin, radians
from time import sleep

def trigger():
    pub = rospy.Publisher('trigger', Int32, queue_size=10)
    rospy.init_node('trigger', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        x = 1
        rospy.loginfo(x)
        pub.publish(x)
        rate.sleep()

if __name__ == '__main__':

    try:
        trigger()
    except rospy.ROSInterruptException:
        pass
