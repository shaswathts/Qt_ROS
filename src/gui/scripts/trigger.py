#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import numpy as np
import random

def trigger():
    pub = rospy.Publisher('trigger', Int32, queue_size=10)
    rospy.init_node('trigger', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        x = random.randint(1, 190)

        """  x = np.arange(0,4*np.pi,0.1)   # start,stop,step
        y = np.sin(x) """

        rospy.loginfo(x)
        pub.publish(x)
        rate.sleep()

if __name__ == '__main__':

    try:
        trigger()
    except rospy.ROSInterruptException:
        pass
