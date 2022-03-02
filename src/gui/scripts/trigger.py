#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import numpy as np
import random

def trigger():
    pub = rospy.Publisher('trigger', Int32, queue_size=10)
    rospy.init_node('trigger', anonymous=True)
    
    rate = rospy.Rate(4) # 10hz
    while not rospy.is_shutdown():
        #x = random.randint(1, 190)

        for a in range(-180, 180):
            s = round( float( "{:.02f}".format( np.sin( np.radians(a) ) * 100 ) ) ) // 2

            rospy.loginfo(-s+2)
            pub.publish(-s+2)
            rate.sleep()

if __name__ == '__main__':

    try:
        trigger()
    except rospy.ROSInterruptException:
        pass
