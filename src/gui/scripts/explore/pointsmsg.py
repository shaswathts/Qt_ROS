#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from gui.msg import PointMsg, PointsMsg


def callback(data):
    """ print(data.geometry_msgs.position) """
    """ print(data) """
    points = PointsMsg()
    points.header.stamp = data.header.stamp
    points.header.frame_id = data.header.frame_id
    #point = po
    # ints.deserialize(data)
    print (data)

def trigger()
    rospy.init_node("PointsMsg", anonymous=True)

    rospy.Subscriber("/cluster_list", PointsMsg, callback)

    rospy.spin()

if __name__ == '__main__':

    try:
        trigger()
    except rospy.ROSInterruptException:
        pass
