#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point, Quaternion


class MouseTracker(object):
    def __init__(self, x, y):
        object.__init__(self)
        self.msg = PointStamped()
        self.msg.header.stamp = rospy.Time.now()
        self.position = np.array([x, y])
        self.frame = "map_image"
        self.publisher = rospy.Publisher(self.frame, PointStamped, queue_size=10)
        self.publishPoint()

    def publishPoint(self):
        self.msg.header.frame_id = self.frame
        self.msg.point = Point(self.position[0], self.position[1], 0)
        self.publisher.publish(self.msg)
        #rospy.loginfo(self.msg)