import sys
from python_qt_binding.QtWidgets import QGraphicsScene

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point, Quaternion

#import GUI file
from GLMap import *


class MouseTracker(object):
    def __init__(self, x, y):
        object.__init__(self)    # call the init for the parent class
        self.position = np.array([x, y])
        self.frame = "groundPlane"
        self.publisher = rospy.Publisher(self.frame, PointStamped, queue_size=10)
        #self.setMouseTracking(True)

    def publishPoint(self):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.point = Point(self.position[0], self.position[1], 0)
        self.publisher.publish(msg)
        rospy.loginfo(msg)

    def mouseMoveEvent(self, event):
        self.label.setText('Mouse coords: ( %d : %d )' % (event.x(), event.y()))

