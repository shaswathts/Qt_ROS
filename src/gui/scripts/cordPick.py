#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point, Quaternion

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MouseTracker(object):

    """
    This class publishes the user touch inputs as goal
    in the /t265_odom/sample frame of reference.
    """

    def __init__(self, x, y,cluster_name):
        object.__init__(self)
        
        self.position = np.array([x, y])
        self.publisher = rospy.Publisher("/touch_points", PointStamped, queue_size=1)
        self.publishPoint()

        """ client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/t265/odom/sample"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.x = y
        rospy.loginfo(goal)
        client.send_goal(goal)
        rospy.set_param('/cluster_name', 'cluster'+cluster_name)
        rospy.set_param('/move', True) """

    def publishPoint(self):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/d435_color_optical_frame"
        msg.point.x = self.position[0]
        msg.point.y = self.position[1]
        msg.point.z = 0
        rospy.loginfo(msg)
        self.publisher.publish(msg)
        