#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, String
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np


class Worker(QObject):
    finished = pyqtSignal(name="Exit/quit worker thread")
    signal1 = pyqtSignal(list, name="Posistion change")
    signal2 = pyqtSignal(object, name="Map update")
    signal3 = pyqtSignal(list, name="Battery status")
    signal4 = pyqtSignal(int, list, name="Wheelchair information")

    def __init__(self):
        super().__init__()
        self.sub1 = rospy.Subscriber("/map", OccupancyGrid, self.qt_map)
        self.sub2 = rospy.Subscriber("/t265/odom/sample", Odometry, self.callback)
        self.sub3 = rospy.Subscriber("/batteryInfo", Int32, self.battery_info)
        self.sub4 = rospy.Subscriber("/wheelchairInfo", String, self.chair_info)
        self.pub1 = rospy.Publisher('/qtMap', Image, queue_size=1)

    """@pyqtSlot()
    def qt_callback(self):  # A slot takes no params
        self.intReady.emit(self.pose)
        self.image.emit(self.map_image)
        self.finished.emit()
        pass """

    def callback(self, data):

        pose = [data.pose.pose.position.x*40, data.pose.pose.position.y*40, data.pose.pose.orientation.z*40]
        self.signal1.emit(pose) #self.pose = pose
        #rospy.loginfo("Posistion info emitted to Qt")

    def qt_map(self, data):
        """ print("Recived grid map from ROS") """
        width = data.info.width
        height = data.info.height
        resolution = data.info.resolution
        pixels = data.data

        imgData = np.array([pixels])
        
        imgData = np.where(imgData==0, 255,imgData)
        imgData = np.where(imgData==-1, 205,imgData)
        imgData = np.where(imgData==100, 0,imgData)
        imgData = imgData.reshape(height, width)

        """ print(imgData.dtype) """
        imgData = np.uint8(imgData)

        self.signal2.emit(imgData) # self.map_image = imgData
        rospy.loginfo("Grid map emitted to Qt")
        self.pubImage(imgData)
        return imgData

    def battery_info(self, data):
        battery_info = []
        charge = data.charge
        status = data.status
        time_left = data.battery_time
        plug = data.plugin
        battery_info = [charge, status, time_left, plug]
        self.signal3.emit(battery_info)

    def chair_info(self, data):
        self.signal4.emit(data)

    def pubImage(self, image):
        self.bridge = CvBridge()
        img = self.bridge.cv2_to_imgmsg(image)
        img.header.stamp = rospy.Time.now()

        self.pub1.publish(img)

if __name__ == "__main__":
    x = Worker()
    rospy.spin()