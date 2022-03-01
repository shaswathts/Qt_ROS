#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from gui.msg import PointMsg, PointsMsg

class Worker(QObject):
    finished = pyqtSignal()
    intReady = pyqtSignal(int)

    def __init__(self):
        super().__init__()

    @pyqtSlot()
    def qt_callback(self):  # A slot takes no params
        rospy.Subscriber("/trigger", Int32, self.callback)
        rospy.Subscriber("/cluster_list", PointsMsg, self.points)
        rospy.spin()

    def callback(self, data):
        self.intReady.emit(data.data)
        self.finished.emit()

    def points(self, data):
        pass
        #print(data.position.x)
