#!/usr/bin/env python

#ROS imports
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

import sys 
import os
from python_qt_binding.QtCore import QPropertyAnimation, Qt
from python_qt_binding.QtGui import QColor 
from python_qt_binding.QtWidgets import QApplication, QGraphicsDropShadowEffect, QMainWindow, QSizeGrip
#from PySide2.QtWidgets import QApplication, QMainWindow, QGraphicsDropShadowEffect, QSizeGrip

#import Qt Material for style sheet
from qt_material import *

#import GUI file
from ui_interface import *

#swril lib
import swri_transform_util

class MainWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        #ROS node initilization
        rospy.init_node('GUI_node', anonymous=True)

        apply_stylesheet(app, theme='dark_cyan.xml')

        # Remove window title bar
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        #set main background to transprent 
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)

        #Shadow effect style
        self.shadow = QGraphicsDropShadowEffect(self)
        self.shadow.setBlurRadius(50)
        self.shadow.setXOffset(0)
        self.shadow.setYOffset(0)
        self.shadow.setColor(QColor(0, 92, 157, 550))

        #Apply shadow to central widget
        self.ui.centralwidget.setGraphicsEffect(self.shadow)

        #Set window icon and name
        self.setWindowIcon(QtGui.QIcon(":/images/images/logo-ADAPT-fauteuille.png"))
        self.setWindowTitle("ADAPT GUI")

        #Window minimize, restore and maximize
        self.ui.minimize_window_button.clicked.connect(lambda:self.showMinimized())
        self.ui.resize_window_button.clicked.connect(lambda:self.restore_or_maximize_window())
        self.ui.close_window_button.clicked.connect(lambda:self.close())

        #Window size grip to resize window
        QSizeGrip(self.ui.window_resize)

        #Display stacked widgets when menu button is pressed 
        self.ui.navigation_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.navigation_widget))
        self.ui.battery_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.battery_widget))
        self.ui.speedSel_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.speedSel_widget))
        self.ui.modeSel_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.modeSel_widget))
        self.ui.statistics_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.statistics_widget))
        self.ui.menu_button.clicked.connect(lambda:self.menuAnimation())

        # To move the window on screen
        def moveWindow(e):
            if self.isMaximized() == False:
                if e.buttons() == Qt.LeftButton:
                    self.move(self.pos() + e.globalPos() - self.cursorPosition)
                    self.cursorPosition = e.globalPos()
                    e.accept()
        self.ui.header_frame.mouseMoveEvent = moveWindow

        #Publish integer value when user use the slider to change the value
        self.ui.horizontalSlider.valueChanged.connect(lambda:self.pubSpeed(self.ui.horizontalSlider.value()))

        #Publish Automode or joystick mode 
        self.ui.autoMode.setCheckable(True)
        self.ui.autoMode.clicked.connect(lambda:self.driveModeSelection())
        self.ui.joystickMode.setCheckable(True)
        self.ui.joystickMode.clicked.connect(lambda:self.driveModeSelection())

        self.show()

    # Animation for menu button
    def menuAnimation(self):
        width = self.ui.leftMenuList_frame.width()
        #if minimized
        if width == 40:
            extendWidth = 200
        #if maximized
        else:
            extendWidth = 40

        #Transisition animation 
        self.animation = QPropertyAnimation(self.ui.leftMenuList_frame, b"minimumWidth") #Animate minimumWidth
        self.animation.setDuration(250)
        self.animation.setStartValue(width)
        self.animation.setEndValue(extendWidth)
        self.animation.setEasingCurve(QtCore.QEasingCurve.InOutQuart)
        self.animation.start()
 
    # To obtain the X,y cordinates when mouse click 
    def mousePressEvent(self, event):
        self.cursorPosition = event.globalPos()
        print(self.cursorPosition)

    # Maximize window function
    def restore_or_maximize_window(self):
        if self.isMaximized():
            self.showNormal()
        else:
            self.showMaximized()    
    
    #Publish the wheelChairSpeed value when user changes the speed 
    def pubSpeed(self, value):
        pub = rospy.Publisher('wheelChairSpeed', Int32, queue_size=10)
        rospy.loginfo(value)
        pub.publish(value)

    #Publish wheel chair drive mode 
    def driveModeSelection(self):
        pub = rospy.Publisher('driveMode', String, queue_size=10)
        source = self.sender()
        if source == self.ui.autoMode:
            rospy.loginfo("Auto mode selected")
            pub.publish("Auto")
        else:
            rospy.loginfo("Joystick mode selected")
            pub.publish("Manual")

if __name__=="__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())