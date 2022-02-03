#!/usr/bin/env python

#ROS imports
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import tf2_ros

import sys 
import os
from python_qt_binding.QtCore import QPropertyAnimation, Qt, QLineF
from python_qt_binding.QtGui import QColor, QPen 
from python_qt_binding.QtWidgets import QApplication, QGraphicsDropShadowEffect, QMainWindow, QSizeGrip, QOpenGLWidget
from python_qt_binding import QtOpenGL
#from PySide2.QtWidgets import QApplication, QMainWindow, QGraphicsDropShadowEffect, QSizeGrip

import OpenGL.GL as gl        # python wrapping of OpenGL
from OpenGL import GLU        # OpenGL Utility Library, extends OpenGL functionality

#import Qt Material for style sheet
from qt_material import *

#import GUI file
from ui_interface import *

from GItems import Graphicsscene
#import GLMap
#from GLMap import *
#from cordPick import *
#from smpxy import MyFrame


class MainWindow(QMainWindow):
    """
    This class defines the Qt widget for the application, to which we add Qt widgets for
    OpenGL graphics, user input, etc.
    
    """

    def __init__(self):
        """ Initialize the Qt MainWindow.
        
        Initializes the Qt main window by setting up the window from Ui_interface, initializing the GLWidget,
        adding GUI elements, creating a timed rendering loop and setting up signals and slot connections.
        
        Args:
            (None)
        Returns:
           (None)
        """
        QMainWindow.__init__(self) # call the init for the parent class
        
        # Set up the UI window
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self) 

        # Set up the OpenGL window
        #self.glWidget = GLWidget(self)
        #self.initGUI()

        #self.axis = MyFrame()
        self.initGUI()

        # Create a timer and connect its signal to the QGLWidget update function
        """
        timer = QtCore.QTimer(self)
        timer.setInterval(20)   # period, in milliseconds
        timer.timeout.connect(self.glWidget.updateGL)
        timer.start()
        """

        #apply_stylesheet(app, theme='dark_cyan.xml')

        # Remove window title bar
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        #set main background to transprent 
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)

        # Shadow effect style
        self.shadow = QGraphicsDropShadowEffect(self)
        self.shadow.setBlurRadius(50)
        self.shadow.setXOffset(0)
        self.shadow.setYOffset(0)
        self.shadow.setColor(QColor(0, 92, 157, 550))

        # Apply shadow to central widget
        self.ui.centralwidget.setGraphicsEffect(self.shadow)

        # Set window icon and name
        self.setWindowIcon(QtGui.QIcon(":/images/images/logo-ADAPT-fauteuille.png"))
        self.setWindowTitle("ADAPT GUI")

        # Window minimize, restore and maximize
        self.ui.minimize_window_button.clicked.connect(lambda:self.showMinimized())
        self.ui.resize_window_button.clicked.connect(lambda:self.restore_or_maximize_window())
        self.ui.close_window_button.clicked.connect(lambda:self.close())

        # Window size grip to resize window
        QSizeGrip(self.ui.window_resize)

        # Display stacked widgets when menu button is pressed 
        self.ui.navigation_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.navigation_widget))
        self.ui.battery_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.battery_widget))
        self.ui.speedSel_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.speedSel_widget))
        self.ui.modeSel_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.modeSel_widget))
        self.ui.statistics_button.clicked.connect(lambda:self.ui.stackedWidget.setCurrentWidget(self.ui.statistics_widget))
        self.ui.menu_button.clicked.connect(lambda:self.menuAnimation())

        # To move the window 
        def moveWindow(e):
            if self.isMaximized() == False:
                if e.buttons() == Qt.LeftButton:
                    self.move(self.pos() + e.globalPos() - self.cursorPosition)
                    self.cursorPosition = e.globalPos()
                    e.accept()

        self.ui.header_frame.mouseMoveEvent = moveWindow
        self.ui.graphicsView.mousePressEvent = self.mouseEvent
        self.ui.graphicsView.mouseMoveEvent = self.toggleDragMode

        # Publish integer value when user use the slider to change the value
        self.ui.horizontalSlider.valueChanged.connect(lambda:self.pubSpeed(self.ui.horizontalSlider.value()))
        self.ui.horizontalSlider.valueChanged.connect(lambda:self.rotate_item(self.ui.horizontalSlider.value()))

        # Publish Automode or joystick mode 
        self.ui.autoMode.setCheckable(True)
        self.ui.autoMode.clicked.connect(lambda:self.driveModeSelection())
        self.ui.joystickMode.setCheckable(True)
        self.ui.joystickMode.clicked.connect(lambda:self.driveModeSelection())
        
    def initGUI(self):
        """ Initialize the Qt GUI elements for the main window.
        
        Initializes the Qt main window GUI elements.  Sets up a central widget with a vertical
        layout and adds the GLWidget followed by user input elements (sliders).
        
        Args:
            (None)
        Returns:
           (None)
        
        """

        self.zoom = 2
        self.angle = 0
        self._empty = True
        self._scene = QtWidgets.QGraphicsScene()
        self._photo = QtWidgets.QGraphicsPixmapItem()    
        self._view = Graphicsscene()
        self.ui.graphicsView.setMouseTracking(True)

        self.ui.graphicsView.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.ui.graphicsView.setResizeAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.ui.graphicsView.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.ui.graphicsView.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        
        self.setPhoto(QtGui.QPixmap('/home/intern/adapt_Pyrqt/src/smp_gui/src/second_map.pgm'))
        robot = self._view.addRobot()
        self._scene.addItem(robot)
        robot.setZValue(500)
        
        self.objPose()
        for cluster in self.cluster:
            self._scene.addItem(cluster)
            cluster.setZValue(500)
            cluster.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, True)
            cluster.setFlag(QtWidgets.QGraphicsItem.ItemSendsGeometryChanges, True)
        self._photo.setPos(130, 130)
        self._scene.addItem(self._photo)
        self.ui.graphicsView.setScene(self._scene)
        #self.toggleDragMode()
        
    # Get cluster data from the TF buffer from semantic map(dbscan)
    def objPose(self):
        i = 0
        get_pose = True
        self.cluster_list = []
        pix_scale = 37 #rospy.get_param('pixel_scale')

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        
        while get_pose:
            i = i+1
            print('cluster' + str(i))
            try:
                trans = tfBuffer.lookup_transform("map", "cluster"+str(i), rospy.Time(0), rospy.Duration(5))
                cluster = [trans.transform.translation.x * pix_scale, trans.transform.translation.y * pix_scale]
                self.cluster_list.append(cluster)
            except (tf2_ros.LookupException):
                get_pose = False
                continue

        trans2 = tfBuffer.lookup_transform("map", "d435_color_optical_frame", rospy.Time(0), rospy.Duration(5))
        self.cluster_list.append([trans2.transform.translation.x * pix_scale, trans2.transform.translation.y * pix_scale])
        
        self.cluster = self._view.objDet(self.cluster_list)
    
    # Map image is true/false
    def hasPhoto(self):
        return not self._empty  

    # Update the graphic window after changes
    def updateView(self):
        #self.ui.graphicsView.setTransform(QtGui.QTransform().scale(self.zoom, self.zoom).rotate(self.angle))
        transform = QtGui.QTransform()
        transform.scale(self.zoom, self.zoom)
        transform.rotate(self.angle)
        self.ui.graphicsView.setTransform(transform)

    # Initilization of map image
    def setPhoto(self, pixmap=None):
        if pixmap and not pixmap.isNull():
            self._empty = False
            self._photo.setPixmap(pixmap)
        else:
            self._empty = True
            self._photo.setPixmap(QtGui.QPixmap())
        self.updateView()

    # Capturing mouse wheel events to zoom in/out
    def wheelEvent(self, event):
        if self.hasPhoto():
            moose = event.angleDelta().y()/120
            if moose > 0:
                self.zoomIn()
            elif moose < 0:
                self.zoomOut()
    
    def zoomIn(self):
        self.zoom *= 1.05
        self.updateView()

    def zoomOut(self):
        self.zoom /= 1.05
        self.updateView()

    def zoomReset(self):
        self.zoom = 1
        self.updateView()

    # To get the pixel cordinates from the map image
    def mouseEvent(self, event):
        if self.hasPhoto:
            if self._photo.isUnderMouse() and event.buttons() == Qt.MiddleButton:
                print(self.ui.graphicsView.mapToScene(event.pos()).toPoint())

    # Rotate the map with respect to the pose frame orientation 
    def rotate_item (self, value) :
        self.angle = value 
        self.updateView() 

    # Drag function         
    def toggleDragMode(self, event):
        '''if event.buttons() == QtCore.Qt.RightButton:
            if self.hasPhoto():
                self.ui.graphicsView.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
            else:
                self.ui.graphicsView.setDragMode(QtWidgets.QGraphicsView.NoDrag)
                #self.image = False'''

        if self.ui.graphicsView.dragMode() == QtWidgets.QGraphicsView.ScrollHandDrag:
            self.ui.graphicsView.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        elif not self._photo.pixmap().isNull():
            self.ui.graphicsView.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
    
    # Animation for menu button
    def menuAnimation(self):
        width = self.ui.leftMenuList_frame.width()
        # if minimized
        if width == 40:
            extendWidth = 200
        # if maximized
        else:
            extendWidth = 40

        # Transisition animation 
        self.animation = QPropertyAnimation(self.ui.leftMenuList_frame, b"minimumWidth") #Animate minimumWidth
        self.animation.setDuration(250)
        self.animation.setStartValue(width)
        self.animation.setEndValue(extendWidth)
        self.animation.setEasingCurve(QtCore.QEasingCurve.InOutQuart)
        self.animation.start()
 
    # To obtain the X,y cordinates when mouse click 
    def mousePressEvent(self, event):
        self.cursorPosition = event.globalPos()

    # Maximize window function
    def restore_or_maximize_window(self):
        if self.isMaximized():
            self.showNormal()
        else:
            self.showMaximized()    
    
    # Publish the wheelChairSpeed value when user changes the speed 
    def pubSpeed(self, value):
        pub = rospy.Publisher('wheelChairSpeed', Int32, queue_size=10)
        rospy.loginfo(value)
        #print(value)
        pub.publish(value)

    # Publish wheel chair drive mode 
    def driveModeSelection(self):
        pub = rospy.Publisher('driveMode', String, queue_size=10)
        source = self.sender()
        if source == self.ui.autoMode:
            rospy.loginfo("Auto mode selected")
            pub.publish("Auto")
            #print("Auto mode selected")
        else:
            rospy.loginfo("Joystick mode selected")
            pub.publish("Manual")
            #print("Joystick mode selected")

if __name__=="__main__":
    app = QApplication(sys.argv)
    
    # ROS node initilization
    rospy.init_node('GUI_node', anonymous=True)
    
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())