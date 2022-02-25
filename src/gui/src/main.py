#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import Int16, Int32, String
from geometry_msgs.msg import PointStamped
import tf2_ros

# Qt imports 
from python_qt_binding.QtCore import QPropertyAnimation, Qt, QLineF, QPointF
from python_qt_binding.QtGui import QColor, QPen 
from python_qt_binding.QtWidgets import QApplication, QGraphicsDropShadowEffect, QMainWindow, QSizeGrip, QOpenGLWidget
from python_qt_binding import QtOpenGL

# utils
import cv2
from qt_material import *
from ui_interface import *
from GItems import Graphicsscene
from cv_transform import UpdateTransformation, opencv
import worker


class MainWindow(QMainWindow):

    """ This class defines the Qt widget for the application, to which we add Qt
    widgets for graphics, user input, etc.

    """
    def __init__(self):
        
        """ Initialize the Qt MainWindow.
        Initializes the Mainwindow by setting up the window from Ui_interface,
        QGraphic Widget, QThread for worker thread optional QTimer.

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

        # create Worker and Thread inside the MainWindow class
        self.obj = worker.Worker()  # no parent!
        self.thread = QtCore.QThread()  # no parent!

        # Connect Worker`s Signals to MainWindow method slots to post data.
        self.obj.intReady.connect(self.wheelchair_pose)

        # Move the Worker object to the Thread object
        self.obj.moveToThread(self.thread)

        # Connect Worker Signals to the Thread slots
        self.obj.finished.connect(self.thread.quit)

        # Connect Thread started signal to Worker operational slot method
        self.thread.started.connect(self.obj.qt_callback)

        # Start the thread
        self.thread.start()

        # Initilize the graphic widget
        self.initGUI()

        # Create a timer and connect its signal to wheelchair_pose to update
        """ timer = QtCore.QTimer(self)
        timer.setInterval(100)   # period, in milliseconds
        timer.timeout.connect(self.wheelchair_pose)
        timer.start() """

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
        
        # Publish Automode or joystick mode 
        self.ui.autoMode.setCheckable(True)
        self.ui.autoMode.clicked.connect(lambda:self.driveModeSelection())
        self.ui.joystickMode.setCheckable(True)
        self.ui.joystickMode.clicked.connect(lambda:self.driveModeSelection())

    def initGUI(self):

        """ Initialize the Qt graphical elements for the main window.

        Args:
            (None)
        Returns:
           (None)

        """

        self.ui.graphicsView.setMouseTracking(True)
        self.ui.graphicsView.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.ui.graphicsView.setResizeAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.ui.graphicsView.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.ui.graphicsView.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        self._zoom = 1
        self._empty = True
        self._scene = QtWidgets.QGraphicsScene()
        self._photo = QtWidgets.QGraphicsPixmapItem()
        self._view = Graphicsscene()
        self._cv = opencv()

        robot = self._view.addRobot()
        self._scene.addItem(robot)
        robot.setZValue(500)

        self.cluster_pose()
        for cluster in self.cluster:
            self._scene.addItem(cluster)
            cluster.setZValue(500)
            cluster.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, True)
            cluster.setFlag(QtWidgets.QGraphicsItem.ItemSendsGeometryChanges, True)

        self._scene.addItem(self._photo)
        self.ui.graphicsView.setScene(self._scene)

    def convert_cv_qt(self, cv_img):

        """ Convert from an opencv image to QPixmap

        Args:
            cv_img: OpenCV image
        Returns:
            OpenCV image converted to QPixmap format
   
        """

        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        bytes_per_line = channel * width
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)

        return QtGui.QPixmap.fromImage(convert_to_Qt_format)

    def wheelchair_pose(self, pose):

        """ Get odometry coordinates from ROS topic (name)

        Args:
            pose: List of [x, y, theta]
        Returns:
            (None)

        """

        _get_pose = True
        xy = [pose, pose+28, pose]
        h = UpdateTransformation(xy, _get_pose)
        self.image = self.convert_cv_qt(h.transformation) #h.transformation) #self.rob)
        self.setPhoto(self.image)     

    def cluster_pose(self):

        """ Get cluster coordinates from ROS

        Args:
            (None)
        Returns:
            Appends cluster coordinate to a list to render on graphic widget.

        """

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
        
        self.cluster = self._view.cluster_obj(self.cluster_list)
    
    def hasPhoto(self):

        """ Checks if there is map image to be displayed. """

        return not self._empty  

    def updateView(self):

        """ Update method to set new transform (aspect ratio) after zoom, pan

        Args:
            (None)
        Returns:
            (None)

        """

        self.ui.graphicsView.setTransform(QtGui.QTransform().scale(self._zoom, self._zoom).rotate(0))

    def setPhoto(self, pixmap=None):

        """ Method to set transformed map image for QGraphic widget.

        Args:
            pixmap: widgets used to display images.
        Returns:
            (None)

        """

        if pixmap and not pixmap.isNull():
            self._empty = False
            self._photo.setPixmap(pixmap)
        else:
            self._empty = True
            self._photo.setPixmap(QtGui.QPixmap())
        self.updateView()

    def wheelEvent(self, event):

        """ Method to recive mouse wheel movements.

        Args:
            event: Mouse wheel occurrence
        Returns:
            (None)

        """

        if self.hasPhoto():
            moose = event.angleDelta().y()/120
            if moose > 0:
                self.zoomIn()
            elif moose < 0:
                self.zoomOut()

    def zoomIn(self):

        """ Method to zoom-In on M-wheel-up movement."""

        self.zoom *= 1.05
        self.updateView()

    def zoomOut(self):

        """ Method to zoom-In on M-wheel-down movement."""

        self.zoom /= 1.05
        self.updateView()

    def zoomReset(self):

        """ Method to reset map image to initial scale."""

        self.zoom = 1
        self.updateView()

    def mouseEvent(self, event):

        """ Method to recive mouse move movements.

        Args:
            event: Mouse movement occurrence
        Returns:
            (None)

        """

        if self.hasPhoto:
            if self._photo.isUnderMouse() and event.buttons() == Qt.MiddleButton:
                print(self.ui.graphicsView.mapToScene(event.pos()))#.toPoint())

    # Drag function         
    def toggleDragMode(self, event):

        """ Method to pan zoomed map image bind to mouse move event.
        Args:
            event: Mouse movement occurrence
        Returns:
            (None)

        """

        if self.ui.graphicsView.dragMode() == QtWidgets.QGraphicsView.ScrollHandDrag:
            self.ui.graphicsView.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        elif not self._photo.pixmap().isNull():
            self.ui.graphicsView.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)

    def menuAnimation(self):

        """ Animation for menu button """

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

    def mousePressEvent(self, event):

        """ To obtain the X,y cordinates when mouse click """

        self.cursorPosition = event.globalPos()

    def restore_or_maximize_window(self):

        """ Maximize window function """

        if self.isMaximized():
            self.showNormal()
        else:
            self.showMaximized()    

    def pubSpeed(self, value):

        """ Publish the wheelChairSpeed value when user changes the speed

        Args:
            value: Slider value from the UI
        Returns:
            (None)

        """

        pub = rospy.Publisher('wheelChairSpeed', Int32, queue_size=10)
        rospy.loginfo(value)
        #print(value)
        pub.publish(value)

    def driveModeSelection(self):

        """ Publish wheel chair drive mode. """

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
    
    # ROS node initilization
    rospy.init_node('GUI_node', anonymous=True)
    
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())