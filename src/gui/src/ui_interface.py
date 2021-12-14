# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(746, 562)
        MainWindow.setStyleSheet("background-color: rgb(46, 52, 54);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.header_frame = QtWidgets.QFrame(self.centralwidget)
        self.header_frame.setStyleSheet("border:none;\n"
"background-color: rgb(52, 101, 164);\n"
"")
        self.header_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.header_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.header_frame.setObjectName("header_frame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.header_frame)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.header_left_frame = QtWidgets.QFrame(self.header_frame)
        self.header_left_frame.setStyleSheet("*{\n"
"    border:none;\n"
"}")
        self.header_left_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.header_left_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.header_left_frame.setObjectName("header_left_frame")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.header_left_frame)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.menu_button = QtWidgets.QPushButton(self.header_left_frame)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.menu_button.setFont(font)
        self.menu_button.setStyleSheet("color: rgb(255, 255, 255);")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icons/icons/cil-menu.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.menu_button.setIcon(icon)
        self.menu_button.setIconSize(QtCore.QSize(32, 32))
        self.menu_button.setAutoDefault(False)
        self.menu_button.setDefault(False)
        self.menu_button.setFlat(False)
        self.menu_button.setObjectName("menu_button")
        self.horizontalLayout_3.addWidget(self.menu_button, 0, QtCore.Qt.AlignLeft)
        self.horizontalLayout.addWidget(self.header_left_frame, 0, QtCore.Qt.AlignLeft)
        self.header_center_frame = QtWidgets.QFrame(self.header_frame)
        self.header_center_frame.setStyleSheet("*{\n"
"    border:none;\n"
"}")
        self.header_center_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.header_center_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.header_center_frame.setObjectName("header_center_frame")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.header_center_frame)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label = QtWidgets.QLabel(self.header_center_frame)
        font = QtGui.QFont()
        font.setFamily("Monospace")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.horizontalLayout_4.addWidget(self.label, 0, QtCore.Qt.AlignHCenter)
        self.horizontalLayout.addWidget(self.header_center_frame)
        self.header_right_frame = QtWidgets.QFrame(self.header_frame)
        self.header_right_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.header_right_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.header_right_frame.setObjectName("header_right_frame")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.header_right_frame)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setSpacing(10)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.minimize_window_button = QtWidgets.QPushButton(self.header_right_frame)
        self.minimize_window_button.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("../qt_adapt/images/icons/icon_minimize.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.minimize_window_button.setIcon(icon1)
        self.minimize_window_button.setObjectName("minimize_window_button")
        self.horizontalLayout_2.addWidget(self.minimize_window_button)
        self.resize_window_button = QtWidgets.QPushButton(self.header_right_frame)
        self.resize_window_button.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("../qt_adapt/images/icons/cil-window-restore.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.resize_window_button.setIcon(icon2)
        self.resize_window_button.setObjectName("resize_window_button")
        self.horizontalLayout_2.addWidget(self.resize_window_button)
        self.close_window_button = QtWidgets.QPushButton(self.header_right_frame)
        self.close_window_button.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("../qt_adapt/images/icons/icon_close.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.close_window_button.setIcon(icon3)
        self.close_window_button.setObjectName("close_window_button")
        self.horizontalLayout_2.addWidget(self.close_window_button)
        self.horizontalLayout.addWidget(self.header_right_frame, 0, QtCore.Qt.AlignRight)
        self.verticalLayout.addWidget(self.header_frame, 0, QtCore.Qt.AlignTop)
        self.mainBody_frame = QtWidgets.QFrame(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.mainBody_frame.sizePolicy().hasHeightForWidth())
        self.mainBody_frame.setSizePolicy(sizePolicy)
        self.mainBody_frame.setStyleSheet("*{\n"
"    border:none;\n"
"}")
        self.mainBody_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.mainBody_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mainBody_frame.setObjectName("mainBody_frame")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.mainBody_frame)
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_8.setSpacing(0)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.leftMenuList_frame = QtWidgets.QFrame(self.mainBody_frame)
        self.leftMenuList_frame.setMinimumSize(QtCore.QSize(40, 0))
        self.leftMenuList_frame.setMaximumSize(QtCore.QSize(40, 16777215))
        self.leftMenuList_frame.setStyleSheet("background-color: rgb(52, 101, 164);")
        self.leftMenuList_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.leftMenuList_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.leftMenuList_frame.setObjectName("leftMenuList_frame")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.leftMenuList_frame)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.menuList = QtWidgets.QFrame(self.leftMenuList_frame)
        self.menuList.setMinimumSize(QtCore.QSize(90, 0))
        self.menuList.setMaximumSize(QtCore.QSize(200, 16777215))
        self.menuList.setStyleSheet("*{\n"
"    border:none;\n"
"}")
        self.menuList.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.menuList.setFrameShadow(QtWidgets.QFrame.Raised)
        self.menuList.setObjectName("menuList")
        self.gridLayout = QtWidgets.QGridLayout(self.menuList)
        self.gridLayout.setContentsMargins(0, 0, 5, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.Battery = QtWidgets.QLabel(self.menuList)
        self.Battery.setObjectName("Battery")
        self.gridLayout.addWidget(self.Battery, 1, 1, 1, 1, QtCore.Qt.AlignLeft)
        self.Navigation = QtWidgets.QLabel(self.menuList)
        self.Navigation.setObjectName("Navigation")
        self.gridLayout.addWidget(self.Navigation, 0, 1, 1, 1, QtCore.Qt.AlignLeft)
        self.navigation_button = QtWidgets.QPushButton(self.menuList)
        self.navigation_button.setText("")
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(":/icons/icons/cil-map.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.navigation_button.setIcon(icon4)
        self.navigation_button.setIconSize(QtCore.QSize(42, 42))
        self.navigation_button.setObjectName("navigation_button")
        self.gridLayout.addWidget(self.navigation_button, 0, 0, 1, 1)
        self.Speed = QtWidgets.QLabel(self.menuList)
        self.Speed.setObjectName("Speed")
        self.gridLayout.addWidget(self.Speed, 2, 1, 1, 1, QtCore.Qt.AlignLeft)
        self.Mode = QtWidgets.QLabel(self.menuList)
        self.Mode.setObjectName("Mode")
        self.gridLayout.addWidget(self.Mode, 3, 1, 1, 1, QtCore.Qt.AlignLeft)
        self.Statistics = QtWidgets.QLabel(self.menuList)
        self.Statistics.setObjectName("Statistics")
        self.gridLayout.addWidget(self.Statistics, 4, 1, 1, 1, QtCore.Qt.AlignLeft)
        self.battery_button = QtWidgets.QPushButton(self.menuList)
        self.battery_button.setText("")
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap(":/icons/icons/cil-battery-5.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.battery_button.setIcon(icon5)
        self.battery_button.setIconSize(QtCore.QSize(42, 42))
        self.battery_button.setObjectName("battery_button")
        self.gridLayout.addWidget(self.battery_button, 1, 0, 1, 1)
        self.speedSel_button = QtWidgets.QPushButton(self.menuList)
        self.speedSel_button.setText("")
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap(":/icons/icons/cil-speedometer.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.speedSel_button.setIcon(icon6)
        self.speedSel_button.setIconSize(QtCore.QSize(42, 42))
        self.speedSel_button.setObjectName("speedSel_button")
        self.gridLayout.addWidget(self.speedSel_button, 2, 0, 1, 1)
        self.modeSel_button = QtWidgets.QPushButton(self.menuList)
        self.modeSel_button.setText("")
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap(":/icons/icons/cil-settings.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.modeSel_button.setIcon(icon7)
        self.modeSel_button.setIconSize(QtCore.QSize(42, 42))
        self.modeSel_button.setObjectName("modeSel_button")
        self.gridLayout.addWidget(self.modeSel_button, 3, 0, 1, 1)
        self.statistics_button = QtWidgets.QPushButton(self.menuList)
        self.statistics_button.setText("")
        icon8 = QtGui.QIcon()
        icon8.addPixmap(QtGui.QPixmap(":/icons/icons/cil-chart.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.statistics_button.setIcon(icon8)
        self.statistics_button.setIconSize(QtCore.QSize(42, 42))
        self.statistics_button.setObjectName("statistics_button")
        self.gridLayout.addWidget(self.statistics_button, 4, 0, 1, 1)
        self.verticalLayout_2.addWidget(self.menuList, 0, QtCore.Qt.AlignLeft|QtCore.Qt.AlignTop)
        self.horizontalLayout_8.addWidget(self.leftMenuList_frame)
        self.display_area = QtWidgets.QFrame(self.mainBody_frame)
        self.display_area.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.display_area.setFrameShadow(QtWidgets.QFrame.Raised)
        self.display_area.setObjectName("display_area")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.display_area)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.stackedWidget = QtWidgets.QStackedWidget(self.display_area)
        self.stackedWidget.setObjectName("stackedWidget")
        self.navigation_widget = QtWidgets.QWidget()
        self.navigation_widget.setObjectName("navigation_widget")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.navigation_widget)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.display_map = QtWidgets.QOpenGLWidget(self.navigation_widget)
        self.display_map.setStyleSheet("background-color: rgb(0, 0, 0);")
        self.display_map.setObjectName("display_map")
        self.verticalLayout_8.addWidget(self.display_map)
        self.stackedWidget.addWidget(self.navigation_widget)
        self.battery_widget = QtWidgets.QWidget()
        self.battery_widget.setObjectName("battery_widget")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.battery_widget)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.BatterInformation = QtWidgets.QLabel(self.battery_widget)
        self.BatterInformation.setObjectName("BatterInformation")
        self.verticalLayout_4.addWidget(self.BatterInformation, 0, QtCore.Qt.AlignBottom)
        self.frame = QtWidgets.QFrame(self.battery_widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame.sizePolicy().hasHeightForWidth())
        self.frame.setSizePolicy(sizePolicy)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.frame)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.batterStatus = QtWidgets.QLabel(self.frame)
        self.batterStatus.setObjectName("batterStatus")
        self.gridLayout_2.addWidget(self.batterStatus, 0, 0, 1, 1)
        self.display_batterStatus = QtWidgets.QLabel(self.frame)
        self.display_batterStatus.setObjectName("display_batterStatus")
        self.gridLayout_2.addWidget(self.display_batterStatus, 0, 1, 1, 1)
        self.batteryCharge = QtWidgets.QLabel(self.frame)
        self.batteryCharge.setObjectName("batteryCharge")
        self.gridLayout_2.addWidget(self.batteryCharge, 1, 0, 1, 1)
        self.display_batteryCharge = QtWidgets.QLabel(self.frame)
        self.display_batteryCharge.setObjectName("display_batteryCharge")
        self.gridLayout_2.addWidget(self.display_batteryCharge, 1, 1, 1, 1)
        self.batteryTimeLeft = QtWidgets.QLabel(self.frame)
        self.batteryTimeLeft.setObjectName("batteryTimeLeft")
        self.gridLayout_2.addWidget(self.batteryTimeLeft, 2, 0, 1, 1)
        self.battery_TimeLeft = QtWidgets.QLabel(self.frame)
        self.battery_TimeLeft.setObjectName("battery_TimeLeft")
        self.gridLayout_2.addWidget(self.battery_TimeLeft, 2, 1, 1, 1)
        self.batteryPluggedIn = QtWidgets.QLabel(self.frame)
        self.batteryPluggedIn.setObjectName("batteryPluggedIn")
        self.gridLayout_2.addWidget(self.batteryPluggedIn, 3, 0, 1, 1)
        self.battery_pluggedIn = QtWidgets.QLabel(self.frame)
        self.battery_pluggedIn.setObjectName("battery_pluggedIn")
        self.gridLayout_2.addWidget(self.battery_pluggedIn, 3, 1, 1, 1)
        self.verticalLayout_4.addWidget(self.frame, 0, QtCore.Qt.AlignTop)
        self.stackedWidget.addWidget(self.battery_widget)
        self.speedSel_widget = QtWidgets.QWidget()
        self.speedSel_widget.setObjectName("speedSel_widget")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.speedSel_widget)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.wheelChairSpeed = QtWidgets.QFrame(self.speedSel_widget)
        self.wheelChairSpeed.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.wheelChairSpeed.setFrameShadow(QtWidgets.QFrame.Raised)
        self.wheelChairSpeed.setObjectName("wheelChairSpeed")
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout(self.wheelChairSpeed)
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_2 = QtWidgets.QLabel(self.wheelChairSpeed)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_11.addWidget(self.label_2, 0, QtCore.Qt.AlignBottom)
        self.verticalLayout_5.addWidget(self.wheelChairSpeed, 0, QtCore.Qt.AlignBottom)
        self.slider_frame = QtWidgets.QFrame(self.speedSel_widget)
        self.slider_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.slider_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.slider_frame.setObjectName("slider_frame")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.slider_frame)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.horizontalSlider = QtWidgets.QSlider(self.slider_frame)
        self.horizontalSlider.setStyleSheet("background-color: rgb(46, 52, 54);")
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.horizontalLayout_9.addWidget(self.horizontalSlider)
        self.verticalLayout_5.addWidget(self.slider_frame)
        self.speedDisplay_frame = QtWidgets.QFrame(self.speedSel_widget)
        self.speedDisplay_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.speedDisplay_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.speedDisplay_frame.setObjectName("speedDisplay_frame")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout(self.speedDisplay_frame)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.lowSpeed = QtWidgets.QLabel(self.speedDisplay_frame)
        self.lowSpeed.setObjectName("lowSpeed")
        self.horizontalLayout_10.addWidget(self.lowSpeed, 0, QtCore.Qt.AlignVCenter)
        self.mediumSpeed = QtWidgets.QLabel(self.speedDisplay_frame)
        self.mediumSpeed.setObjectName("mediumSpeed")
        self.horizontalLayout_10.addWidget(self.mediumSpeed, 0, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        self.highSpeed = QtWidgets.QLabel(self.speedDisplay_frame)
        self.highSpeed.setObjectName("highSpeed")
        self.horizontalLayout_10.addWidget(self.highSpeed, 0, QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter)
        self.verticalLayout_5.addWidget(self.speedDisplay_frame, 0, QtCore.Qt.AlignTop)
        self.stackedWidget.addWidget(self.speedSel_widget)
        self.modeSel_widget = QtWidgets.QWidget()
        self.modeSel_widget.setObjectName("modeSel_widget")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.modeSel_widget)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.frame_3 = QtWidgets.QFrame(self.modeSel_widget)
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout(self.frame_3)
        self.horizontalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_12.setSpacing(9)
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.wheelChairDriveMode = QtWidgets.QLabel(self.frame_3)
        self.wheelChairDriveMode.setObjectName("wheelChairDriveMode")
        self.horizontalLayout_12.addWidget(self.wheelChairDriveMode, 0, QtCore.Qt.AlignBottom)
        self.verticalLayout_6.addWidget(self.frame_3)
        self.modeSelPB_frame = QtWidgets.QFrame(self.modeSel_widget)
        self.modeSelPB_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.modeSelPB_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.modeSelPB_frame.setObjectName("modeSelPB_frame")
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout(self.modeSelPB_frame)
        self.horizontalLayout_13.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_13.setSpacing(10)
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.autoMode = QtWidgets.QPushButton(self.modeSelPB_frame)
        self.autoMode.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: rgb(136, 138, 133);")
        self.autoMode.setIconSize(QtCore.QSize(32, 32))
        self.autoMode.setDefault(False)
        self.autoMode.setFlat(False)
        self.autoMode.setObjectName("autoMode")
        self.horizontalLayout_13.addWidget(self.autoMode)
        self.joystickMode = QtWidgets.QPushButton(self.modeSelPB_frame)
        self.joystickMode.setStyleSheet("color: rgb(238, 238, 236);\n"
"background-color: rgb(136, 138, 133);")
        self.joystickMode.setObjectName("joystickMode")
        self.horizontalLayout_13.addWidget(self.joystickMode)
        self.verticalLayout_6.addWidget(self.modeSelPB_frame, 0, QtCore.Qt.AlignVCenter)
        self.stackedWidget.addWidget(self.modeSel_widget)
        self.statistics_widget = QtWidgets.QWidget()
        self.statistics_widget.setObjectName("statistics_widget")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.statistics_widget)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.Statistics_2 = QtWidgets.QLabel(self.statistics_widget)
        self.Statistics_2.setObjectName("Statistics_2")
        self.verticalLayout_7.addWidget(self.Statistics_2, 0, QtCore.Qt.AlignBottom)
        self.stat_frame = QtWidgets.QFrame(self.statistics_widget)
        self.stat_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.stat_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.stat_frame.setObjectName("stat_frame")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.stat_frame)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.uptime = QtWidgets.QLabel(self.stat_frame)
        self.uptime.setObjectName("uptime")
        self.gridLayout_3.addWidget(self.uptime, 0, 0, 1, 1)
        self.display_Uptime = QtWidgets.QLabel(self.stat_frame)
        self.display_Uptime.setObjectName("display_Uptime")
        self.gridLayout_3.addWidget(self.display_Uptime, 0, 1, 1, 1)
        self.distanceDriven = QtWidgets.QLabel(self.stat_frame)
        self.distanceDriven.setObjectName("distanceDriven")
        self.gridLayout_3.addWidget(self.distanceDriven, 1, 0, 1, 1)
        self.display_distanceDriven = QtWidgets.QLabel(self.stat_frame)
        self.display_distanceDriven.setObjectName("display_distanceDriven")
        self.gridLayout_3.addWidget(self.display_distanceDriven, 1, 1, 1, 1)
        self.network = QtWidgets.QLabel(self.stat_frame)
        self.network.setObjectName("network")
        self.gridLayout_3.addWidget(self.network, 2, 0, 1, 1)
        self.display_networkStatus = QtWidgets.QLabel(self.stat_frame)
        self.display_networkStatus.setObjectName("display_networkStatus")
        self.gridLayout_3.addWidget(self.display_networkStatus, 2, 1, 1, 1)
        self.version = QtWidgets.QLabel(self.stat_frame)
        self.version.setObjectName("version")
        self.gridLayout_3.addWidget(self.version, 3, 0, 1, 1)
        self.display_version = QtWidgets.QLabel(self.stat_frame)
        self.display_version.setObjectName("display_version")
        self.gridLayout_3.addWidget(self.display_version, 3, 1, 1, 1)
        self.verticalLayout_7.addWidget(self.stat_frame, 0, QtCore.Qt.AlignTop)
        self.stackedWidget.addWidget(self.statistics_widget)
        self.verticalLayout_3.addWidget(self.stackedWidget)
        self.horizontalLayout_8.addWidget(self.display_area)
        self.verticalLayout.addWidget(self.mainBody_frame)
        self.footer_frame = QtWidgets.QFrame(self.centralwidget)
        self.footer_frame.setStyleSheet("*{\n"
"    border:none;\n"
"}")
        self.footer_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.footer_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.footer_frame.setObjectName("footer_frame")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.footer_frame)
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_5.setSpacing(0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.footer_copyrights = QtWidgets.QFrame(self.footer_frame)
        self.footer_copyrights.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.footer_copyrights.setFrameShadow(QtWidgets.QFrame.Raised)
        self.footer_copyrights.setObjectName("footer_copyrights")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.footer_copyrights)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.copyrights = QtWidgets.QLabel(self.footer_copyrights)
        self.copyrights.setObjectName("copyrights")
        self.horizontalLayout_7.addWidget(self.copyrights)
        self.horizontalLayout_5.addWidget(self.footer_copyrights)
        self.footer_help_frame = QtWidgets.QFrame(self.footer_frame)
        self.footer_help_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.footer_help_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.footer_help_frame.setObjectName("footer_help_frame")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.footer_help_frame)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.footer_helpButton = QtWidgets.QPushButton(self.footer_help_frame)
        self.footer_helpButton.setStyleSheet("color: rgb(255, 255, 255);")
        self.footer_helpButton.setObjectName("footer_helpButton")
        self.horizontalLayout_6.addWidget(self.footer_helpButton, 0, QtCore.Qt.AlignRight)
        self.horizontalLayout_5.addWidget(self.footer_help_frame, 0, QtCore.Qt.AlignRight)
        self.window_resize = QtWidgets.QFrame(self.footer_frame)
        self.window_resize.setMinimumSize(QtCore.QSize(10, 10))
        self.window_resize.setMaximumSize(QtCore.QSize(10, 10))
        self.window_resize.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.window_resize.setFrameShadow(QtWidgets.QFrame.Raised)
        self.window_resize.setObjectName("window_resize")
        self.horizontalLayout_5.addWidget(self.window_resize, 0, QtCore.Qt.AlignBottom)
        self.verticalLayout.addWidget(self.footer_frame, 0, QtCore.Qt.AlignBottom)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        self.stackedWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.menu_button.setText(_translate("MainWindow", "Menu"))
        self.label.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#ffffff;\">ADAPT</span></p></body></html>"))
        self.Battery.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">Battery</span></p></body></html>"))
        self.Navigation.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">Navigation</span></p></body></html>"))
        self.Speed.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">Speed</span></p></body></html>"))
        self.Mode.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">Mode</span></p></body></html>"))
        self.Statistics.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">Statistics</span></p></body></html>"))
        self.BatterInformation.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#ffffff;\">Battery Information :</span></p></body></html>"))
        self.batterStatus.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">Status</span></p></body></html>"))
        self.display_batterStatus.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">N/A</span></p></body></html>"))
        self.batteryCharge.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">Charge</span></p></body></html>"))
        self.display_batteryCharge.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">N/A</span></p></body></html>"))
        self.batteryTimeLeft.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">Time Left</span></p></body></html>"))
        self.battery_TimeLeft.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">N/A</span></p></body></html>"))
        self.batteryPluggedIn.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">Plugged In</span></p></body></html>"))
        self.battery_pluggedIn.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; color:#ffffff;\">N/A</span></p></body></html>"))
        self.label_2.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#ffffff;\">Wheelchair Speed :</span></p></body></html>"))
        self.lowSpeed.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#ffffff;\">Low speed</span></p></body></html>"))
        self.mediumSpeed.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#ffffff;\">Medium Speed</span></p></body></html>"))
        self.highSpeed.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#ffffff;\">High speed</span></p></body></html>"))
        self.wheelChairDriveMode.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#ffffff;\">Wheelchair Drive Mode :</span></p></body></html>"))
        self.autoMode.setText(_translate("MainWindow", "Auto mode"))
        self.joystickMode.setText(_translate("MainWindow", "JoyStick mode"))
        self.Statistics_2.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#ffffff;\">Statistics :</span></p></body></html>"))
        self.uptime.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#ffffff;\">Uptime</span></p></body></html>"))
        self.display_Uptime.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#ffffff;\">N/A</span></p></body></html>"))
        self.distanceDriven.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#ffffff;\">Distance driven</span></p></body></html>"))
        self.display_distanceDriven.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#ffffff;\">N/A</span></p></body></html>"))
        self.network.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#eeeeec;\">Network</span></p></body></html>"))
        self.display_networkStatus.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#ffffff;\">N/A</span></p></body></html>"))
        self.version.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#ffffff;\">Version</span></p></body></html>"))
        self.display_version.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#ffffff;\">N/A</span></p></body></html>"))
        self.copyrights.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" color:#ffffff;\">INTEREG</span></p></body></html>"))
        self.footer_helpButton.setText(_translate("MainWindow", "?"))
import images_rc
