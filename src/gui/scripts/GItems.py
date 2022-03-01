#!/usr/bin/env python

from python_qt_binding.QtWidgets import QGraphicsPixmapItem, QGraphicsScene
from python_qt_binding import QtWidgets, QtGui, QtCore
from python_qt_binding.QtCore import Qt, QRectF, QLineF, QPointF
from python_qt_binding.QtGui import QImage, QPixmap, QPen
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsItem, QGraphicsItemGroup, QGraphicsLineItem


class Graphicsscene(object):
    def __init__(self):
        object.__init__(self)
        #self.setSceneRect(-100, -100, 200, 200)     

    def addAxisItem(self):
        pencilX = QPen( Qt.red, 1 )
        pencilY = QPen( Qt.green, 1 )
        pencilX.setStyle( Qt.SolidLine )
        pencilY.setStyle( Qt.SolidLine )

        xAxis = QGraphicsLineItem(0, 0, 50, 0)
        yAxis = QGraphicsLineItem(0, -50, 0, 0)

        xAxis.setPen(pencilX)
        yAxis.setPen(pencilY)

        axisGrp = QGraphicsItemGroup()
        axisGrp.setFlag( QGraphicsItem.ItemIsMovable )

        axisGrp.addToGroup(xAxis)
        axisGrp.addToGroup(yAxis)

        return axisGrp

    def addRobot(self):
        ellipse = QtWidgets.QGraphicsEllipseItem(0, 0, 8, 8)
        ellipse.setPos(190.0, 190.0)

        brush = QtGui.QBrush(Qt.GlobalColor.blue)
        ellipse.setBrush(brush)

        pen = QPen(Qt.GlobalColor.green)
        pen.setWidth(5)
        ellipse.setPen(pen)

        return ellipse

    def cluster_obj(self, cluster_list):

        elipse_list = []

        """ for cluster in cluster_list:

            elipse = QtWidgets.QGraphicsEllipseItem(0, 0, 2, 2)
            elipse.setPos(cluster[0], cluster[1])  
            brush = QtGui.QBrush(Qt.GlobalColor.blue)
            elipse.setBrush(brush)
            pen = QPen(Qt.GlobalColor.green)
            pen.setWidth(5)
            elipse.setPen(pen)
            elipse_list.append(elipse)
            print('Cluster in pixel ; '+str(cluster[0])+ ' '+ str(cluster[1])) """

        elipse = QtWidgets.QGraphicsEllipseItem(0, 0, 2, 2)
        elipse.setPos(cluster_list[0], cluster_list[1])  
        brush = QtGui.QBrush(Qt.GlobalColor.blue)
        elipse.setBrush(brush)
        pen = QPen(Qt.GlobalColor.green)
        pen.setWidth(5)
        elipse.setPen(pen)
        elipse_list.append(elipse)
        #print('Cluster in pixel ; '+str(cluster_list[0])+ ' '+ str(cluster_list[1]))        
        
        return elipse_list
        
    

"""
def fitInView(self, scale=True):
        rect = QtCore.QRectF(self._photo.pixmap().rect())
        if not rect.isNull():
            self.ui.graphicsView.setSceneRect(rect)
            print(rect)
            if self.hasPhoto():
                unity = self.ui.graphicsView.transform().mapRect(QtCore.QRectF(0, 0, 1, 1))
                self.ui.graphicsView.scale(1 / unity.width(), 1 / unity.height()) # Pass in
                viewrect = self.ui.graphicsView.viewport().rect()
                scenerect = self.ui.graphicsView.transform().mapRect(rect)
                factor = min(viewrect.width() / scenerect.width(),
                             viewrect.height() / scenerect.height())
                self.ui.graphicsView.scale(factor, factor)
            self._zoom = 0
        #self.ui.graphicsView.fitInView(self._scene.sceneRect(), QtCore.Qt.KeepAspectRatio)
"""    