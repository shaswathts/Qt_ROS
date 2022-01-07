from python_qt_binding import QtCore, QtGui, QtWidgets


class GraphicsScene(QtWidgets.QGraphicsScene):
    def drawForeground(self, painter, rect):
        super(GraphicsScene, self).drawForeground(painter, rect)
        if not hasattr(self, "cursor_position"):
            return
        painter.save()
        pen = QtGui.QPen(QtGui.QColor("yellow"))
        pen.setWidth(4)
        painter.setPen(pen)
        linex = QtCore.QLineF(
            rect.left(),
            self.cursor_position.y(),
            rect.right(),
            self.cursor_position.y(),
        )
        liney = QtCore.QLineF(
            self.cursor_position.x(),
            rect.top(),
            self.cursor_position.x(),
            rect.bottom(),
        )
        for line in (linex, liney):
            painter.drawLine(line)
        painter.restore()

    def mouseMoveEvent(self, event):
        self.cursor_position = event.scenePos()
        self.update()
        super(GraphicsScene, self).mouseMoveEvent(event)


class GraphicsView(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
        super(GraphicsView, self).__init__(parent)
        self.setMouseTracking(True)
        scene = GraphicsScene(QtCore.QRectF(-200, -200, 400, 400), self)
        self.setScene(scene)


if __name__ == "__main__":
    import sys
    import random

    app = QtWidgets.QApplication(sys.argv)

    w = GraphicsView()

    for _ in range(4):
        r = QtCore.QRectF(
            *random.sample(range(-200, 200), 2),
            *random.sample(range(50, 150), 2)
        )
        it = w.scene().addRect(r)
        it.setBrush(QtGui.QColor(*random.sample(range(255), 3)))

    w.resize(640, 480)
    w.show()

    sys.exit(app.exec_())
