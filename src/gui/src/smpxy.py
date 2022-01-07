from python_qt_binding import QtGui
from python_qt_binding.QtCore import QEvent
from python_qt_binding.QtWidgets import QGraphicsView


class MyFrame(QGraphicsView):

    def __init__(self, parent=None):
        """ Initialize the Qt OpenGL Widget.
        Initialize the Qt OpenGL widget.
        Args:
            (None)
        Returns:
            (None)
        """
        QGraphicsView.__init__(self)
        self.setMouseTracking(True)

    def mouseMoveEvent(self, event):
        self.cursor_position = event.scenePos()
        self.update()
        super(MyFrame, self).mouseMoveEvent(event)

    def wheelEvent( self, event ):
        """
            PyQT4 WheelEvent? how to detect if the wheel have been use?
            https://stackoverflow.com/questions/9475772/pyqt4-wheelevent-how-to-detect-if-the-wheel-have-been-use

            QGraphicsView Zooming in and out under mouse position using mouse wheel
            https://stackoverflow.com/questions/19113532/qgraphicsview-zooming-in-and-out-under-mouse-position-using-mouse-wheel
        """

        # Zoom Factor
        zoomInFactor = 1.1
        zoomOutFactor = 1 / zoomInFactor

        # Set Anchors
        self.setTransformationAnchor( QGraphicsView.NoAnchor )
        self.setResizeAnchor( QGraphicsView.NoAnchor )

        # Save the scene pos
        oldPos = self.mapToScene( event.pos() )

        # Zoom
        if event.delta() > 0:
            zoomFactor = zoomInFactor
        else:
            zoomFactor = zoomOutFactor
        self.scale( zoomFactor, zoomFactor )

        # Get the new position
        newPos = self.mapToScene( event.pos() )

        # Move scene to old position
        delta = newPos - oldPos
        self.translate( delta.x(), delta.y() )

"""if ( __name__ == '__main__' ):
    app = QApplication( [] )
    f = MyFrame()
    f.show()
    app.exec_()
    """