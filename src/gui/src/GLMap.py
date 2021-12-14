from python_qt_binding import QtCore      # core Qt functionality
from python_qt_binding import QtGui       # extends QtCore with GUI functionality
from python_qt_binding import QtOpenGL    # provides QGLWidget, a special OpenGL QWidget
from python_qt_binding.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QWidget

import OpenGL.GL as gl        # python wrapping of OpenGL
from OpenGL import GLU        # OpenGL Utility Library, extends OpenGL functionality

import sys                    # we'll need this later to run our Qt application

from OpenGL.arrays import vbo
import numpy as np

class GroundGraphics(object):
    """
    This class defines a grid (triangular mesh) representing the ground plane. The render
    function must be called from the main paintGL rendering function.

    """

    def __init__(self, length, width):
        """ Initialize the ground graphics object.

        Initialize the ground graphics object.

        Args:
            length (float): Length of the ground grid, in meters.
            width (float): Width of the ground grid, in meters.

        Returns:
            (None)

        """
        length = 20.0
        width = 20.0

        # Store the grid dimensions and compute number of squares and vertices
        self.len = length
        self.w = width
        self.res = 10
        self.n_sq = self.res**2
        self.n_vert = 6 * self.n_sq

        # Define the vertex (x,y,z) values as a grid:
        self.vx = np.linspace(-0.5*self.len, 0.5*self.len, self.res + 1)
        self.vy = np.linspace(-0.5*self.w, 0.5*self.w, self.res + 1)
        self.vz = np.zeros((self.res + 1, self.res + 1))

        self.vert = np.zeros((self.n_vert, 3))

        # Organize the vertices into triangles for storing in a VBO:
        sq_ind = 0
        for i in range(self.res):
            for j in range(self.res):
                # Upper triangle in square:
                self.vert[6 * sq_ind,:] = np.array([self.vx[i], self.vy[j], self.vz[i, j]])
                self.vert[6 * sq_ind + 1,:] = np.array([self.vx[i+1], self.vy[j+1], self.vz[i+1, j+1]])
                self.vert[6 * sq_ind + 2,:] = np.array([self.vx[i], self.vy[j+1], self.vz[i, j+1]])

                # Lower triangle in square:
                self.vert[6 * sq_ind + 3,:] = np.array([self.vx[i], self.vy[j], self.vz[i, j]])
                self.vert[6 * sq_ind + 4,:] = np.array([self.vx[i+1], self.vy[j], self.vz[i+1, j]])
                self.vert[6 * sq_ind + 5,:] = np.array([self.vx[i+1], self.vy[j+1], self.vz[i+1, j+1]])

                sq_ind += 1

        # Pack the triangle vertices into a dedicated VBO:
        self.vert_stride = 12 # number of bytes between successive triangles
        self.vert_vbo = vbo.VBO(np.reshape(self.vert, (1,-1), order='C').astype(np.float32))

    def render(self):
        """ Renders the ground plane graphics object.

        Render the ground plane graphics using the geometry defined in the constructor.
        This function must be called from the main paintGL rendering function.

        Args:
            (None)

        Returns:
            (None)

        """


        gl.glPushMatrix()

        try:
            # Bind the vertex data buffer to the VBO all future rendering
            # (or until unbound with 'unbind'):
            self.vert_vbo.bind()

            # Set the vertex pointer for rendering:
            gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
            gl.glVertexPointer(3, gl.GL_FLOAT, self.vert_stride, self.vert_vbo)

            # Set the polygons to have front and back faces and to not be filled:
            gl.glColor3f(1.0, 1.0, 1.0)
            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)

            # Render triangle edges using the loaded vertex pointer data:
            gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.n_vert)

            # Set the polygons to have front and back faces and to not be filled:
            gl.glColor3f(0.5, 0.5, 0.5)
            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)

            # Render triangle faces using the loaded vertex pointer data:
            gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.n_vert)

        except Exception as e:
            print(e)

        finally:
            self.vert_vbo.unbind()
            gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
            gl.glPopMatrix()


class GLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        self.parent = parent
        QtOpenGL.QGLWidget.__init__(self, parent)
            
    def initializeGL(self):
        """ Initializes OpenGL functionality and geometry.

        Virtual function provided by QGLWidget, called once at the beginning of application.
        OpenGL and geometry initialization is performed here.

        Args:
            (None)

        Returns:
            (None)

        """
        # Initialize the camera state and set the initial view
        self.eye_r = 20.0     # camera range, in meters
        self.eye_th = 1.0     # camera azimuth angle, in radians
        self.eye_phi = 1.0    # camera elevation angle, in radians
        self.center_pos = np.array([0.0, 0.0, 0.0])
        self.update_view()

        # Set focus to the window
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # Convenience function, calls glClearColor under the hood.
        # QColor is specified as RGB ints (0-255).  Specify this clear
        # color once and call glClear(GL_COLOR_BUFFER_BIT) before each
        # round of rendering (in paintGL):
        self.qglClearColor(QtGui.QColor(100, 100, 100)) # a grey background
                
        # Enable the depth buffer:
        gl.glEnable(gl.GL_DEPTH_TEST)

        # Initialize the ground plane graphics geometry
        self.ground_graphics = GroundGraphics(length=10.0, width=10.0)
         
    def resizeGL(self, width, height):
        gl.glViewport(0, 0, width, height)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = width / float(height)

        GLU.gluPerspective(45.0, aspect, 1.0, 100.0)
        gl.glMatrixMode(gl.GL_MODELVIEW)

    def paintGL(self):
        """ Defines behavior of OpenGL window when resized.

        Virtual function provided by QGLWidget, called from QGLWidget method updateGL.
        All user rendering code should be defined here.

        Args:
            (None)

        Returns:
            (None)

        """
        
        # Start from a blank slate each render by clearing buffers
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        
        GroundGraphics.render()

    def update_view(self):
        """ Updates the camera view using current camera state.

        Function to be called after updating any camera state variable in order to update
        the camera view. Converts spherical camera coordinates to a Cartesian position for
        the eye of the camera, with the center position (focal point) fixed at the origin.

        Args:
            (None)

        Returns:
            (None)

        """

        self.eye_pos = np.array([self.eye_r*np.sin(self.eye_phi)*np.cos(self.eye_th),
                                 self.eye_r*np.sin(self.eye_phi)*np.sin(self.eye_th),
                                 self.eye_r*np.cos(self.eye_phi)])
        up_vec = np.array([0.0, 0.0, 1.0])
        gl.glLoadIdentity()
        GLU.gluLookAt(*np.concatenate((self.eye_pos, self.center_pos, up_vec)))    

    def keyPressEvent(self, event):
        """ Defines callbacks for keypress events.

        Implement override for virtual function provided by Qt base class for defining
        keypress event callbacks, for example manipulating the primary view camera.

        Args:
            event (QKeyEvent): Screen width in pixels.
            height (int): Screen height in pixels.

        Returns:
            (None)

        """
        
        if type(event) == QtGui.QKeyEvent:
            if event.key() == QtCore.Qt.Key_W:
                # Hold W to decrease range (zoom in)
                self.eye_r -= 0.5
                self.update_view()
                
            elif event.key() == QtCore.Qt.Key_S:
                # Hold S to increase range (zoom out)
                self.eye_r += 0.5
                self.update_view()

            elif event.key() == QtCore.Qt.Key_Down:
                # Hold DOWNARROW to increase elevation angle 
                self.eye_phi += 0.05
                self.update_view()

            elif event.key() == QtCore.Qt.Key_Up:
                # Hold UPARROW to decrease elevation angle
                self.eye_phi -= 0.05
                self.update_view()

            elif event.key() == QtCore.Qt.Key_Right:
                # Hold RIGHTARROW to increase azimuth angle
                self.eye_th += 0.05
                self.update_view()

            elif event.key() == QtCore.Qt.Key_Left:
                # Hold LEFTARROW to decrease azimuth angle
                self.eye_th -= 0.05
                self.update_view()


class MainWindow(QMainWindow):

    def __init__(self):
        QMainWindow.__init__(self)    # call the init for the parent class
        
        self.resize(800, 700)
        self.setWindowTitle('Hello OpenGL App')

        self.glWidget = GLWidget(self)
        self.initGUI()
        
        timer = QtCore.QTimer(self)
        timer.setInterval(20)   # period, in milliseconds
        timer.timeout.connect(self.glWidget.updateGL)
        timer.start()
        
    def initGUI(self):
        central_widget = QWidget()
        gui_layout = QVBoxLayout()
        central_widget.setLayout(gui_layout)

        self.setCentralWidget(central_widget)

        gui_layout.addWidget(self.glWidget)

        """
        sliderX = QSlider(QtCore.Qt.Horizontal)
        sliderX.valueChanged.connect(lambda val: self.glWidget.setRotX(val))

        sliderY = QSlider(QtCore.Qt.Horizontal)
        sliderY.valueChanged.connect(lambda val: self.glWidget.setRotY(val))

        sliderZ = QSlider(QtCore.Qt.Horizontal)
        sliderZ.valueChanged.connect(lambda val: self.glWidget.setRotZ(val))
        
        gui_layout.addWidget(sliderX)
        gui_layout.addWidget(sliderY)
        gui_layout.addWidget(sliderZ)
        
        """
        
if __name__ == '__main__':

    app = QApplication(sys.argv)
    
    win = MainWindow()
    win.show()

    sys.exit(app.exec_())
    
