#!/usr/bin/env python

import rospy

import warnings
from collections import deque
import numpy as np
import cv2


class UpdateTransformation:

    def __init__(self, map=None, transformation=[0, 0, 0], transform=False, pre_xy=[]):
        
        if map is not None:
            mapImage = map
        else:
            mapImage = cv2.imread("/home/intern/adapt_Pyrqt/src/smp_gui/src/second_map.pgm", cv2.IMREAD_UNCHANGED)
            warnings.warn("Map read from source !!")

        #self.buffer = deque([], 10)
        #self.buffer.append(mapImage)
        #self.image = self.buffer.popleft
        
        self._trans = opencv(mapImage)
        self.transformation = transformation
        self._pos = transform
        self.pre_xy = pre_xy
        

    def return_displacement(self):

        """ Method to get transformed OpenCV image if transform is False
        else just get the displacement for a given coordinates

        Args:
            (None)
        Returns:
           _img: Transformed OpenCV image
           _shift: Transformed OpenCV image for cluster points

        """

        if self._pos:
            _img = self._trans.transform(self._transformation)
            return _img
        else:
            _shift = self._trans.get_displacement(self.pre_xy, self._transformation)
            return _shift

    # Getter method
    @property
    def transformation(self):

        """ Method to get transformed OpenCV image

        Args:
            (None)
        Returns:
           openCV_image: Transformed OpenCV image

        """

        openCV_image = self.return_displacement()
        return openCV_image 

    # Setter method
    @transformation.setter
    def transformation(self, value):

        """ Method to set new transforms

        Args:
            value: List of [x, y, theta] coordinates
        Returns:
           (None)

        """

        self._transformation = value


class opencv():

    """ This class defines the map image manipulation in OpenCV. """

    def __init__(self, image=None):# Load the image
        # cv2.IMREAD_COLOR = 1
        # cv2.IMREAD_GRAYSCALE = 0
        # cv2.IMREAD_UNCHANGED = -1 
        self.image = image #cv2.imread("/home/intern/adapt_Pyrqt/src/smp_gui/src/second_map.pgm", cv2.IMREAD_UNCHANGED)

    def set(self, rotation, translation):
 
        """ Method to create a Homgeneous matrix

        Args:
            rotation: 2 x 2 matrix
            translation: 2 x 1 matrix
        Returns:
           mat: 3 x 3 matrix

        """

        mat = np.block([
                        [rotation, translation.reshape(2,1)],
                        [np.zeros((1,2)), 1.0]
                    ])

        return mat

    # Drawing clusters 
    def draw(self, img, cluster_list):

        """ Draw marker for cluster position on the transformed image. 

        Args:
            img: Transformed OpenCV image
            robot_pos: List of [x, y] coordinates
        Returns:
           rob: OpenCV image with marker for cluster position

        """

        for cluster in cluster_list:
            rob = cv2.drawMarker(img , (int(cluster[0]), int(cluster[1])), (0, 255, 0), cv2.MARKER_DIAMOND,
                                        markerSize=10, thickness=3, line_type=cv2.LINE_AA)

        return rob

    # This will always be the center of the map to get the perspective view  
    def draw_rob(self, img):

        """ Draw marker for wheelchair position on the transformed image. 

        Args:
            img: Transformed OpenCV image
            robot_pos: List of [x, y] coordinates
        Returns:
           rob: OpenCV image with marker for wheelchair position

        """

        rows,cols = img.shape
        rob = cv2.drawMarker(img , (int(rows/2), int(cols/2)), (0, 255, 0), cv2.MARKER_TRIANGLE_UP,
                                        markerSize=10, thickness=2, line_type=cv2.LINE_AA)
        return rob

    def translate(self, img, robot_pos):

        """ Translate a given map image to a new (x,y) position.

        Args:
            img: OpenCV image
            robot_pos: List of [x, y] coordinates
        Returns:
           rob: Translated OpenCV image  

        """

        rows,cols = img.shape
        shiftx, shifty = 0, 0
        translate_m = np.float32([ [1, 0, int(robot_pos[0])], [0, 1, int(robot_pos[1])] ])
        if robot_pos[0] < 0:
            shiftx = cols - int(robot_pos[0])
        elif robot_pos[1] < 0:
            shifty = rows - int(robot_pos[1])
        else:
            shiftx = cols + int(robot_pos[0])
            shifty = rows + int(robot_pos[1])

        rob = cv2.warpAffine(img, translate_m, (shiftx, shifty))
        #print("Translation matrix :\n", translate_m)
        return rob

    def rotate(self, img, angle, pivot):
        
        """ Rotate a given map image in the counter or clockwise direction by an angle θ,
        where the rotation pivot point will always be from the perspective of wheelchair.

        Args:
            img: OpenCV image
            angle: Rotation in degrees 
            pivot: Center of Rotation
        Returns:
           rob: Rotated OpenCV image  

        """

        (rows,cols) = img.shape

        rotation_m = cv2.getRotationMatrix2D((rows/2, cols/2), angle[2], 1)
        rob = cv2.warpAffine(img, rotation_m, (cols,rows))
        #print("Rotation matrix :\n", rotation_m) #self.rotation_m[:2, :2])

        return rob

    def get_displacement(self, pre, trans):

        """ Method to map a point in the image after applying a transformation

        Args:
            pre: List of X & Y coordinates of a pixel
            trans: List of x, y & theta transformation
        Returns:
           matrixC: Homogemeous matrix for transformed pixel coordinates  

        """

        # pre_pos takes the position of a vertex before transformation 
        # post_pos gives the position of pre_pos vertex after transformation

        #robot_pos=[190, 190] # Focus/perspective point of the map 

        image = self.image #self.image
        rows, cols = image.shape

        mat_Dxy = np.array([[pre[0]],
                            [pre[1]]
                        ])

        mat_Dsx = np.array([[trans[0]],
                            [trans[1]]
                        ])

        mat_Rid = np.identity(2)

        pivot=[rows/2, cols/2]
        matR = cv2.getRotationMatrix2D((int(pivot[0]),int(pivot[1])), trans[2], 1) # mat_Rth = self.rotation_m
        mat_Rth = np.block([
            [matR],
            [np.zeros((1,2)), 1.0]
        ])

        matrixA = self.set(mat_Rid, mat_Dsx)
        matrixT = np.dot(mat_Rth, matrixA)
        matrixA = self.set(mat_Rid, mat_Dxy)

        matrixC = np.dot(matrixT, matrixA)
        #print ("Homogeneous def:\n", matrixC[:, 2:3])

        return matrixC[:, 2:3]

    def transform(self, trans_vector, center=[116.0, 182.0]):

        """ Method to apply translation and rotation to a given image

        Args:
            trans_vector: List of X, Y and theta coordinates
            center: List of center pixel of the image
        Returns:
           (None)

        """

        # Update translation position if there is displacement in (x,y) and if there is no rotation 
        if trans_vector[2] == 0:

            trans = self.translate(self.image, trans_vector)
            # To update the position of wheelchair in the map take-away the transformation vector from the center/focus point
            trans = self.draw_rob(trans)

            return trans

        elif trans_vector[2] != 0:

            trans = self.translate(self.image, trans_vector)
            trans = self.rotate(trans, trans_vector, center)
            trans = self.draw_rob(trans)
            #self.display(trans)

            return trans

    def display(self, image):

        """ Method to display OpenCV image

        Args:
            image: OpenCV image
        Returns:
           (None)

        """

        while(1):
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.imshow('image', image)
            if not cv2.waitKey(0) & 0xFF == 27:
                break
        cv2.destroyAllWindows()

        # return image # testing