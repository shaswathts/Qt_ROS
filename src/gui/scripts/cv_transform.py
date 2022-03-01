#!/usr/bin/env python

import numpy as np
import cv2


class UpdateTransformation:

    def __init__(self, transformation=[0, 0, 0], displacement=False, pre_xy=[]):
        self.transformation = transformation
        self._trans = opencv()
        self._pos = displacement
        self.pre_xy = pre_xy

    def return_displacement(self):
        if self._pos:
            _img = self._trans.transform(self._transformation)
            return _img
        else:
            _shift = self._trans.get_displacement(self.pre_xy, self._transformation)
            return _shift

    # Getter method
    @property
    def transformation(self):
        x = self.return_displacement()
        return x #self._transformation 

    # Setter method
    @transformation.setter
    def transformation(self, value):
        if value[0] > 190:
            raise ValueError("outside map boundary!")
        self._transformation = value


class opencv():

    def __init__(self):# Load the image
        # cv2.IMREAD_COLOR = 1
        # cv2.IMREAD_GRAYSCALE = 0
        # cv2.IMREAD_UNCHANGED = -1
        self.image = cv2.imread("/home/intern/adapt_Pyrqt/src/smp_gui/src/second_map.pgm", cv2.IMREAD_UNCHANGED)

    # To create a Homgeneous matrix 
    def set(self, rotation, translation):

        mat = np.block([
                        [rotation, translation.reshape(2,1)],
                        [np.zeros((1,2)), 1.0]
                    ])

        return mat

    # Drawing clusters 
    def draw(self, img, cluster_list, robot_pos):
        #cluster_list = [[180.0, 170.0], [200.0, 170.0]]
        #cluster_list = [[130.0, 120.0], [150.0, 120.0]]
        for cluster in cluster_list:
            rob = cv2.drawMarker(img , (int(cluster[0]), int(cluster[1])), (0, 255, 0), cv2.MARKER_TRIANGLE_UP,
                                        markerSize=10, thickness=2, line_type=cv2.LINE_AA)

        return rob

    # This will always be the center of the map to get the perspective view  
    def draw_rob(self, img, robot_pos=[190, 190]):
        rob = cv2.drawMarker(img , (int(robot_pos[0]), int(robot_pos[1])), (0, 255, 0), cv2.MARKER_TRIANGLE_UP,
                                        markerSize=10, thickness=2, line_type=cv2.LINE_AA)
        return rob

    def translate(self, img, robot_pos):

        '''
        This matrices will translate a given map image to a new (x,y) position, while keeping the 
        wheelchair static.
        '''

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
        
        '''
        This matrices rotate a given map image in the counter or clockwise direction by an angle θ,
        where the rotation pivot point will always be from the perspective of wheelchair.
        '''

        (rows,cols) = img.shape

        rotation_m = cv2.getRotationMatrix2D((int(pivot[0]),int(pivot[1])), angle[2], 1)
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

        mat_Dxy = np.array([[pre[0]],
                            [pre[1]]
                        ])

        mat_Dsx = np.array([[trans[0]],
                            [trans[1]]
                        ])

        mat_Rid = np.identity(2)

        pivot=[0, 0]
        matR = cv2.getRotationMatrix2D((int(pivot[0]),int(pivot[1])), trans[2], 1) # mat_Rth = self.rotation_m
        mat_Rth = np.block([
            [matR],
            [np.zeros((1,2)), 1.0]
        ])

        matrixA = self.set(mat_Rid, mat_Dsx)
        matrixT = np.dot(mat_Rth, matrixA)
        matrixA = self.set(mat_Rid, mat_Dxy)

        matrixC = np.dot(matrixT, matrixA)
        print ("Homogeneous def:\n", matrixC)

        return matrixC

    def transform(self, trans_vector, center=[116.0, 182.0]):

        """ Method to apply translation and rotation to a given image

        Args:
            trans_vector: List of X, Y and theta coordinates
            center: List of center pixel of the image
        Returns:
           (None)

        """

        cluster_list = [[180.0, 170.0], [200.0, 170.0]] # Get this cluster data list from the camera 

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

    def display(self, trans):

        while(1):
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.imshow('image', trans)
            if not cv2.waitKey(0) & 0xFF == 27:
                break
        cv2.destroyAllWindows()

        return trans

""" x = opencv()
x.estimate_touch() """