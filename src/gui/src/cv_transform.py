#!/usr/bin/env python

from time import sleep
import random
import numpy as np
import cv2

import rospy
from std_msgs.msg import Int32


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

    def return_transformation(self):
        print("In ret_transformation calling openCV.transform")
        

    # Getter method
    @property
    def transformation(self):
        #print("Getting value...")
        x = self.return_displacement()
        return x #self._transformation 
    
    # Setter method
    @transformation.setter
    def transformation(self, value):
        #print("Setting value...")
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
       
        return rob

    def rotate(self, img, angle, pivot):
        '''
        This matrices rotate a given map image in the counter or clockwise direction by an angle θ,
        where the rotation pivot point will always be from the perspective of wheelchair.
        '''
        (rows,cols) = img.shape

        rotation_m = cv2.getRotationMatrix2D((int(pivot[0]),int(pivot[1])), angle[2], 1)
        rob = cv2.warpAffine(img, rotation_m, (cols,rows))
        #print("Rotation matrix :", rotation_m) #self.rotation_m[:2, :2])

        return rob
    
    def remap_center(self, image, tran_vector):
        pass

    def get_displacement(self, pre, trans):
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

    def transform(self, trans_vector, center=[190, 190]):
        '''
        arg = image (Map image)
        arg = Dx (Displacement in x direction)
        arg = Dy (Displacement in y direction)
        arg = Dth (Rotation theta in degree)
        arg = pivot (pivot point for rotation)
        '''

        cluster_list = [[180.0, 170.0], [200.0, 170.0]] # Get this cluster data list from the camera 

        #initial = self.draw_rob(image, center)
        #self.display(initial)
        
         # Update translation position if there is displacement in (x,y) and if there is no rotation 
        if trans_vector[2] == 0:
            trans = self.translate(self.image, trans_vector)
            # To update the position of wheelchair in the map take-away the transformation vector from the center/focus point
            #remap = list(map(lambda x: -x, trans_vector))
            #remap_perspective = self.get_displacement(center, remap)
            trans = self.draw_rob(trans)
            #self.display(trans)
            return trans

        elif trans_vector[2] != 0:
            trans = self.translate(self.image, trans_vector)
            trans = self.rotate(trans, trans_vector, center)
            trans = self.draw_rob(trans)
            #self.display(trans)
            return trans
            
    def display(self, trans):
        while(1):
            #cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.imshow('image', trans)
            if not cv2.waitKey(100) & 0xFF == 27:
                break
        cv2.destroyAllWindows()
        
        return trans

'''def wheelchair_pose(data):
    x = opencv()
    _get_pose = True
    y = random.randint(0, 190)
    xy = [y, y+28, y]
    # for a in range(-180, 180):
    #     s = round( float( "{:.02f}".format( np.sin( np.radians(a) ) * 100 ) ) ) // 2
    #     #xy = [s, s, -a]
    # h = UpdateTransformation(xy, _get_pose)
    # x.display(h.transformation)
    
    h = x.transform(xy)
    x.display(h)'''


""" # ROS node initilization
rospy.init_node('GUI_node', anonymous=True)

rospy.Subscriber("/trigger", Int32, wheelchair_pose)
rospy.spin() """

'''
#img = cv2.imread("/home/intern/adapt_Pyrqt/src/smp_gui/src/second_map.pgm", cv2.IMREAD_UNCHANGED)
#rob = opencv()

#point = [0, 0]
#trans = [0, 0, 0] # (x, y, th)
#angle = 0
#pivot = [190, 190]

#rob.get_displacement(point, trans, angle, pivot)
#rob.transform(trans, pivot)

'''
'''
wheelChair_pos = []

# When the wheelChair posistion changes -> append the (x,y) to wheelChair_pos
T1 = np.radians(0)
T2 = np.radians(0)

x, y = 0, 0 # Transform map to (x,y) pos -> obsulute system

x1, y1 = 0, 0 # This takes the displacement of map image and maps the objects to their original pos 
x2, y2 = 0.0, 0.0 # This takes the current posistion of the objects and maps it to the original pos based on the previous displacement

wheelChair_pos.append(x)
wheelChair_pos.append(y)

mat_R_0_1 = np.array([[np.cos(T1), -np.sin(T1)],
                      [np.sin(T1),  np.cos(T1)]
                    ])

mat_R_1_2 = np.array([[np.cos(T2), -np.sin(T2)],
                      [np.sin(T2),  np.cos(T2)]
                     ])

mat_R_0_2 = np.dot(mat_R_0_1, mat_R_1_2)

#print ("Rotation (R_0_1):\n", mat_R_0_1)
#print ("Rotation (R_0_2):\n", mat_R_1_2)

mat_D_0_1 = np.array([[x1*np.cos(T1)],
                      [y1*np.cos(T1)]
                    ])

mat_D_1_2 = np.array([[x2*np.cos(T2)],
                      [y2*np.cos(T2)]
                    ])
'''
'''To find displacement multiply with the rotation matrix as identity 
    else take the matrix from cv2.getRotationMatrix2D and multiply '''
'''
matrixA = rob.set(mat_R_0_1, mat_D_0_1)
matrixB = rob.set(mat_R_1_2, mat_D_1_2)
matrixC = np.dot(matrixA, matrixB)

robot_pose = []
#robot_pose = [H0_2[0][2], H0_2[1][2], T2]
robot_pose.append(matrixC[0][2])
robot_pose.append(matrixC[1][2])
robot_pose.append(T1*180/np.pi)

#print ("Homogeneous :\n", matrixC)
#print ("Displacement :\n", robot_pose)

#trans = rob.translate(img, wheelChair_pos)
#trans = rob.rotate(trans, angle)
#trans = rob.draw_rob(trans, robot_pos=[190, 190])
#trans = rob.draw(trans)
#rob.display(trans)
#trans = rob.translate(img, robot_pos=[-50, -50])
#trans = rob.draw_rob(trans, robot_pos=[87, 58])
#rob.display(trans)


# Display the image
cv2.imshow('map', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
# Display the image window with resize 
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()

'''