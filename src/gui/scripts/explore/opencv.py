#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

import numpy as np
import cv2


# ROS node initilization
#rospy.init_node('OpenCV', anonymous=True)
class opencv():
    def __init__(self):# Load the image
        # cv2.IMREAD_COLOR = 1
        # cv2.IMREAD_GRAYSCALE = 0
        # cv2.IMREAD_UNCHANGED = -1
        self.img = cv2.imread("/home/intern/adapt_Pyrqt/src/smp_gui/src/second_map.pgm", cv2.IMREAD_UNCHANGED)


    def drwa_rob(self, map, robot_pos):
        
        (rows,cols) = map.shape
            
        translate_m = np.float32([ [1, 0, int(robot_pos[0])], [0, 1, int(robot_pos[1])] ])
        rob = cv2.warpAffine(map, translate_m, (cols,rows))

        rotation_m = cv2.getRotationMatrix2D((int(robot_pos[0]),int(robot_pos[1])), robot_pos[2], 1)
        dst = cv2.warpAffine(rob, rotation_m, (cols,rows))

        rob = cv2.drawMarker(dst , (190,150), (0, 255, 0), cv2.MARKER_TRIANGLE_UP,
                                        markerSize=10, thickness=2, line_type=cv2.LINE_AA)
        return rob

    def draw_obj(self, map, cluster_list):

        for cluster in cluster_list:
            objs = cv2.drawMarker(map , (int(cluster[0]), int(cluster[1])), (0, 255, 0), cv2.MARKER_DIAMOND,
                                        markerSize=8, thickness=2, line_type=cv2.LINE_AA)
            print('Cluster in pixel ; '+str(cluster[0])+ ' '+ str(cluster[1]))   
        return objs

    # Get cluster data from TF buffer
    def cluster_pos(self):
        
        i = 0
        get_pose = True
        cluster_list = []
        pix_scale = 37 #rospy.get_param('pixel_scale')
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
            
        while get_pose:
            i = i+1
            print('cluster' + str(i))
            try:
                trans = tfBuffer.lookup_transform("map", "cluster"+str(i), rospy.Time(0), rospy.Duration(5))
                cluster = [trans.transform.translation.x * pix_scale, trans.transform.translation.y * pix_scale]
                cluster_list.append(cluster)
            
            except (tf2_ros.LookupException):
                get_pose = False
                continue
        
        #trans2 = tfBuffer.lookup_transform("map", "d435_color_optical_frame", rospy.Time(0), rospy.Duration(5))
        #cluster_list.append([trans2.transform.translation.x * pix_scale, trans2.transform.translation.y * pix_scale])
        
        return self.draw_obj(self.img, cluster_list)

    # Draw the cluster pos on map image
    #img = cluster_pos()

    # Draw the posistion of wheelchair w.r.t map's pixel location, translation & rotation robot_pos=[x,y,rad]
    #rob = drwa_rob(img, robot_pos=[182, 116, 10])



    #cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    #cv2.imshow('image', rob)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    '''
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
