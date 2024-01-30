#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time
from sensor_msgs.msg import Image


class ImageRecognition(object):

    def __init__(self):
        
        self.build_bar()
        self.bridge = CvBridge()
        self.depthbridge = CvBridge()
        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.convert_depth_image)
        self.sub_colour = rospy.Subscriber("/camera/color/image_raw", Image, self.convert_colour_image)
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        

    @staticmethod
    def nothing():
        pass

    @staticmethod
    def build_bar():
        # A bar to edit the colour filter
        cv2.namedWindow('Filter')
        cv2.createTrackbar("upper_H", "Filter", 255, 255, ImageRecognition.nothing)
        cv2.createTrackbar("upper_S", "Filter", 255, 255, ImageRecognition.nothing)
        cv2.createTrackbar("upper_V", "Filter", 255, 255, ImageRecognition.nothing)
        cv2.createTrackbar("lower_H", "Filter", 0, 255, ImageRecognition.nothing)
        cv2.createTrackbar("lower_S", "Filter", 0, 255, ImageRecognition.nothing)
        cv2.createTrackbar("lower_V", "Filter", 0, 255, ImageRecognition.nothing)

    def convert_depth_image(self, ros_image):
        
        try:
            depth_image = self.depthbridge.imgmsg_to_cv2(ros_image, "32FC1")
        except CvBridgeError, e:
            print e
        
        self.depth_array = np.array(depth_image)
        cv2.imshow("depth",self.depth_array) 
        cv2.waitKey(3)

    def convert_colour_image(self, ros_image):
        
        try:
            colour_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        self.rgb_array = np.array(colour_image)
        self.select_target()
        self.target_recognition()
        cv2.imshow("targeting",self.rgb_array) 
        cv2.waitKey(3)

    def select_target(self):
        # get the image and transform it to HSV
        
        hsv = cv2.cvtColor(self.rgb_array, cv2.COLOR_BGR2HSV)

        # colour filter
        u_H = cv2.getTrackbarPos("upper_H", "Filter")
        u_S = cv2.getTrackbarPos("upper_S", "Filter")
        u_V = cv2.getTrackbarPos("upper_V", "Filter")
        l_H = cv2.getTrackbarPos("lower_H", "Filter")
        l_S = cv2.getTrackbarPos("lower_S", "Filter")
        l_V = cv2.getTrackbarPos("lower_V", "Filter")
        upper_bound = np.array([u_H, u_S, u_V])
        lower_bound = np.array([l_H, l_S, l_V])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        image_filter = cv2.bitwise_and(self.rgb_array, self.rgb_array, mask=mask)
        cv2.imshow("image_filter", image_filter)

        # transform to 0-1 picture
        _, thresh = cv2.threshold(mask, 20, 255, cv2.THRESH_BINARY)
        self.thresh = cv2.dilate(thresh, None, iterations=3)


    def target_recognition(self):
        # find boundary
        area = []
        _, contours, _ = cv2.findContours(self.thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area.append(cv2.contourArea(contour))
            contour = contours[area.index(max(area))]
            if cv2.contourArea(contour) > 1000:
                cv2.drawContours(self.rgb_array, contours, area.index(max(area)), (0, 0, 255), 3)
                # ret = cv2.matchShapes(contour, cnt2, 1, 0)  # use matchShapes to recognize the image
                # x, y, w, h = cv2.boundingRect(contour)
                mass_centre = cv2.moments(contour)
                mass_centre_x = int(mass_centre['m10'] / mass_centre['m00'])
                mass_centre_y = int(mass_centre['m01'] / mass_centre['m00'])
                center_strXY = str(mass_centre_x) + ',' + str(mass_centre_y) + ',' + str(self.depth_array[mass_centre_y][mass_centre_x])
                cv2.putText(self.rgb_array, center_strXY, (mass_centre_x, mass_centre_y), self.font, 1, (255, 255, 0), 2)
                image_h = self.rgb_array.shape[0]
                image_w = self.rgb_array.shape[1]
                self.vertical_move = mass_centre_y - image_h/2
                self.horizontal_move = mass_centre_x - image_w/2
                print(self.horizontal_move, self.vertical_move)


    def body_part(self):
        
        # choose_algorithms = int(input("please choose the algorithms (1, 2 or 3)"))
        rospy.init_node('read_rgb_image',anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()
    
if __name__ == "__main__":
    algorithm = ImageRecognition()
    algorithm.body_part()


    
    
    
    
    


