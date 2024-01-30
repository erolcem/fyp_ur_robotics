#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time
from sensor_msgs.msg import Image
import sys


class ImageRecognition():

    def __init__(self):
        self.bridge = CvBridge()
        self.sub_colour = rospy.Subscriber("/camera/color/image_raw", Image, self.convert_colour_image)
        

    def convert_colour_image(self, ros_image):
        
        try:
            colour_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            
        except CvBridgeError, e:
            print e

        cv2.imshow("hello",colour_image)
        cv2.waitKey(3)



    def body_part(self):
        
    # choose_algorithms = int(input("please choose the algorithms (1, 2 or 3)"))
        rospy.init_node('read_rgb_image',anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ic=ImageRecognition()
    ic.body_part()
    


    
    
    
    
    


