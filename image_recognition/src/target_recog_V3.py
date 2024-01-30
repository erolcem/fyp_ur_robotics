#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time
from sensor_msgs.msg import Image
from image_recognition.msg import publishdata
from geometry_msgs.msg import PoseStamped, Pose
import tf

camera_pixel_x = 640
camera_pixel_y = 480
class ImageRecognition(object):

    def __init__(self):
        rospy.init_node('Read_image',anonymous=True)
        self.build_bar()
        self.rotate_angle = 0
        self.bridge = CvBridge()
        self.depth_array = [[0 for x in range(camera_pixel_x)] for y in range(camera_pixel_y)] 
        #self.depthbridge = CvBridge()

        try:
            self.sub_colour = rospy.Subscriber("/camera/color/image_raw", Image, self.convert_colour_image, queue_size=10, buff_size=152428800)
                
        except:
            print("colour error")

        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        self.width = 0.018
        self.length = 0.03
        #self.camera2hand=[-0.04,0.165,0,0,1.5705,-3.1415]
        self.camera2hand=[0,0,0,0,1.5705,-3.1415]
        

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
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "")
            
        except CvBridgeError, e:
            print e
        
        self.depth_array = np.array(depth_image)
        #rospy.loginfo(self.depth_array)
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
        hsv = cv2.cvtColor(self.rgb_array, cv2.COLOR_BGR2HSV)

        # colour filter
        u_H = cv2.getTrackbarPos("upper_H", "Filter")
        u_S = cv2.getTrackbarPos("upper_S", "Filter")
        u_V = cv2.getTrackbarPos("upper_V", "Filter")
        l_H = cv2.getTrackbarPos("lower_H", "Filter")
        l_S = cv2.getTrackbarPos("lower_S", "Filter")
        l_V = cv2.getTrackbarPos("lower_V", "Filter")
        # upper_bound1 = np.array([255, 255, 255])
        # lower_bound1 = np.array([130, 180, 120])
        # upper_bound2 = np.array([43, 255, 255])
        # lower_bound2 = np.array([0, 150, 120])
        upper_bound = np.array([u_H, u_S, u_V])
        lower_bound = np.array([l_H, l_S, l_V])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        # mask2 = cv2.inRange(hsv, lower_bound2, upper_bound2)
        # mask = cv2.bitwise_or(mask1, mask2)
        image_filter = cv2.bitwise_and(self.rgb_array, self.rgb_array, mask=mask)
        cv2.imshow("image_filter", image_filter)

        # transform to 0-1 picture
        _, thresh = cv2.threshold(mask, 20, 255, cv2.THRESH_BINARY)
        self.thresh = cv2.dilate(thresh, None, iterations=3)
        print(self.thresh)


    def target_recognition(self):
        # find boundary
        area = []
        point1 = [0,0]
        point2 = [0,0]
        point3 = [0,0]
        point4 = [0,0]
        point1_rot = [0,0]
        point2_rot = [0,0]
        point3_rot = [0,0]
        point4_rot = [0,0]
        point1_ref = [0,0]
        point2_ref = [0,0]
        point3_ref = [0,0]
        point4_ref = [0,0]
        point1_main = [0,0]
        point2_main = [0,0]
        point3_main = [0,0]
        point4_main = [0,0]
        _, contours, _ = cv2.findContours(self.thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area.append(cv2.contourArea(contour))
            contour = contours[area.index(max(area))]
            rect = cv2.minAreaRect(contour)

            
            if cv2.contourArea(contour) > 100:
                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(self.rgb_array, [box], 0, (36,255,12), 3)
                #mass_centre = cv2.moments(contour)
                mass_centre_x = rect[0][0]
                mass_centre_y = rect[0][1]



                
                quarter_x = 0.2*rect[1][0]
                quarter_y = 0.2*rect[1][1]
                point1[0] = mass_centre_x + quarter_x
                point1[1] = mass_centre_y + quarter_y
                point2[0] = mass_centre_x - quarter_x
                point2[1] = mass_centre_y + quarter_y
                point3[0] = mass_centre_x - quarter_x
                point3[1] = mass_centre_y - quarter_y
                point4[0] = mass_centre_x + quarter_x
                point4[1] = mass_centre_y - quarter_y

                point1_ref[0] = point1[0]-mass_centre_x
                point1_ref[1] = point1[1]-mass_centre_y
                point2_ref[0] = point2[0]-mass_centre_x
                point2_ref[1] = point2[1]-mass_centre_y
                point3_ref[0] = point3[0]-mass_centre_x
                point3_ref[1] = point3[1]-mass_centre_y
                point4_ref[0] = point4[0]-mass_centre_x
                point4_ref[1] = point4[1]-mass_centre_y


                point1_rot[0] = point1_ref[0]*np.cos((rect[2])*3.1415/180)-point1_ref[1]*np.sin((rect[2])*3.1415/180)
                point1_rot[1] = point1_ref[0]*np.sin((rect[2])*3.1415/180)+point1_ref[1]*np.cos((rect[2])*3.1415/180)
                point2_rot[0] = point2_ref[0]*np.cos((rect[2])*3.1415/180)-point2_ref[1]*np.sin((rect[2])*3.1415/180)
                point2_rot[1] = point2_ref[0]*np.sin((rect[2])*3.1415/180)+point2_ref[1]*np.cos((rect[2])*3.1415/180)
                point3_rot[0] = point3_ref[0]*np.cos((rect[2])*3.1415/180)-point3_ref[1]*np.sin((rect[2])*3.1415/180)
                point3_rot[1] = point3_ref[0]*np.sin((rect[2])*3.1415/180)+point3_ref[1]*np.cos((rect[2])*3.1415/180)
                point4_rot[0] = point4_ref[0]*np.cos((rect[2])*3.1415/180)-point4_ref[1]*np.sin((rect[2])*3.1415/180)
                point4_rot[1] = point4_ref[0]*np.sin((rect[2])*3.1415/180)+point4_ref[1]*np.cos((rect[2])*3.1415/180)

                point1_main[0] = int(point1_rot[0]+mass_centre_x)
                point1_main[1] = int(point1_rot[1]+mass_centre_y)
                point2_main[0] = int(point2_rot[0]+mass_centre_x)
                point2_main[1] = int(point2_rot[1]+mass_centre_y)
                point3_main[0] = int(point3_rot[0]+mass_centre_x)
                point3_main[1] = int(point3_rot[1]+mass_centre_y)
                point4_main[0] = int(point4_rot[0]+mass_centre_x)
                point4_main[1] = int(point4_rot[1]+mass_centre_y)


                cv2.circle(self.rgb_array,(int(mass_centre_x),int(mass_centre_y)),2,(0,0,255), -1)

                
                


                position = PoseStamped()
                if(rect[1][0]<rect[1][1]):
                    angle = 90 - rect[2]                #rectangle angle is rect[2], rect[0][0] & rect[0][1] is the central coordinates; rect[1][0] and rect[1][1] are the dimensions of the rectangular
                    ratio = self.length/rect[1][1]
                    position.pose.position.x = (mass_centre_x - camera_pixel_x/2)*-1*ratio+self.camera2hand[0]
                    position.pose.position.y = (mass_centre_y - camera_pixel_y/2)*1*ratio+ self.camera2hand[1]
                    
                    
                else:
                    angle = -rect[2]
                    ratio = self.length/rect[1][0]
                    position.pose.position.x = (mass_centre_x - camera_pixel_x/2)*-1*ratio+self.camera2hand[0]
                    position.pose.position.y = (mass_centre_y - camera_pixel_y/2)*1*ratio+ self.camera2hand[1]
                    
                    
                position.pose.position.z = 0#self.depth_array[mass_centre_y][mass_centre_x] - self.camera2hand[2]
                rotz = angle*3.1415/180 + self.camera2hand[5]
              #  rotx = rot_x + self.camera2hand[4]
              #  roty = rot_y + self.camera2hand[5]
                center_strXY = str(int(mass_centre_x)) + ',' + str(int(mass_centre_y)) + ',' + str(rect[2])
                self.rotate_angle = rect[2]
                cv2.putText(self.rgb_array, center_strXY, (int(mass_centre_x),int(mass_centre_y)), self.font, 1, (255, 255, 0), 2)
                quaternion = tf.transformations.quaternion_from_euler(0,1.5707,rotz)#rotx, roty, rotz)
                position.pose.orientation.x = quaternion[0]#np.sin(rotx/2)*np.cos(roty/2)*np.cos(rotz/2) - np.cos(rotx/2)*np.sin(roty/2)*np.sin(rotz/2)
                position.pose.orientation.y = quaternion[1]#np.cos(rotx/2)*np.sin(roty/2)*np.cos(rotz/2) + np.sin(rotx/2)*np.cos(roty/2)*np.sin(rotz/2)
                position.pose.orientation.z = quaternion[2]#np.cos(rotx/2)*np.cos(roty/2)*np.sin(rotz/2) - np.sin(rotx/2)*np.sin(roty/2)*np.cos(rotz/2)
                position.pose.orientation.w = quaternion[3]#np.cos(rotx/2)*np.cos(roty/2)*np.cos(rotz/2) + np.sin(rotx/2)*np.sin(roty/2)*np.sin(rotz/2)



    def closewindow(self):
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()
        self.thresh.release()
        self.depth_array.release()
        file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/angle.out","w")
        file.write(str(rect[2]))
        file.close
        
    
if __name__ == "__main__":
    algorithm = ImageRecognition()
    algorithm.closewindow()



    
    
    
    
    


