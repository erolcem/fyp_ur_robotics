#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from image_recognition.msg import publishdata
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
import tf
from gripper import rg6

#home = [-0.03,0.7,-0.01,0,0,0,0]


class MoveItik(object):
    def __init__(self):
        #  Initialize API of the move_group 
        #moveit_commander.roscpp_initialize(sys.argv)
        self.object_target = AprilTagDetectionArray()

        # Initialize ROS nodes
        rospy.init_node('moveit_ik_demo')
        
        #sub = rospy.Subscriber("image_recognition", PoseStamped, self.callback, queue_size=10)
        sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback, queue_size=1)
        rospy.sleep(3)

                               

    def callback(self, position):

        self.object_target = position
        print(self.object_target)


if __name__ == "__main__":
    operation = MoveItik()


