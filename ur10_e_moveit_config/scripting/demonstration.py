#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from image_recognition.msg import publishdata
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import numpy as np
import tf
from gripper import rg6
import os

class MoveItik(object):
    def __init__(self):
        #  Initialize API of the move_group 
        #moveit_commander.roscpp_initialize(sys.argv)
        self.object_position = AprilTagDetectionArray()
        self.object_target = AprilTagDetection()
        self.origin_position = PoseStamped()
        self.to_link = '/base'
        self.rotangle = 0.0
        self.reference_frame = 'base'
        self.camera_frame = 'camera_link'
        self.to_link = '/world'
        self.from_link = '/camera_link'
        self.cal = "false"
        self.count = 0
        self.pos_x = 0
        self.pos_y = 0
        self.euler_angle = [0,0,0]
        self.gripper_rg6 = rg6()
        self.depth = 0.0
        self.depthbridge = CvBridge()
        self.ip_robotA = "192.168.1.100"
        self.ip_robotB = "192.168.1.101"
        self.object_target.id.append(0)


        # Initialize ROS nodes
        rospy.init_node('moveit_ik_demo')
        self.tf_listener = tf.TransformListener()
        #sub = rospy.Subscriber("image_recognition", PoseStamped, self.callback, queue_size=10)
        sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback, queue_size=1)
        # sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callback_depth, queue_size=1)
                               
        # Initialize arm_group used in move_group
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        
        # get the name of end_effector link
        self.end_effector_link = self.arm.get_end_effector_link()
        print(self.end_effector_link)
                        
        # set up reference
        self.reference_frame = 'base'
        self.camera_frame = 'camera_link'
        self.to_link = '/world'
        self.from_link = 'camera_link'

        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # allow replanning
        self.arm.allow_replanning(True)
        
        # set up the tolerent of position and orientation.
        self.arm.set_goal_position_tolerance(0.0001)
        self.arm.set_goal_orientation_tolerance(0.001)
       
        # Allow max acc and vel
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # self.go_home()
        self.demonstrate()
        self.calibrate()

    def demonstrate(self):
    	self.demonstrate_position()
    	self.change_component(10,0.1,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        print(self.object_target.id[0])
        while self.object_target.id[0] != 10:
        	print("waiting for the tag")
        	rospy.sleep(5)
        else:
        	while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
        		self.calibrate()
        	else:
				print("target is aimed")
                # self.record_position()
				rospy.sleep(5)
        	


    def go_home(self):
       
        # set up a home pose
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.2
        target_pose.pose.position.y = -0.7
        target_pose.pose.position.z = 0.7
        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        self.arm.set_start_state_to_current_state()
        
        # set up the target pose of end effector 
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # trajectory
        traj = self.arm.plan()
        
        #execute following the plan
        self.arm.execute(traj)
        rospy.sleep(1)
    def demonstrate_position(self):
    	target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.2
        target_pose.pose.position.y = -0.9
        target_pose.pose.position.z = 0.7
        quaternion = tf.transformations.quaternion_from_euler(1.57, -3.1415, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        self.arm.set_start_state_to_current_state()
        
        # set up the target pose of end effector 
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # trajectory
        traj = self.arm.plan()
        
        #execute following the plan
        self.arm.execute(traj)
        print("done")
        rospy.sleep(1)

    def calibrate(self):

        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = 'camera_link'
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = self.object_target.pose.pose.pose.position.x
        target_pose.pose.position.y = self.object_target.pose.pose.pose.position.y
        target_pose.pose.position.z = self.object_target.pose.pose.pose.position.z
        target_pose.pose.orientation.x = self.object_target.pose.pose.pose.orientation.x
        target_pose.pose.orientation.y = self.object_target.pose.pose.pose.orientation.y
        target_pose.pose.orientation.z = self.object_target.pose.pose.pose.orientation.z
        target_pose.pose.orientation.w = self.object_target.pose.pose.pose.orientation.w

        
        

        target_pose.header.stamp = rospy.Time.now()
        t = rospy.Time(0)
        self.tf_listener.waitForTransform(self.to_link,self.from_link,t,rospy.Duration(5))
        if self.tf_listener.canTransform(self.to_link,self.from_link,t):
            self.tf_listener.waitForTransform(self.to_link,self.from_link,rospy.Time.now(),rospy.Duration(5))
            target_pose_transferred = self.tf_listener.transformPose(self.to_link,target_pose)
            target_pose_transferred.pose.position.y = current_pose.pose.position.y
            quaternion = tf.transformations.quaternion_from_euler(-1.5708, 0, 0)
            target_pose_transferred.pose.orientation.x = quaternion[0]
            target_pose_transferred.pose.orientation.y = quaternion[1]
            target_pose_transferred.pose.orientation.z = quaternion[2]
            target_pose_transferred.pose.orientation.w = quaternion[3]

        else:
            rospy.logerr('Transformation is not possible!')
            sys.exit(0)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose_transferred, self.end_effector_link)   #try to centralize the camera just a test
        traj = self.arm.plan()
        self.arm.execute(traj)

    def change_component(self,tag_number,size,name):
		file = open("/home/cheavporchea/apriltag/src/apriltag_ros/apriltag_ros/config/tags.yaml","w")
		file.write("standalone_tags:"+"\n"+"  [{id: "+str(tag_number)+", size: "+str(size)+", name: "+str(name)+"}]")
		file.close()



    def callback(self, position):
        target_pose = PoseStamped()
        self.object_position = position
        self.object_target = self.object_position.detections[0]
        target_pose.pose.orientation.x = self.object_target.pose.pose.pose.orientation.x
        target_pose.pose.orientation.y = self.object_target.pose.pose.pose.orientation.y
        target_pose.pose.orientation.z = self.object_target.pose.pose.pose.orientation.z
        target_pose.pose.orientation.w = self.object_target.pose.pose.pose.orientation.w
        self.euler_angle = tf.transformations.euler_from_quaternion([target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w])

	def callback_depth(self, ros_image):
		try:
			depth_image = self.depthbridge.imgmsg_to_cv2(ros_image, "32FC1")
		except CvBridgeError, e:
			print e
		depth_array = np.array(depth_image)
		self.depth = depth_array[240][320]
		self.depth1 = np.shape(depth_array)

	   
        

if __name__ == "__main__":
    operation = MoveItik()