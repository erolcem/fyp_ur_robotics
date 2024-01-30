#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, PoseWithCovarianceStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from image_recognition.msg import publishdata
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import numpy as np
import tf
from gripper import rg6
import os
#home = [-0.03,0.7,-0.01,0,0,0,0]


class MoveItik(object):
    def __init__(self):
        #  Initialize API of the move_group 
        #moveit_commander.roscpp_initialize(sys.argv)
        self.object_target = AprilTagDetectionArray()
        self.object_position = AprilTagDetection()
        self.to_link = '/base'
        self.rotangle = 0.0

       
        gripper_rg6 = rg6()

        # Initialize ROS nodes
        rospy.init_node('moveit_ik_demo')
        self.tf_listener = tf.TransformListener()
        #sub = rospy.Subscriber("image_recognition", PoseStamped, self.callback, queue_size=10)
        sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback, queue_size=1)
                               
        # Initialize arm_group used in move_group
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # get the name of end_effector link
        self.end_effector_link = self.arm.get_end_effector_link()
        print(self.end_effector_link)
                        
        # set up reference
        self.reference_frame = 'base'
        self.camera_frame = 'camera_link'
        self.to_link = '/world'
        self.from_link = '/camera_link'

        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # allow replanning
        self.arm.allow_replanning(True)
        
        # set up the tolerent of position and orientation.
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)
       
        # Allow max acc and vel
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
        rospy.sleep(1)
        self.change_component(20)
        os.system("roslaunch apriltag_ros continuous_detection.launch")
        self.go_home()
        self.go_storage()
        self.go_pickup()
        self.calibrate()
        os.system("pkill apriltag_ros")
        
        #while (abs(self.object_position.pose.pose.pose.position.x)>0.003 or abs(self.object_position.pose.pose.pose.position.y) > 0.003):
            #print(self.object_position.pose.pose.pose.position.x, self.object_position.pose.pose.pose.position.y)
            #print("targeting...")
            #self.calibrate()
        #else:
            #print("target is aimed")


        #self.go_pickup()
        #self.go_storage()
        #self.go_home()



        #self.calibrate()
        #for i in range(6):
            #self.calibrate()

        

        #self.go_home()
        #self.home2storage()
        #self.calibrate()
        #gripper_rg6.opengripper_slow()
        #self.storage2home()
        #self.precision_test()
        #self.calibration_pos()
        

    def go_home(self):
       
        # set up a home pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.2
        target_pose.pose.position.y = -0.7
        target_pose.pose.position.z = 0.7
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        self.arm.set_start_state_to_current_state()
        
        # set up the target pose of end effector 
        self.arm.set_pose_target(target_pose, self.end_effector_link, )
        
        # trajectory
        traj = self.arm.plan()
        
        #execute following the plan
        self.arm.execute(traj)
        rospy.sleep(1)

    def go_storage(self):
        # make sure we start from home
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = -0.7
        target_pose.pose.position.y = -0.2
        target_pose.pose.position.z = 0.7
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)          #self.object_position = self.object_target.detections

    def go_pickup(self):
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = -0.7
        target_pose.pose.position.y = -0.2
        target_pose.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)     
        #go down to the target


    def go_installing(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.9
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0
        
        # set the current set as start state
        self.arm.set_start_state_to_current_state()
        
        # set up the target pose of end effector 
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # trajectory
        traj = self.arm.plan()
        
        # execute following the plan
        self.arm.execute(traj)
        rospy.sleep(1)

    def callback(self, position):

        self.object_target = position
        self.object_position = self.object_target.detections[0]
        #euler = tf.transformations.euler_from_quaternion([self.object_position.pose.pose.pose.orientation.x,self.object_position.pose.pose.pose.orientation.y,self.object_position.pose.pose.pose.orientation.z,self.object_position.pose.pose.pose.orientation.w])
        # print(euler)
        
        
    def calibrate(self):

        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        t = rospy.Time(0)
        target_pose.pose.position.x = self.object_position.pose.pose.pose.position.x
        target_pose.pose.position.y = self.object_position.pose.pose.pose.position.y
        target_pose.pose.position.z = self.object_position.pose.pose.pose.position.z
        target_pose.pose.orientation.x = self.object_position.pose.pose.pose.orientation.x
        target_pose.pose.orientation.y = self.object_position.pose.pose.pose.orientation.y
        target_pose.pose.orientation.z = self.object_position.pose.pose.pose.orientation.z
        target_pose.pose.orientation.w = self.object_position.pose.pose.pose.orientation.w
        target_pose.header.frame_id = '/camera_link'
        target_pose.header.stamp = rospy.Time.now()
        self.tf_listener.waitForTransform(self.to_link,self.from_link,t,rospy.Duration(5))
        if self.tf_listener.canTransform(self.to_link,self.from_link,t):
            self.tf_listener.waitForTransform(self.to_link,self.from_link,rospy.Time.now(),rospy.Duration(5))
            target_pose_transferred = self.tf_listener.transformPose(self.to_link,target_pose)
            target_pose_transferred.pose.position.z = current_pose.pose.position.z
            target_pose_transferred.pose.orientation.x = current_pose.pose.orientation.x  #can't tranform the rotation properly that's why transform only the translation
            target_pose_transferred.pose.orientation.y = current_pose.pose.orientation.y
            target_pose_transferred.pose.orientation.z = current_pose.pose.orientation.z
            target_pose_transferred.pose.orientation.w = current_pose.pose.orientation.w
        else:
            rospy.logerr('Transformation is not possible!')
            sys.exit(0)


    def rot_calibrate(self):

        target_pose = PoseStamped()
        #current_pose = PoseStamped()
        #current_pose = self.arm.get_current_pose()
        target_pose = self.arm.get_current_pose()
        quaternion_list=[target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w]
        target_euler = tf.transformations.euler_from_quaternion(quaternion_list)
        target_pose.header.frame_id = '/camera_link'
        target_pose.header.stamp = rospy.Time.now()
       # current_euler = tf.transformations.euler_from_quaternion([0.709575,-0.00838021,0.703532,0.0384338])
       # print("current",current_euler[0],current_euler[1],current_euler[2])

       # t = rospy.Time(0)
        #target_pose = self.object_position.pose.pose
        #target_pose.pose.position.x += self.object_position.pose.pose.pose.position.x
       # target_pose.pose.position.y -= self.object_position.pose.pose.pose.position.y
        #arget_pose.pose.position.z = self.object_position.pose.pose.pose.position.z
        
        euler = tf.transformations.euler_from_quaternion([self.object_position.pose.pose.pose.orientation.x,self.object_position.pose.pose.pose.orientation.y,self.object_position.pose.pose.pose.orientation.z,self.object_position.pose.pose.pose.orientation.w])
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, -euler[2])
        self.rotangle = -euler[2]-3.1415
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]




      #  self.tf_listener.waitForTransform(self.to_link,self.from_link,t,rospy.Duration(5))
      #  if self.tf_listener.canTransform(self.to_link,self.from_link,t):
         #   self.tf_listener.waitForTransform(self.to_link,self.from_link,rospy.Time.now(),rospy.Duration(5))
          #  target_pose_transferred = self.tf_listener.transformPose(self.to_link,target_pose)
           # target_pose_transferred.pose.position.z = current_pose.pose.position.z


            #quaternion=tf.transformations.quaternion_from_euler(0,1.5707,3.1415)
            #target_pose_transferred.pose.orientation.x = quaternion[0]
            #target_pose_transferred.pose.orientation.y = quaternion[1]
            #target_pose_transferred.pose.orientation.z = quaternion[2]
            #target_pose_transferred.pose.orientation.w = quaternion[3]
            #hello = tf.transformations.euler_from_quaternion([target_pose_transferred.pose.orientation.x,target_pose_transferred.pose.orientation.y,target_pose_transferred.pose.orientation.z,target_pose_transferred.pose.orientation.w])
            #print("transferred",hello[0],hello[1],hello[2])

            #print ("transferred", target_pose_transferred)
       # else:
          #  rospy.logerr('Transformation is not possible!')
           # sys.exit(0)
        
    
        #while (self.object_target.pose.position.x>0.005):



        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)
        rospy.sleep(1)


        #target_pose.pose.orientation.x = self.object_target.pose.orientation.x
        #target_pose.pose.orientation.y = self.object_target.pose.orientation.y
        #target_pose.pose.orientation.z = self.object_target.pose.orientation.z
        #target_pose.pose.orientation.w = self.object_target.pose.orientation.w
        ##self.arm.set_start_state_to_current_state()
        #self.arm.set_pose_target(target_pose, self.end_effector_link)
        #traj = self.arm.plan()
        #self.arm.execute(traj)
        #rospy.sleep(1)

        
        #while (self.object_target.pose.position.y>0.005):
        


    def storage2home(self):

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()   

        target_pose.pose.position.z += 0.6
        self.arm.set_start_state_to_current_state() 
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)
        rospy.sleep(1)
        target_pose.pose.position.x += 0.5
        target_pose.pose.position.y -= 0.5
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)  

        target_pose.pose.position.x = 0.2
        target_pose.pose.position.y = 0.7
        target_pose.pose.position.z = 0.7
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, 0)
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

    def precision_test(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.70000
        target_pose.pose.position.y = -0.20000
        target_pose.pose.position.z = 0.20000
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, -3.1415)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)
        rospy.sleep(3)
        # make sure we start from home

        target_pose.pose.position.x = -0.65500
        target_pose.pose.position.y = -0.25500
        target_pose.pose.position.z = 0.20000
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, -3.1415)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(3)  

        target_pose.pose.position.x = -0.70000
        target_pose.pose.position.y = -0.20000
        target_pose.pose.position.z = 0.20000
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, -3.1415)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(3)     

    def change_component(self,id):
        file = open("/home/cheavporchea/apriltag/src/apriltag_ros/apriltag_ros/config/tags.yaml")
        file.write = ("standalone_tags:"+"\n\t"+"[{id: "+str(id)+", size: 0.015, name: test}]")
        file.close()   






        

if __name__ == "__main__":
    operation = MoveItik()


