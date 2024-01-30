#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from image_recognition.msg import publishdata
import numpy as np
import tf
from gripper import rg6

#home = [-0.03,0.7,-0.01,0,0,0,0]


class MoveItik(object):
    def __init__(self):
        #  Initialize API of the move_group 
        #moveit_commander.roscpp_initialize(sys.argv)
        self.object_target = PoseStamped()
        gripper_rg6 = rg6()

        # Initialize ROS nodes
        rospy.init_node('moveit_ik_demo')
        sub = rospy.Subscriber("image_recognition", PoseStamped, self.callback, queue_size=10)
           
                       
        # Initialize arm_group used in move_group
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # get the name of end_effector link
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # set up reference
        self.reference_frame = 'base'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # allow replanning
        self.arm.allow_replanning(True)
        
        # set up the tolerent of position and orientation.
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)
       
        # Allow max acc and vel
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        

        #self.go_home()
        self.home2storage()
        #self.calibrate()
        #gripper_rg6.opengripper_slow()
        #self.storage2home()
        #self.precision_test()

    def go_home(self):
       
        # set up a home pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.2
        target_pose.pose.position.y = -0.7
        target_pose.pose.position.z = 0.7
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, -3.1415)
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

    def home2storage(self):
        # make sure we start from home
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.2
        target_pose.pose.position.y = -0.7
        target_pose.pose.position.z = 0.7
        quaternion = tf.transformations.quaternion_from_euler(0, 1.5707, -3.1415)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)

        #first way point
        target_pose.pose.position.x -= 0.5
        target_pose.pose.position.y += 0.5
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)  

        #go down to the target
        target_pose.pose.position.z -= 0.5
        self.arm.set_start_state_to_current_state() 
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)
        rospy.sleep(1)   


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

        
    def aim_target(self):
            

        

    def calibrate(self):

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.7 
        target_pose.pose.position.y = -0.2 
        target_pose.pose.position.z = 0.2
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
        #while (self.object_target.pose.position.x>0.005):
        print(int(self.object_target.pose.position.x*1000), int(self.object_target.pose.position.y*1000))
        target_pose.pose.position.x += self.object_target.pose.position.x
        target_pose.pose.position.y += self.object_target.pose.position.y

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


        





        

if __name__ == "__main__":
    operation = MoveItik()


