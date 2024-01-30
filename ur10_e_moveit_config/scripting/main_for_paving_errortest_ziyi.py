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
import math 

#home = [-0.03,0.7,-0.01,0,0,0,0]


class MoveItik(object):
    def __init__(self):
        #  Initialize API of the move_group 
        #moveit_commander.roscpp_initialize(sys.argv)
        self.object_target = PoseStamped()
        self.to_link = '/base'
        self.rotangle = 0.0


       
        gripper_rg6 = rg6()

        # Initialize ROS nodes
        rospy.init_node('moveit_ik_demo')
        self.tf_listener = tf.TransformListener()
        #sub = rospy.Subscriber("image_recognition", PoseStamped, self.callback, queue_size=10)
        # sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback, queue_size=1)
                               
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
        self.arm.set_goal_position_tolerance(0.00005)
        self.arm.set_goal_orientation_tolerance(0.0001)
       
        # Allow max acc and vel
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        
        rospy.sleep(3)
        

    def go_home(self):
       
        # set up a home pose
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        print(current_pose)
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

    def back_home(self):
       
        # set up a home pose
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.5
        target_pose.pose.position.y = -0.7
        target_pose.pose.position.z = 0
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

    def go_target(self):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = '/camera_link'
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = self.object_target.pose.position.x
        target_pose.pose.position.y = self.object_target.pose.position.y
        target_pose.pose.position.z = self.object_target.pose.position.z
        target_pose.pose.orientation.x = self.object_target.pose.orientation.x
        target_pose.pose.orientation.y = self.object_target.pose.orientation.y
        target_pose.pose.orientation.z = self.object_target.pose.orientation.z
        target_pose.pose.orientation.w = self.object_target.pose.orientation.w
        target_pose.header.stamp = rospy.Time.now()
        t = rospy.Time(0)
        self.tf_listener.waitForTransform(self.to_link,self.from_link,t,rospy.Duration(5))
        if self.tf_listener.canTransform(self.to_link,self.from_link,t):
            self.tf_listener.waitForTransform(self.to_link,self.from_link,rospy.Time.now(),rospy.Duration(5))
            target_pose_transferred = self.tf_listener.transformPose(self.to_link,target_pose)
            target_pose_transferred.pose.position.z = -0.1
            quaternion = tf.transformations.quaternion_from_euler(3.1415, 0, 0)
            target_pose_transferred.pose.orientation.x = quaternion[0]
            target_pose_transferred.pose.orientation.y = quaternion[1]
            target_pose_transferred.pose.orientation.z = quaternion[2]
            target_pose_transferred.pose.orientation.w = quaternion[3]
            print(target_pose_transferred)

        else:
            rospy.logerr('Transformation is not possible!')
            sys.exit(0)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose_transferred, self.end_effector_link)   #try to centralize the camera just a test
        traj = self.arm.plan()
        self.arm.execute(traj)
        rospy.sleep(1)

    def go_storage(self):
        # make sure we start from home
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = -0.2
        target_pose.pose.position.y = -0.8
        target_pose.pose.position.z = 0.2
        quaternion = tf.transformations.quaternion_from_euler(0, -2.0944, 0)
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
        target_pose.pose.position.z = 0.1
        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 0)
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

    def readfile(self, position):

        os.system('/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/tutorial-mb-generic-tracker-rgbd-realsense --model_color /home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/model/brick/brickstack/brick-stack.cao')
        file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/objpose.out","r")
        pose = file.readlines()
        position.pose.position.x  = float(pose[0].strip())
        position.pose.position.y = float(pose[1].strip())
        position.pose.position.z = float(pose[2].strip())
        position.pose.orientation.x = float(pose[3].strip())
        position.pose.orientation.y = float(pose[4].strip())
        position.pose.orientation.z = float(pose[5].strip())
        position.pose.orientation.w = float(pose[6].strip())

        file.close()


    def rot_calibrate(self):

        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose = self.arm.get_current_pose()
        print(target_pose)

        # quaternion_list=[target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w]
        # target_euler = tf.transformations.euler_from_quaternion(quaternion_list)
        target_pose.header.frame_id = '/world'
        target_pose.header.stamp = rospy.Time.now()
       # current_euler = tf.transformations.euler_from_quaternion([0.709575,-0.00838021,0.703532,0.0384338])
       # print("current",current_euler[0],current_euler[1],current_euler[2])
        #euler = tf.transformations.euler_from_quaternion([target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w])

        t = rospy.Time(0)
        #target_pose = self.object_position.pose.pose
        #target_pose.pose.position.x += self.object_position.pose.pose.pose.position.x
       # target_pose.pose.position.y -= self.object_position.pose.pose.pose.position.y
        #arget_pose.pose.position.z = self.object_position.pose.pose.pose.position.z
        
        euler = tf.transformations.euler_from_quaternion([self.object_position.pose.pose.pose.orientation.x,self.object_position.pose.pose.pose.orientation.y,self.object_position.pose.pose.pose.orientation.z,self.object_position.pose.pose.pose.orientation.w])
        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, -euler[2])

        #self.rotangle = -euler[2]-3.1415
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        print(target_pose)


        # self.tf_listener.waitForTransform(self.to_link,self.from_link,t,rospy.Duration(5))
        # if self.tf_listener.canTransform(self.to_link,self.from_link,t):
        #     self.tf_listener.waitForTransform(self.to_link,self.from_link,rospy.Time.now(),rospy.Duration(5))
        #     target_pose_transferred = self.tf_listener.transformPose(self.to_link,target_pose)
        #     target_pose_transferred.pose.position.z = current_pose.pose.position.z


        # else:
        #     rospy.logerr('Transformation is not possible!')
        #     sys.exit(0)
        
    
        #while (self.object_target.pose.position.x>0.005):
        # print(target_pose_transferred)


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

    def camera_gomid(self):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        #current_pose = self.arm.get_current_pose()
        current_pose = self.arm.get_current_pose()
        #quaternion_list=[target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w]
        #target_euler = tf.transformations.euler_from_quaternion(quaternion_list)
        target_pose.header.frame_id = '/camera_link'
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = self.object_position.pose.pose.pose.position.x
        target_pose.pose.position.y = self.object_position.pose.pose.pose.position.y
        target_pose.pose.position.z = self.object_position.pose.pose.pose.position.z
        target_pose.pose.orientation.x = self.object_position.pose.pose.pose.orientation.x
        target_pose.pose.orientation.y = self.object_position.pose.pose.pose.orientation.y
        target_pose.pose.orientation.z = self.object_position.pose.pose.pose.orientation.z
        target_pose.pose.orientation.w = self.object_position.pose.pose.pose.orientation.w

        # euler = tf.transformations.euler_from_quaternion([target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w])
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, -euler[2])
        # target_pose.pose.orientation.x = quaternion[0]
        # target_pose.pose.orientation.y = quaternion[1]
        # target_pose.pose.orientation.z = quaternion[2]
        # target_pose.pose.orientation.w = quaternion[3]

        target_pose.header.stamp = rospy.Time.now()
        t = rospy.Time(0)
        self.tf_listener.waitForTransform(self.to_link,self.from_link,t,rospy.Duration(5))
        if self.tf_listener.canTransform(self.to_link,self.from_link,t):
            self.tf_listener.waitForTransform(self.to_link,self.from_link,rospy.Time.now(),rospy.Duration(5))
            target_pose_transferred = self.tf_listener.transformPose(self.to_link,target_pose)
            target_pose_transferred.pose.position.z = current_pose.pose.position.z
            # euler = tf.transformations.euler_from_quaternion([target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w])
            # quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, -euler[2]-1.5707)
            # target_pose_transferred.pose.orientation.x = quaternion[0]
            # target_pose_transferred.pose.orientation.y = quaternion[1]
            # target_pose_transferred.pose.orientation.z = quaternion[2]
            # target_pose_transferred.pose.orientation.w = quaternion[3]
            target_pose_transferred.pose.orientation.x = current_pose.pose.orientation.x
            target_pose_transferred.pose.orientation.y = current_pose.pose.orientation.y
            target_pose_transferred.pose.orientation.z = current_pose.pose.orientation.z
            target_pose_transferred.pose.orientation.w = current_pose.pose.orientation.w
            



        else:
            rospy.logerr('Transformation is not possible!')
            sys.exit(0)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose_transferred, self.end_effector_link)   #try to centralize the camera just a test
        traj = self.arm.plan()
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

    def test(self):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        print(current_pose)
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = current_pose.pose.position.x+0.01000
        target_pose.pose.position.y = current_pose.pose.position.y
        target_pose.pose.position.z = current_pose.pose.position.z
        target_pose.pose.orientation.x = current_pose.pose.orientation.x
        target_pose.pose.orientation.y = current_pose.pose.orientation.y
        target_pose.pose.orientation.z = current_pose.pose.orientation.z
        target_pose.pose.orientation.w = current_pose.pose.orientation.w
        target_pose.header.frame_id = "world"
        print(target_pose)

        self.arm.set_start_state_to_current_state()
        
        # set up the target pose of end effector 
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # trajectory
        traj = self.arm.plan()
        
        #execute following the plan
        self.arm.execute(traj)
        rospy.sleep(1)

        target_pose.pose.position.x = target_pose.pose.position.x-0.01000

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # trajectory
        traj = self.arm.plan()
        
        #execute following the plan
        self.arm.execute(traj)
        rospy.sleep(1)

n_z = 1
n_x = 2
n_y = 2
brick_z = 0.04
brick_x = 0.2
brick_y = 0.1
i = 0
j = 0
k = 0

cmd = '/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/tutorial-mb-generic-tracker-rgbd-realsense --model_color /home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/model/brick/brickstack/brick-stack.cao'

def print_cao(n_y,n_x,n_z,a,b,c,file):
    file.write("V1"+'\n'+'################################################'+'\n')
    for l in range(n_z-c):
        if (l == 0):
            for m in range (n_x-b):
                if(m == 0):
                    for n in range (n_y-a):
                        file.write('load("../brickstack/brick.cao", t=['+str(m*brick_x)+';'+ str(n*brick_y)+';'+str(l*-1*brick_z)+'])'+'\n')
                else:
                    for n in range (n_y):
                        file.write('load("../brickstack/brick.cao", t=['+str((m-b)*brick_x)+';'+ str((n-a)*brick_y)+';'+str(l*-1*brick_z)+'])'+'\n')
        else:
            for m in range (n_x):
                for n in range (n_y):
                    file.write('load("../brickstack/brick.cao", t=['+str((m-b)*brick_x)+';'+ str((n-a)*brick_y)+';'+str(l*-1*brick_z)+'])'+'\n')
    file.write('################################################'+'\n')
    file.write('# 3D points\n0                    # No 3D points\n# 3D lines\n0                    # No 3D lines\n# 3D faces from lines\n0                    # No 3D faces from lines\n# 3D faces from points\n0                    # No 3D faces from points\n# 3D cylinders\n0                    # No 3D cylinders\n# 3D circle\n0                    # No 3D circle')


if __name__ == "__main__":
    operation = MoveItik()
    for k in range (n_z):
        for j in range (n_x):
            for i in range (n_y):
                operation.back_home()
                operation.go_home()
                operation.go_storage()
                if(i==0 and j==0 and k==0):
                    file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/objinitpose.out","r")
                    pose = file.readlines()
                    xcam_brick = float(pose[0].strip())
                    ycam_brick = float(pose[1].strip())
                    zcam_brick = float(pose[2].strip())
                    rxcam_brick = float(pose[3].strip())
                    rycam_brick = float(pose[4].strip())
                    rzcam_brick = float(pose[5].strip())
                    orixcam_brick = float(pose[6].strip())
                    oriycam_brick = float(pose[7].strip())
                    orizcam_brick = float(pose[8].strip())

                    file.close()
                    file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/model/brick/brickstack/brick-stack.0.pos","w")
                    file.write(str(float(xcam_brick))+'\n'+str(float(ycam_brick))+'\n'+str(float(zcam_brick))+'\n'+str(orixcam_brick)+'\n'+str(oriycam_brick)+'\n'+str(orizcam_brick))
                    file.close()

                    file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/model/brick/brickstack/brick-stack.cao","w")
                    print_cao(n_y,n_x,n_z,i,j,k,file)
                    file.close()
                    operation.readfile(operation.object_target)

                else:
                    file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/objpose.out","r")
                    pose = file.readlines()
                    xcam_brick = float(pose[0].strip())
                    ycam_brick = float(pose[1].strip())
                    zcam_brick = float(pose[2].strip())
                    rxcam_brick = float(pose[3].strip())
                    rycam_brick = float(pose[4].strip())
                    rzcam_brick = float(pose[5].strip())
                    orixcam_brick = float(pose[6].strip())
                    oriycam_brick = float(pose[7].strip())
                    orizcam_brick = float(pose[8].strip())

                    file.close()

                    cx = math.cos(rxcam_brick)
                    sx = math.sin(rxcam_brick)
                    cy = math.cos(rycam_brick)
                    sy = math.sin(rycam_brick)
                    cz = math.cos(rzcam_brick)
                    sz = math.sin(rzcam_brick)
                    Rx = np.matrix([[1,0,0],[0,cx,sx],[0,-sx,cx]])
                    Ry = np.matrix([[cy,0,sy],[0,1,0],[sy,0,cy]])           
                    Rz = np.matrix([[cz,sz,0],[-sz,cz,0],[0,0,1]])
                    if (i==0):
                        if (j==0):
                            t_in_brick = [[-(n_x-1)*brick_x],[-(n_y-1)*brick_y],[-brick_z]]
                        else:
                            t_in_brick = [[brick_x],[-(n_y-1)*brick_y],[0.0000]]
                    else:
                        t_in_brick = [[0.0000],[brick_y],[0.0000]]

                    t_brick_to_cam = [[xcam_brick],[ycam_brick],[zcam_brick]]
                    transformed_pose = Rz*Ry*Rx*t_in_brick+t_brick_to_cam

                    file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/model/brick/brickstack/brick-stack.0.pos","w")
                    file.write(str(float(transformed_pose[0]))+'\n'+str(float(transformed_pose[1]))+'\n'+str(float(transformed_pose[2]))+'\n'+str(orixcam_brick)+'\n'+str(oriycam_brick)+'\n'+str(orizcam_brick))
                    file.close()

                    file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/model/brick/brickstack/brick-stack.cao","w")
                    print_cao(n_y,n_x,n_z,i,j,k,file)
                    file.close()
                    operation.readfile(operation.object_target)

                operation.go_target()

                if(i==0 and j==0 and k==0):
                    os.system("cp /home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/objpose.out /home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/objinitpose.out")
