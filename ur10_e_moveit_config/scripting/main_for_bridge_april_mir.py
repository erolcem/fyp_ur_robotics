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
from webclick_initialize import fwebdriver
import timeit

#home = [-0.03,0.7,-0.01,0,0,0,0]


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


        self.webdriver = fwebdriver()
        
        rospy.sleep(1)
        self.CC_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component_CC.out","w")
        self.MC_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component_MC.out","w")
        self.time_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/time.out","w")
        self.output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/output.out","w")
        self.output_angle = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/output_angle.out","w")
        t0 = timeit.default_timer()
        # self.observe_MC_front(30)
        # self.mir_calibrate_tag(30)
        # self.CC2()
        t1 = timeit.default_timer()
        # self.webdriver.edit_and_run(0.609, 0, 0)
        # self.ref_pos_output.write("================ MC1 ================="+"\n")
        # self.grab_pos_output.write("================ MC1 ================="+"\n")
        # self.ref_rot_output.write("================ MC1 ================="+"\n")
        t2 = timeit.default_timer()
        self.observe_MC_front(30)
        self.mir_calibrate_tag(30)
        self.MC1()
        # self.ref_pos_output.write("================ CC3 ================="+"\n")
        # self.grab_pos_output.write("================ CC3 ================="+"\n")
        # self.ref_rot_output.write("================ CC3 ================="+"\n")
        # self.observe_MC_front(31)
        # self.mir_calibrate_tag(31)
        # self.CC3()
        t3 = timeit.default_timer()
        delta_t1 = t1-t0
        delta_t2 = t3-t2
        # self.time_output.write(str(delta_t1)+"\t"+str(delta_t2))
        # self.time_output.close()
        # self.CC_output.close() 
        # self.MC_output.close()



        # self.test(49)
        # self.go_home()
        # self.mir_error()
        # self.go_inspect()
        # self.mir_error_mid()
        # self.mir_error_side()
# 
    def mir_error_mid(self):
        self.go_inspect()
        self.mir_calibrate_tag(31)
        self.output_angle.write("\n")
        self.output.write("\n")
        # self.go_inspect()
        self.go_home()
        self.webdriver.edit_and_run(1, 0, 0)
        rospy.sleep(8)
        self.go_inspect()
        # self.aim_tag(103)
        self.mir_calibrate_tag(32)
        self.output_angle.write("\n")
        self.output.write("\n")
        # self.go_inspect()
        self.go_home()
        self.webdriver.edit_and_run(-1, 0, 0)
        rospy.sleep(8)
        self.go_inspect()
        self.mir_calibrate_tag(31)
        self.output_angle.write("\n")
        self.output.write("\n")
        self.output.close()
        self.output_angle.close()

    def mir_error_side(self):
        self.go_inspect()
        self.mir_calibrate_tag(31)
        self.output_angle.write("\n")
        self.output.write("\n")
        self.go_inspect()
        # self.go_home()
        self.webdriver.edit_and_run(1, 0, 0)
        rospy.sleep(8)
        # self.go_inspect()
        # self.aim_tag(103)
        self.mir_calibrate_tag(32)
        self.output_angle.write("\n")
        self.output.write("\n")
        self.go_inspect()
        # self.go_home()
        self.webdriver.edit_and_run(-1, 0, 0)
        rospy.sleep(8)
        self.go_inspect()
        self.mir_calibrate_tag(31)
        self.output_angle.write("\n")
        self.output.write("\n")
        self.output.close()
        self.output_angle.close()
        

    def aim_tag(self,id):
        while self.cal == "false":
            self.count = self.count+1
            self.change_component(id,0.133,'Ref2')
            # self.change_component(id,0.02,'Ref2')
            os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
            rospy.sleep(2)
            while (abs(self.object_target.pose.pose.pose.position.x)>0.0005 or abs(self.object_target.pose.pose.pose.position.y) > 0.0005):
                print("targeting...")
                self.calibrate()
            else:
                print("target is aimed")
                self.record_position()
                rospy.sleep(5)


                os.system('pkill apriltag_ros_co')
            
            file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/angle.out","w")
            file.write(str(self.euler_angle[2]))
            file.close()
            self.cal = "true"
    def go_inspect(self):
       
        # set up a home pose
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 1.1
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.4
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

    def record_position(self):
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        self.output.write(str(current_pose.pose.position.x)+"\t"+ str(current_pose.pose.position.y) +"\n")

    def mir_calibrate_tag(self,id):
        
        while self.cal == "false":
            self.count = self.count+1
            # self.change_component(id,0.133,'Ref2')
            self.change_component(id,0.002,'Ref2')
            os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
            rospy.sleep(2)
            while (abs(self.object_target.pose.pose.pose.position.x)>0.0005 or abs(self.object_target.pose.pose.pose.position.y) > 0.0005):
                print("targeting...")
                self.calibrate()
            else:
                print("target is aimed")
                self.record_position()
                rospy.sleep(5)


                os.system('pkill apriltag_ros_co')
            
            file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/angle.out","w")
            file.write(str(self.euler_angle[2]))
            file.close()
            file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/angle.out","r")
            rot = file.readlines()
            file.close()
            r = float(rot[0].strip())
            r = self.euler_angle[2]
            r = r*180/3.1415
            self.output_angle.write(str(r)+"\n")
            if abs(r)<0.25:
                self.cal = "true"
                print("done with mir calibration")                
            else:
                if (r < 0):
                    r  = -r +5

                    self.webdriver.edit_and_run(0, 0, r)
                    r = -5
                    self.webdriver.edit_and_run(0, 0, r)
                
                if (r > 0):
                    r = -r -5
                    self.webdriver.edit_and_run(0, 0, r)
                    r = 5
                    self.webdriver.edit_and_run(0, 0, r)
                
        self.cal = "false"


    def test(self,id):
        i =0
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.0002 or abs(self.object_target.pose.pose.pose.position.y) > 0.0002):
            print("targeting...")
            i=i+1
            
            self.calibrate()
        else:
            print("target is aimed and takes ")
            print(str(i)+" times"+"\n")
            current_pose = self.arm.get_current_pose()
            print(str(current_pose.pose.position.x)+"\t"+ str(current_pose.pose.position.y) +"\n")
            os.system('pkill apriltag_ros_co')

    def calibrate_mir(self,id):
        
        while self.cal == "false":
            self.count = self.count+1
            self.change_component(id,0.02,'Ref2')
            os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
            rospy.sleep(2)
            while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
                print("targeting...")
                self.calibrate()
            else:
                print("target is aimed")
                # self.ref_pos_output.write(str(self.pos_x)+", "+ str(self.pos_y) +"\n")
                os.system('pkill apriltag_ros_co')

            os.system('gnome-terminal -e "rosrun ur10_e_moveit_config target_recog_V3.py"')
            rospy.sleep(10)

            file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/angle.out","r")
            rot = file.readlines()
            file.close()
            r = float(rot[0].strip())
            
            print(abs(r),self.cal)
            if abs(r)<0.35 or abs(90-abs(r))<0.35:
                self.cal = "true"
                self.ref_pos_output.write(str(self.pos_x)+", "+ str(self.pos_y)+ "\n")
                self.ref_rot_output.write(str(r))
                self.ref_rot_output.write("\n")

                
            else:
                if (r < 0 and r > -45):
                    print(r)
                    r  = -r +5
                    self.ref_rot_output.write(str(r-5))
                    self.ref_rot_output.write("\n")
                    self.webdriver.edit_and_run(0, 0, r)
                    r = -5
                    self.webdriver.edit_and_run(0, 0, r)
                
                if (r > 0 and r < 45):
                    print(r)
                    r = r +5
                    self.ref_rot_output.write(str(r-5))
                    self.ref_rot_output.write("\n")
                    self.webdriver.edit_and_run(0, 0, r)
                    r = -5
                    self.webdriver.edit_and_run(0, 0, r)

                if r < -45:
                    print(r)
                    r = -90 - r -5
                    self.ref_rot_output.write(str(r+5))
                    self.ref_rot_output.write("\n")
                    self.webdriver.edit_and_run(0, 0, r)
                    r = 5
                    self.webdriver.edit_and_run(0, 0, r)

                if r > 45:
                    print(r)
                    r = -90 + r -5
                    self.ref_rot_output.write(str(r+5))
                    self.ref_rot_output.write("\n")
                    self.webdriver.edit_and_run(0, 0, r)
                    r = 5
                    self.webdriver.edit_and_run(0, 0, r)

                
        self.cal = "false"
            

    def observe_MC_back(self,id):
        self.go_home()
        self.go_reference_CC()
        self.go_reference_MC()
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')

    def observe_MC_front(self,id):
        self.go_home()
        self.go_reference_CC()
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')

        
    def CC1(self):
        
        self.go_home()
        self.go_reference_CC()
        self.change_component(30,0.02,'Ref1')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')


        # self.origin_position = self.save_reference()
        # self.go_home()
        # self.go_storage(1.3,0)
        # self.look_for_component()
        # self.change_component(20,0.02,'CC1')
        # os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        # rospy.sleep(2)
        # while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
        #     print("targeting...")
        #     self.calibrate()
        # else:
        #     print("target is aimed")
        #     os.system('pkill apriltag_ros_co')
        # self.pickup()
        # self.gripper_rg6.closegripper_fast()
        # rospy.sleep(1)
        # self.go_storage(1.3,0)
        # self.go_installing()
        # self.install_CC1(0.4712)
        # self.insert_CC1(0.4712)
        # self.gripper_rg6.opengripper_fast()
        # self.back_from_installation()
        # self.go_home()

    def CC2(self):


        self.change_component(30,0.02,'Ref1')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            current_pose = PoseStamped()
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.CC_output.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")
            os.system('pkill apriltag_ros_co')


        self.origin_position = self.save_reference()
        self.go_home()
        self.go_storage(1.3,0)
        self.look_for_component()
        self.change_component(21,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.CC_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            os.system('pkill apriltag_ros_co')
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.3,0)
        # self.go_installing_cc()
        self.install_CC(0.4712,0.315,0.285,0.210,0.225)
        self.insert_CC(0.4712,0.315,0.285,0.210,0.225,0.07)
        self.write_down(self.CC_output)
        self.gripper_rg6.opengripper_fast()
        self.go_reference_CC()
        self.go_home()

    def write_down(self,output_file):
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        self.pos_x = current_pose.pose.position.x
        self.pos_y = current_pose.pose.position.y
        output_file.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")    

    def MC1(self):

        self.change_component(30,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            current_pose = PoseStamped()
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.MC_output.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")            
            os.system('pkill apriltag_ros_co')
        self.origin_position = self.save_reference()
        self.go_reference_CC()
        self.go_home()
        self.go_storage(1.3,0)
        self.look_for_component()
        self.change_component(32,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.MC_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            os.system('pkill apriltag_ros_co')
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.3,0)
                # self.install_MC(0,0.1,0.4,0.335)
        # self.insert_MC(0.2792, 0, 0.075,0.41,0.49,0.65,0.24)
        self.install_MC(0,0.105,0.4,0.335)
        self.insert_MC(0.2792, 0, 0.07,0.41,0.49,0.65,0.24) #self.origin_position.pose.position.z-z
        self.write_down(self.MC_output)
        self.gripper_rg6.opengripper_fast()
        self.go_reference_CC()

    def CC3(self):

        self.change_component(31,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.origin_position = self.save_reference()
        self.go_home()
        self.go_storage(1.4,0)
        self.look_for_component()
        self.change_component(22,0.02,'CC3')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            self.grab_pos_output.write(str(self.pos_x)+", "+ str(self.pos_y))
            self.grab_pos_output.write("\n")
            os.system('pkill apriltag_ros_co')
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.4,0)
        self.install_CC(0.2792,0.265,0.330,0.240,0.225)
        self.insert_CC(0.2792,0.265,0.330,0.240,0.225,0.090)
        self.gripper_rg6.opengripper_fast()
        self.go_reference_CC()
        

    def install_MC(self,pre_angle,x,y,z):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x-x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y
        target_pose.pose.position.z = self.origin_position.pose.position.z-z


        quaternion = tf.transformations.quaternion_from_euler(pre_angle, -3.1415, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        print(target_pose)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1) 

        target_pose.pose.position.x = self.origin_position.pose.position.x-x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y
        target_pose.pose.position.z = self.origin_position.pose.position.z-z-0.05


        quaternion = tf.transformations.quaternion_from_euler(pre_angle, -3.1415, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        print(target_pose)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)  

    def insert_MC(self,angle,pre_angle,x,y,z,r1,r2):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()
        # y_displacement = np.sin((angle+pre_angle)/2+np.arcsin(0.2/np.sqrt(0.2*0.2+(r1+r2)*(r1+r2))))*2*np.sin((angle+pre_angle)/2)*np.sqrt(0.2*0.2+(r1+r2)*(r1+r2))
        y_displacement = (r1+r2)*np.sin(angle+pre_angle)*np.tan((angle+pre_angle)/2)
        # z_displacement = np.cos((angle+pre_angle)/2+np.arcsin(0.2/np.sqrt(0.2*0.2+(r1+r2)*(r1+r2))))*2*np.sin((angle+pre_angle)/2)*np.sqrt(0.2*0.2+(r1+r2)*(r1+r2))
        z_displacement = (r1+r2)*np.sin(angle+pre_angle)

        target_pose.pose.position.x = self.origin_position.pose.position.x-x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y-y_displacement
        target_pose.pose.position.z = self.origin_position.pose.position.z-z+z_displacement

        quaternion = tf.transformations.quaternion_from_euler(-angle, -3.1415, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        print(target_pose)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)



    def go_reference_MC(self):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.45
        target_pose.pose.position.y = 0.609
        target_pose.pose.position.z = 0.6
        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        print("go_reference_CC")
        print(target_pose)

        self.arm.set_start_state_to_current_state()
        
        # set up the target pose of end effector 
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # trajectory
        traj = self.arm.plan()
        
        #execute following the plan
        self.arm.execute(traj)
        rospy.sleep(1)


    def insert_CC(self,angle,x,y,z,prepare_distance, insert_distance):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x-x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y-prepare_distance*np.cos(angle)-insert_distance*np.sin(angle)
        target_pose.pose.position.z = self.origin_position.pose.position.z-z-prepare_distance*np.sin(angle)+insert_distance*np.cos(angle)


        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415-angle, 1.57075)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1) 
    def install_CC(self,angle,x,y,z,prepare_distance):
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x-x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y
        target_pose.pose.position.z = self.origin_position.pose.position.z-z


        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 1.57075)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        print(target_pose)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1) 

        target_pose.pose.position.x = self.origin_position.pose.position.x-x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y-prepare_distance*np.cos(angle)
        target_pose.pose.position.z = self.origin_position.pose.position.z-z-prepare_distance*np.sin(angle)


        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415-angle, 1.57075)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1) 

    def go_installing_cc(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x-0.315
        target_pose.pose.position.y = self.origin_position.pose.position.y+0.31
        target_pose.pose.position.z = self.origin_position.pose.position.z
        print("go_installing_cc")

        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 1.57075)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        print(target_pose)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1) 
        print("end_cc")

    def go_pickup(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x+1.7
        target_pose.pose.position.y = self.origin_position.pose.position.y+0.450
        target_pose.pose.position.z = self.origin_position.pose.position.z-0.37
        target_pose.pose.orientation.x = self.origin_position.pose.orientation.x
        target_pose.pose.orientation.y = self.origin_position.pose.orientation.y
        target_pose.pose.orientation.z = self.origin_position.pose.orientation.z
        target_pose.pose.orientation.w = self.origin_position.pose.orientation.w
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)

    def reference_to_home(self):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = -0.3
        target_pose.pose.position.z = 0.6
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

        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = -0.7
        target_pose.pose.position.z = 0.6
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

    def go_reference(self):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.730
        target_pose.pose.position.y = 0.130
        target_pose.pose.position.z = 0.370
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

    def back_from_installation(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x-0.340
        target_pose.pose.position.y = self.origin_position.pose.position.y-0.615
        target_pose.pose.position.z = self.origin_position.pose.position.z
        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 1.57075)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)  

        target_pose.pose.position.x = self.origin_position.pose.position.x-0.1
        target_pose.pose.position.y = self.origin_position.pose.position.y-0.2
        target_pose.pose.position.z = self.origin_position.pose.position.z
        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 1.57075)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)  

        

    def insert_CC1(self,angle):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x-0.340
        target_pose.pose.position.y = self.origin_position.pose.position.y-0.65+0.115*np.cos(angle)
        target_pose.pose.position.z = self.origin_position.pose.position.z-0.385-0.115*np.sin(angle)
        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415-angle, 1.57075)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)          #self.object_position = self.object_target

    def install_CC1(self,angle):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x-0.340
        target_pose.pose.position.y = self.origin_position.pose.position.y-0.65
        target_pose.pose.position.z = self.origin_position.pose.position.z-0.385
        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415-angle, 1.57075)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)          #self.object_position = self.object_target



    def save_reference(self):
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        reference_coordinate = current_pose
        return reference_coordinate

        

    def go_reference_CC(self):

        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.45
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.6
        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        print("go_reference_CC")
        print(target_pose)

        self.arm.set_start_state_to_current_state()
        
        # set up the target pose of end effector 
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # trajectory
        traj = self.arm.plan()
        
        #execute following the plan
        self.arm.execute(traj)
        rospy.sleep(1)

    def go_home_test(self):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0
        target_pose.pose.position.y = -0.5
        target_pose.pose.position.z = 0.4
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

    def go_storage(self,x,y):
        # make sure we start from home
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x+x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y
        target_pose.pose.position.z = self.origin_position.pose.position.z
        target_pose.pose.orientation.x = self.origin_position.pose.orientation.x
        target_pose.pose.orientation.y = self.origin_position.pose.orientation.y
        target_pose.pose.orientation.z = self.origin_position.pose.orientation.z
        target_pose.pose.orientation.w = self.origin_position.pose.orientation.w
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)          #self.object_position = self.object_target

    def look_for_component(self):
        
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = current_pose.pose.position.x
        target_pose.pose.position.y = current_pose.pose.position.y
        target_pose.pose.position.z = current_pose.pose.position.z - 0.6
        target_pose.pose.orientation.x = current_pose.pose.orientation.x
        target_pose.pose.orientation.y = current_pose.pose.orientation.y
        target_pose.pose.orientation.z = current_pose.pose.orientation.z
        target_pose.pose.orientation.w = current_pose.pose.orientation.w
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)          #self.object_position = self.object_target


    def go_installing(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x
        target_pose.pose.position.y = self.origin_position.pose.position.y
        target_pose.pose.position.z = self.origin_position.pose.position.z


        quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 1.57075)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)  

    def callback(self, position):
        target_pose = PoseStamped()
        self.object_position = position
        self.object_target = self.object_position.detections[0]
        target_pose.pose.orientation.x = self.object_target.pose.pose.pose.orientation.x
        target_pose.pose.orientation.y = self.object_target.pose.pose.pose.orientation.y
        target_pose.pose.orientation.z = self.object_target.pose.pose.pose.orientation.z
        target_pose.pose.orientation.w = self.object_target.pose.pose.pose.orientation.w
        self.euler_angle = tf.transformations.euler_from_quaternion([target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w])
       
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
            target_pose_transferred.pose.position.z = current_pose.pose.position.z
            quaternion = tf.transformations.quaternion_from_euler(3.1415, 0, 0)
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


    def storage2home(self):

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = -0.7
        target_pose.pose.position.y = -0.2
        target_pose.pose.position.z = 0.7
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
  

    def pickup(self):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = 'camera_link'
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = self.object_target.pose.pose.pose.position.x - 0.032
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
            target_pose_transferred.pose.position.z = current_pose.pose.position.z - 0.140
            quaternion = tf.transformations.quaternion_from_euler(3.1415, 0, 0)
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
        rospy.sleep(2)


    def change_component(self,tag_number,size,name):
        file = open("/home/cheavporchea/apriltag/src/apriltag_ros/apriltag_ros/config/tags.yaml","w")
        file.write("standalone_tags:"+"\n"+"  [{id: "+str(tag_number)+", size: "+str(size)+", name: "+str(name)+"}]")
        file.close()   






        

if __name__ == "__main__":
    operation = MoveItik()


