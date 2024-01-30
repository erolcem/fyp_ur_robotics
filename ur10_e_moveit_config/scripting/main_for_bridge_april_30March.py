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
from webclick_initialize import fwebdriver
import timeit
import socket

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
        self.depth = 0.0
        self.depthbridge = CvBridge()
        self.ip_robotA = "192.168.1.100"
        self.ip_robotB = "192.168.1.101"

        # Initialize ROS nodes
        rospy.init_node('moveit_ik_demo')
        self.tf_listener = tf.TransformListener()
        #sub = rospy.Subscriber("image_recognition", PoseStamped, self.callback, queue_size=10)
        sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback, queue_size=1)
        sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callback_depth, queue_size=1)
                               
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
        self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","w")
        self.component_output.write("")
        self.component_output.close()
        self.print_marker_position = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/maker.out","w")
        self.time_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/time.out","w")
        t0 = timeit.default_timer()
        
        self.time_output.close()
        # self.go_home()
        # self.observe_reference_point(0.8,0,0.6)
        # self.waitingfortag_move(30,0.8,0,0.6)
        # self.mir_calibrate_tag(30)
        # self.go_home()
        # self.webdriver.edit_and_run(0.609, 0, 0)
        # self.test_on_marker_installed(33,0.8,0,0.6)
        # self.go_home()
        # self.webdriver.edit_and_run(-0.609, 0, 0)



        
        # self.test_on_marker_installed(34,0.8,0,0.6)
        # self.test_on_marker_installing_nomoving(34)
        # self.test_on_marker_installing_nomoving(30)
        # self.test_on_marker_installing(34)

        self.waitingfortag_move(30,0.8,0,0.6)
        self.mir_calibrate_tag(30)
        self.CC2()

        self.waitingfortag_move(30,0.8,0,0.6)
        self.mir_calibrate_tag(30)
        self.MC1()
        # t1 = timeit.default_timer()
        # self.webdriver.edit_and_run(0.609, 0, 0)
        # # t2 = timeit.default_timer()
        # self.waitingfortag_move(31,0.8,0,0.6)
        # self.mir_calibrate_tag(31)
        # self.CC3()
        # t3 = timeit.default_timer()
        # delta_t1 = t1-t0
        # delta_t2 = t3-t2
        # self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
        # self.component_output.write(str(delta_t1)+"\t"+str(delta_t2))
        # self.component_output.close() 
        # self.go_home()
        # self.waitingfortag_move(31,0.8,0,0.8)
        # self.mir_calibrate_tag(31)
        # self.MC2()
        # self.webdriver.edit_and_run(0.670, 0, 0)
        # self.waitingfortag_move(33,0.7,0,0.8)
        # self.mir_calibrate_tag(33)
        # self.CC4()
        # self.waitingfortag_move(33,0.6,0,0.85)
        # self.mir_calibrate_tag(33)
        # self.MC3()

        # self.webdriver.edit_and_run(0.550, 0, 0)
        # self.waitingfortag_move(35,0.8,0,0.8)
        # self.mir_calibrate_tag(35)
        # self.CC5()
        # self.waitingfortag_move(35,0.8,0,0.9)
        # self.mir_calibrate_tag(35)
        # self.MC4()

        # self.webdriver.edit_and_run(0.550, 0, 0)
        # self.waitingfortag_move(37,0.8,0,0.8)
        # self.mir_calibrate_tag(37)
        # self.CC6()
        # self.waitingfortag_move(37,0.8,0,0.7)
        # self.mir_calibrate_tag(37)
        # self.MC5()

    def test_on_marker_installed(self,id,x,y,z):
        self.observe_reference_point(x,y,z)
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.0002 or abs(self.object_target.pose.pose.pose.position.y) > 0.0002):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
                # self.record_position()
            rospy.sleep(5)
        i=0
        self.go_closer()
        self.calibrate()
        print(str(self.euler_angle[2]*180/3.14))
        os.system('pkill apriltag_ros_co')


    def test_on_marker_installed_nomoving(self,id):

        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        i=0
        while i<30:
            self.print_marker_position.write(str(self.object_target.pose.pose.pose.position.x)+" "+str(self.object_target.pose.pose.pose.position.y)+" "+str(self.object_target.pose.pose.pose.position.z)+" "+str(self.euler_angle[0])+" "+str(self.euler_angle[1])+" "+str(self.euler_angle[2])+"\n")
            rospy.sleep(3)
            i=i+1
        os.system('pkill apriltag_ros_co')

    def test_on_marker_installing(self,id):
        rospy.sleep(5)
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.0002 or abs(self.object_target.pose.pose.pose.position.y) > 0.0002):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
                # self.record_position()
            rospy.sleep(5)
        i=0
        self.go_closer()
        self.calibrate()
        rospy.sleep(5)
        while i<30:
            self.print_marker_position.write(str(self.object_target.pose.pose.pose.position.x)+" "+str(self.object_target.pose.pose.pose.position.y)+" "+str(self.object_target.pose.pose.pose.position.z)+" "+str(self.euler_angle[0])+" "+str(self.euler_angle[1])+" "+str(self.euler_angle[2])+"\n")
            rospy.sleep(3)
            i=i+1
        os.system('pkill apriltag_ros_co')

    def test_on_marker_installing_nomoving(self,id):
        rospy.sleep(5)
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        i=0
        rospy.sleep(5)
        while i<30:
            self.print_marker_position.write(str(self.object_target.pose.pose.pose.position.x)+" "+str(self.object_target.pose.pose.pose.position.y)+" "+str(self.object_target.pose.pose.pose.position.z)+" "+str(self.euler_angle[0])+" "+str(self.euler_angle[1])+" "+str(self.euler_angle[2])+"\n")
            rospy.sleep(3)
            i=i+1
        os.system('pkill apriltag_ros_co')
        
    def mir_error_mid(self):
        self.go_inspect()
        self.mir_calibrate_tag(102)
        self.output_angle.write("\n")
        self.output.write("\n")
        self.go_home()
        self.webdriver.edit_and_run(1, 0, 0)
        rospy.sleep(5)
        # self.go_inspect()
        # self.mir_calibrate_tag(103)
        # self.output_angle.write("\n")
        # self.output.write("\n")
        # self.go_home()
        self.webdriver.edit_and_run(-1, 0, 0)
        rospy.sleep(15)
        self.go_inspect()
        self.mir_calibrate_tag(102)
        self.output_angle.write("\n")
        self.output.write("\n")
        self.output.close()
        self.output_angle.close()

    def waitingfortag_move(self,id,x,y,z):
        self.observe_reference_point(x,y,z)
        # self.go_reference_MC()
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while self.object_target.id[0] != id:
            print("seeking for tag")
            self.move_up()
            rospy.sleep(5)
        else:
            print("cross component is found!")
            os.system('pkill apriltag_ros_co')
        os.system('gnome-terminal -e "rosrun"')

    def move_up(self):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose = current_pose
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.z = target_pose.pose.position.z + 0.1

        self.arm.set_start_state_to_current_state()
        
        # set up the target pose of end effector 
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # trajectory
        traj = self.arm.plan()
        
        #execute following the plan
        self.arm.execute(traj)
        rospy.sleep(1)
        
    def go_inspect(self):
       
        # set up a home pose
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.8
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.5
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
    def talktorobot(self,ip):
        serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        serv.bind((ip, 8080))
        serv.listen(5)

        status = True
        while status == True:
            conn, addr = serv.accept()
            from_client = ''

            while True:
                data = conn.recv(4096)
                if not data: break
                from_client += data
                print from_client
                conn.send("I am Server\n")

            conn.close()
            status = False
    def listentorobot(self,ip):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s=client.connect_ex((ip,8080))
        while s != 0:
            s=client.connect_ex((ip,8080))

        client.send("I am CLIENT\n")

        from_server = client.recv(4096)
            

        client.close()
    def callback_depth(self, ros_image):
        try:
            depth_image = self.depthbridge.imgmsg_to_cv2(ros_image, "32FC1")
        except CvBridgeError, e:
            print e
        depth_array = np.array(depth_image)
        # print(depth_array)
        self.depth = depth_array[240][320]
        self.depth1 = np.shape(depth_array)
    def record_position(self):
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        self.output.write(str(current_pose.pose.position.x)+"\t"+ str(current_pose.pose.position.y) +"\n")
    def mir_calibrate_tag(self,id):
        
        while self.cal == "false":
            self.count = self.count+1
            # self.change_component(id,0.133,'Ref2')
            self.change_component(id,0.02,'Ref2')
            os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
            rospy.sleep(2)
            while (abs(self.object_target.pose.pose.pose.position.x)>0.0002 or abs(self.object_target.pose.pose.pose.position.y) > 0.0002):
                print("targeting...")
                self.calibrate()
            else:
                print("target is aimed")
                # self.record_position()
                rospy.sleep(5)


                os.system('pkill apriltag_ros_co')
            
            # file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/angle.out","w")
            # file.write(str(self.euler_angle[2]))
            # file.close()
            # file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/angle.out","r")
            # rot = file.readlines()
            # file.close()
            # r = float(rot[0].strip())
            r = self.euler_angle[2]
            r = r*180/3.1415
            # self.output_angle.write(str(r)+"\n")
            if abs(r)<0.35:
                print(r)
                self.cal = "true"
                print("done with mir calibration")                
            else:
                # print(r)
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
    def test(self):

        self.talktorobot(self.ip_robotA)
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
            if abs(r)<0.4 or abs(90-abs(r))<0.4:
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
    def observe_MC_front_higher(self,id):

        self.go_reference_CC_higher()
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')  
    def observe_MC_front_higher_higher(self,id):

        self.go_reference_CC_higher_higher()
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co') 
    def observe_MC_front_higher_higher_higher(self,id):

        self.go_reference_CC_higher_higher_higher()
        self.change_component(id,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co') 

    def observe_reference_point(self,x,y,z):
        
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
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
    def save_data(self,name):
        file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/height/"+name+".txt","w")
        file.write(str(self.depth))
        file.close()
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
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a") 
            self.component_output.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')


        self.origin_position = self.save_reference()
        self.save_data('cc2')
        self.go_home()
        self.go_storage(1.4,0,0.7)
        self.change_component(21,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(21,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.3,0, 0.7)
        self.install_CC(0.4712,0.205,0.28,0.210-0.445,0.225)
        self.insert_CC(0.4712,0.205,0.28,0.210-0.445,0.225,0.075)
        self.write_down(self.component_output)
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
            current_pose = PoseStamped()
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output =  open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')


        self.origin_position = self.save_reference()
        self.save_data('cc3')
        self.go_home()
        self.go_storage(1.6,0,0.7)
        self.change_component(22,0.02,'CC3')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(22,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)


        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output =  open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.6,0,0.7)
        self.install_CC(0.2792,0.120,0.320,0.230-0.406,0.225)
        self.insert_CC(0.2792,0.120,0.320,0.230-0.406,0.225,0.075)
        
        # self.install_CC(0.2792,0.265,0.320,0.230,0.225)
        # self.insert_CC(0.2792,0.265,0.320,0.230,0.225,0.070)
        # self.write_down(self.component_output)
        # self.raise_up(0.150, 1.96, 0.526)
        # self.listentorobot(self.ip_robotB)
        # rospy.sleep(3)
        self.gripper_rg6.opengripper_fast()
        self.go_reference_CC_higher()
        self.go_home()
    def CC4(self):

        self.change_component(33,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')

        self.origin_position = self.save_reference()
        self.save_data('cc4')
        self.go_home()
        self.go_storage(1.6,0,0.7)
        self.change_component(23,0.02,'CC4')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(23,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)


        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output =  open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.6,0,0.7)
        self.install_CC(0,0.225,0.305,0.28-0.427,0.16)
        self.insert_CC(0,0.225,0.305,0.28-0.427,0.16,0.04)
        # self.talktorobot(self.ip_robotA)
        # self.listentorobot(self.ip_robotB)
        # self.raise_up(0.150, 1.96, 0.526)

        self.gripper_rg6.opengripper_fast()
        self.go_reference_CC_higher()
        self.go_home()
    def CC5(self):

        self.change_component(35,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')

        self.origin_position = self.save_reference()
        self.save_data('cc5')
        self.go_home()
        self.go_storage(1.5,0.2,0.7)
        self.change_component(24,0.02,'CC5')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(24,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)


        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output =  open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.5,0.2,0.7)
        self.install_CC(-0.1,0.175,0.305,0.425-0.514,0.15)
        self.insert_CC(-0.1,0.190,0.305,0.425-0.514,0.15,0.06)
        self.gripper_rg6.opengripper_fast()
        self.go_reference_CC_higher()
        self.go_home()
    def CC6(self):

        self.change_component(37,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')

        self.origin_position = self.save_reference()
        self.save_data('cc6')
        self.go_home()
        self.go_storage(1.6,0,0.7)
        self.change_component(26,0.02,'CC6')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(26,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)


        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output =  open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.6,0,0.7)
        self.install_CC(-0.35,0.12,0.295,0.48-0.525,0.12)
        self.insert_CC(-0.35,0.12,0.295,0.48-0.525,0.12,0.045)
        self.gripper_rg6.opengripper_fast()
        self.go_reference_CC_higher()
        self.go_home()
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
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")
            self.component_output.close()            
            os.system('pkill apriltag_ros_co')
        self.origin_position = self.save_reference()
        self.save_data('mc1')
        self.go_reference_CC()
        self.go_home()
        self.go_storage(1.6,0,0.7)
        self.change_component(32,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(32,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)


        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            # euler_angle = tf.transformations.euler_from_quaternion([self.object_target.pose.pose.pose.orientation.x,self.object_target.pose.pose.pose.orientation.y,self.object_target.pose.pose.pose.orientation.z,self.object_target.pose.pose.pose.orientation.w])
            # print(euler_angle)
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.6,0,0.7)
        self.preinstall_MC(0,0.105,0.7,0.345-0.444)
        self.install_MC(0,0.105,0.4,0.315-0.444,0.05)
        self.insert_MC(0.1703, 0, 0.07,0.4,0.315-0.444,0.05,0.65,0.24) #self.origin_position.pose.position.z-z
        self.write_down(self.component_output)
        self.gripper_rg6.opengripper_fast()
        self.go_reference_CC()
        self.go_home()
    def MC2(self):

        self.change_component(31,0.02,'Ref2')
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
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")
            self.component_output.close()            
            os.system('pkill apriltag_ros_co')
        self.origin_position = self.save_reference()
        self.save_data('mc2')
        self.go_reference_CC_higher()
        self.go_home()
        self.go_storage(1.6,0,0.7)
        self.change_component(34,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(34,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)


        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.6,0,0.7)
        self.preinstall_MC(0.2792,-0.085,0.8,0.65-0.41)
        self.install_MC(0.2792,-0.085,0.440,0.45-0.41,0.05)
        self.insert_MC(0.04, -0.2792, -0.1,0.44,0.45-0.41,0.05,0.65,0.38) #self.origin_position.pose.position.z-z
        self.gripper_rg6.opengripper_fast()
        self.move_up()
        self.go_home()
    def MC3(self):

        self.change_component(33,0.02,'Ref2')
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
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")
            self.component_output.close()            
            os.system('pkill apriltag_ros_co')
        self.origin_position = self.save_reference()
        self.save_data('mc3')
        self.go_reference_CC_higher_higher()
        self.go_home()
        self.go_storage(1.6,0,0.7)
        self.change_component(36,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(36,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)


        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.6,0,0.7)
        self.preinstall_MC(0.35,0.08,0.8,0.65-0.424)
        self.install_MC(0.35,0.08,0.425,0.375-0.424,0.1)
        self.insert_MC(-0.16, -0.35, 0.075,0.425,0.375-0.424,0.1,0.65,0.38) #self.origin_position.pose.position.z-z
        self.gripper_rg6.opengripper_fast()
        self.go_reference_CC_higher_higher()
        self.go_home()
    def MC4(self):
        
        self.change_component(35,0.02,'Ref2')
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
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")
            self.component_output.close()            
            os.system('pkill apriltag_ros_co')
        self.origin_position = self.save_reference()
        self.save_data('mc4')
        self.go_reference_CC_higher_higher()
        self.go_home()
        self.go_storage(1.5,0.2,0.7)
        self.change_component(38,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(38,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)


        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.4,0.2,0.7)
        self.preinstall_MC(0.45,0.08,0.8,0.75-0.393)
        self.install_MC(0.45,0.08,0.43,0.41-0.393,0.05)
        self.insert_MC(-0.33, -0.45, 0.075,0.43,0.41-0.393,0.05,0.65,0.38) #self.origin_position.pose.position.z-z
        self.gripper_rg6.opengripper_fast()
        self.move_up()
        self.go_home()
    def MC5(self):
        
        self.change_component(37,0.02,'Ref2')
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
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")
            self.component_output.close()            
            os.system('pkill apriltag_ros_co')
        self.origin_position = self.save_reference()
        self.save_data('mc5')
        self.go_reference_CC_higher_higher()
        self.go_home()
        self.go_storage(1.6,0,0.7)
        self.change_component(46,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001):
            print("targeting...")
            self.calibrate()
        else:
            print("target is aimed")
            os.system('pkill apriltag_ros_co')
        self.look_for_component()
        self.change_component(46,0.02,'Ref2')
        os.system('gnome-terminal -e "roslaunch apriltag_ros continuous_detection.launch"')
        rospy.sleep(2)


        self.angle_z = 0
        self.angle_x = 0
        self.angle_y = 0
        while (abs(self.object_target.pose.pose.pose.position.x)>0.001 or abs(self.object_target.pose.pose.pose.position.y) > 0.001 or abs(-1*self.euler_angle[2]-1.57) > 0.05):
            print("targeting...")
            self.advance_calibrate()
        else:
            print("target is aimed")
            current_pose = self.arm.get_current_pose()
            self.pos_x = current_pose.pose.position.x
            self.pos_y = current_pose.pose.position.y
            self.component_output = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
            self.component_output.write(str(self.pos_x)+"\t"+ str(self.pos_y)+"\t\t")
            self.component_output.close()
            os.system('pkill apriltag_ros_co')
        
        self.pickup()
        self.gripper_rg6.closegripper_fast()
        rospy.sleep(1)
        self.go_storage(1.6,0,0.7)
        self.preinstall_MC(0.35,-0.14,0.8,0.82-0.555)
        self.install_MC(0.7,-0.075,0.39,0.67-0.555,0.05)
        self.insert_MC(-0.55, -0.7, -0.09,0.39,0.69-0.563,0.05,0.65,0.38) #self.origin_position.pose.position.z-z
        self.gripper_rg6.opengripper_fast()
        # self.go_reference_CC_higher_higher()
        self.go_home()
    def rotate(self):
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
            # self.angle_y = self.angle_y - self.euler_angle[1]
            angle_z = 5*3.1415/180


            quaternion = tf.transformations.quaternion_from_euler(3.1415, 0, angle_z)
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
        rospy.sleep(5)
    def advance_calibrate(self):
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
            # self.angle_y = self.angle_y - self.euler_angle[1]
            self.angle_z = self.angle_z+ (-1*self.euler_angle[2]-1.57)


            quaternion = tf.transformations.quaternion_from_euler(3.1415, 0, self.angle_z)
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
        rospy.sleep(1)
    def write_down(self,output_file):
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        self.pos_x = current_pose.pose.position.x
        self.pos_y = current_pose.pose.position.y
        output_file = open("/home/cheavporchea/ur10e_paving/src/ur10_e_moveit_config/scripting/Component.out","a")
        output_file.write(str(self.pos_x)+"\t"+str(self.pos_y)+"\t\t")
        output_file.close()      
    def install_MC(self,pre_angle,x,y,z,z_place):
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
        target_pose.pose.position.z = self.origin_position.pose.position.z-z-z_place


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
    def insert_MC(self,angle,pre_angle,x,y,z,z_place,r1,r2):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()
        # y_displacement = np.sin((angle+pre_angle)/2+np.arcsin(0.2/np.sqrt(0.2*0.2+(r1+r2)*(r1+r2))))*2*np.sin((angle+pre_angle)/2)*np.sqrt(0.2*0.2+(r1+r2)*(r1+r2))
        # z_displacement = np.cos((angle+pre_angle)/2+np.arcsin(0.2/np.sqrt(0.2*0.2+(r1+r2)*(r1+r2))))*2*np.sin((angle+pre_angle)/2)*np.sqrt(0.2*0.2+(r1+r2)*(r1+r2))

        y_displacement = np.sqrt(0.25*0.25+(r1+r2-0.15)*(r1+r2-0.15))*(np.cos(angle+np.arctan(0.25/(r1+r2-0.15)))-np.cos(pre_angle+np.arctan(0.25/(r1+r2-0.15))))
        z_displacement = np.sqrt(0.25*0.25+(r1+r2-0.15)*(r1+r2-0.15))*(np.sin(angle+np.arctan(0.25/(r1+r2-0.15)))-np.sin(pre_angle+np.arctan(0.25/(r1+r2-0.15))))
        print(y_displacement)
        print("\n")
        print(z_displacement)
        target_pose.pose.position.x = self.origin_position.pose.position.x-x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y+y_displacement
        target_pose.pose.position.z = self.origin_position.pose.position.z-z-z_place+z_displacement

        quaternion = tf.transformations.quaternion_from_euler(-angle, -3.1415, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
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
        target_pose.pose.position.x = 0.6
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
    def preinstall_MC(self,angle,x,y,z):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x-x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y
        target_pose.pose.position.z = self.origin_position.pose.position.z-z


        quaternion = tf.transformations.quaternion_from_euler(angle, -3.1415, 0)
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
    def raise_up(self,z,y0,z0):
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()
        pre_angle = np.arctan(z0/y0)
        angle = np.arcsin((z0+z)/np.sqrt(z0*z0+y0*y0))
        y = np.sqrt(z0*z0+y0+y0)*(np.cos(angle)-np.cos(pre_angle))
        target_pose.pose.position.x = current_pose.pose.position.x
        target_pose.pose.position.y = current_pose.pose.position.y + y
        target_pose.pose.position.z = current_pose.pose.position.z + z
        
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
        reference_coordinate.pose.position.z = reference_coordinate.pose.position.z-self.depth/1000
        return reference_coordinate

    def go_observe(self,x,y,z):

        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
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
    def go_reference_CC(self):

        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.8
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
    def go_reference_CC_higher_higher(self):

        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.8
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.85
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
    def go_reference_CC_higher_higher_higher(self):

        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.8
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.9
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
    def go_reference_CC_higher(self):

        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.8
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.8
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

        # quaternion = tf.transformations.quaternion_from_euler(-angle, -3.1415, 0)
        # target_pose.pose.orientation.x = quaternion[0]
        # target_pose.pose.orientation.y = quaternion[1]
        # target_pose.pose.orientation.z = quaternion[2]
        # target_pose.pose.orientation.w = quaternion[3]
        
        # self.arm.set_start_state_to_current_state()
        # self.arm.set_pose_target(target_pose, self.end_effector_link)
        # traj = self.arm.plan()
        # self.arm.execute(traj)     
        # rospy.sleep(1)
    def go_home_test(self):
       
        # set up a home pose
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.2
        target_pose.pose.position.y = -0.7
        target_pose.pose.position.z = 0.7
        # quaternion = tf.transformations.quaternion_from_euler(0, -3.1415, 0)
        # target_pose.pose.orientation.x = quaternion[0]
        # target_pose.pose.orientation.y = quaternion[1]
        # target_pose.pose.orientation.z = quaternion[2]
        # target_pose.pose.orientation.w = quaternion[3]

        # self.arm.set_start_state_to_current_state()
        
        # # set up the target pose of end effector 
        # self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # # trajectory
        # traj = self.arm.plan()
        
        # #execute following the plan
        # self.arm.execute(traj)
        # rospy.sleep(1)

        quaternion = tf.transformations.quaternion_from_euler(1.57, -3.1415, 0)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)     
        rospy.sleep(1)
    def go_storage(self,x,y,z):
        # make sure we start from home
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        #first way point
        target_pose.pose.position.x = self.origin_position.pose.position.x+x
        target_pose.pose.position.y = self.origin_position.pose.position.y+y
        target_pose.pose.position.z = z
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
    def go_closer(self):
        
        target_pose = PoseStamped()
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = current_pose.pose.position.x
        target_pose.pose.position.y = current_pose.pose.position.y
        target_pose.pose.position.z = current_pose.pose.position.z - self.depth/1000+0.250
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
            target_pose_transferred.pose.position.z = current_pose.pose.position.z - self.depth/1000 + 0.195
            quaternion = tf.transformations.quaternion_from_euler(3.1415, 0, self.angle_z)
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


