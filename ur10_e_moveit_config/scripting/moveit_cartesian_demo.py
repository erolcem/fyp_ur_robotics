#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        # Initialize API of the move_group
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize ROS's node
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # Using cartisian reference
        cartesian = rospy.get_param('~cartesian', True)
                      
        # Initialize the arm group in move group
        arm = MoveGroupCommander('manipulator')
        
        # Allow replanning
        arm.allow_replanning(True)
        
        # set origin of the reference
        arm.set_pose_reference_frame('base_link')
                
        # set the tolerance of position and orientation
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # set max acceleration scaling_factor
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # get the name of end effector link
        end_effector_link = arm.get_end_effector_link()

        # control the arm back to 'home'
        #arm.set_named_target('home')
        #arm.go()
        #rospy.sleep(1)
                                               
        # set current pose as the start pose
        start_pose = arm.get_current_pose(end_effector_link).pose
                
        # initialize set of waypoints
        waypoints = []

        # add start pose in to the waypoint
        if cartesian:
            waypoints.append(start_pose)
            
        # set the waypoint and add in the list
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.2

        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)

        wpose.position.x += 0.15

        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
        
        wpose.position.y += 0.1

        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)

        wpose.position.x -= 0.15
        wpose.position.y -= 0.1
        wpose.position.z += 0.2

        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)

        if cartesian:
		fraction = 0.0   #fraction
		maxtries = 100   #maximum attempting in planning number
		attempts = 0     #attempted number
		
		# set current state as a start state
		arm.set_start_state_to_current_state()
	 
		# try to build a line of waypoints in cartesian
		while fraction < 1.0 and attempts < maxtries:
		    (plan, fraction) = arm.compute_cartesian_path (
		                            waypoints,   # waypoint poses
		                            0.01,        # eef_step
		                            0.0,         # jump_threshold
		                            True)        # avoid_collisions
		    
		    # number of attempting
		    attempts += 1
		    
		    # printing the process
		    if attempts % 10 == 0:
		        rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
		             
		# If fraction =1.0 then start to move the arm
		if fraction == 1.0:
		    rospy.loginfo("Path computed successfully. Moving the arm.")
		    arm.execute(plan)
		    rospy.loginfo("Path execution complete.")
		# if failed print the status
		else:
		    rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

		rospy.sleep(1)

        # move the arm back to home
        #arm.set_named_target('home')
        #arm.go()
        #rospy.sleep(1)
        
        # close moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
