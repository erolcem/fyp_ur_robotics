#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose


class MoveItIkDemo:
    def __init__(self):
        #  Initialize API of the move_group 
        moveit_commander.roscpp_initialize(sys.argv)
        # Initialize ROS nodes
        rospy.init_node('moveit_ik_demo')
                
        # Initialize arm_group used in move_group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # get the name of end_effector link
        end_effector_link = arm.get_end_effector_link()
                        
        # set up reference
        reference_frame = 'fixed_base'
        arm.set_pose_reference_frame(reference_frame)
                
        # allow replanning
        arm.allow_replanning(True)
        
        # set up the tolerent of position and orientation.
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
       
        # Allow max acc and vel
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # control the arm back to home
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
               
        # set up a target position
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.9
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0
        
        # set the current set as start state
        arm.set_start_state_to_current_state()
        
        # set up the target pose of end effector 
        arm.set_pose_target(target_pose, end_effector_link)
        
        # trajectory
        traj = arm.plan()
        
        # execute following the plan
        arm.execute(traj)
        rospy.sleep(1)

        # return home
        arm.set_named_target('home')
        arm.go()

        # exit moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
