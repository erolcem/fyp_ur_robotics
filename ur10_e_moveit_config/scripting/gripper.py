#!/usr/bin/env python
import rospy
import sys
from ur_msgs.srv import SetIO

class rg6(object):
	def __init__(self):
		#rospy.init_node('call_gripper')
		rospy.wait_for_service('/ur_hardware_interface/set_io')
		self.set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)

	def opengripper_fast(self):
		try:
			mode = self.set_io(1,17,0)
		except rospy.ServiceException, e:
			print "Service call failed: %" %e
		try:
			action = self.set_io(1,16,0)
		except rospy.ServiceException, e:
			print "Service call failed: %" %e

	def closegripper_fast(self):
		try:
			mode = self.set_io(1,17,0)
		except rospy.ServiceException, e:
			print "Service call failed: %" %e
		try:
			action = self.set_io(1,16,1)
		except rospy.ServiceException, e:
			print "Service call failed: %" %e

	def opengripper_slow(self):
		try:
			mode = self.set_io(1,17,1)
		except rospy.ServiceException, e:
			print "Service call failed: %" %e
		try:
			action = self.set_io(1,16,0)
		except rospy.ServiceException, e:
			print "Service call failed: %" %e

	def closegripper_slow(self):
		try:
			mode = self.set_io(1,17,1)
		except rospy.ServiceException, e:
			print "Service call failed: %" %e
		try:
			action = self.set_io(1,16,1)
		except rospy.ServiceException, e:
			print "Service call failed: %" %e

if __name__ == "__main__":
	gripper = rg6()
	gripper.opengripper_slow()
