#!/usr/bin/env python  
import roslib
roslib.load_manifest('ur10_e_moveit_config')
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class TF_publish(object):
    def __init__(self):
        rospy.init_node('tf_broadcaster')
        rate = rospy.Rate(10.0)
        target = PoseStamped()
        listener = tf.TransformListener()
        broadcaster = tf.TransformBroadcaster()
        pub = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            try:
                (translation,rotation) = listener.lookupTransform('camera_link', 'ee_link', rospy.Time(0))
                pub.sendTransform((translation[0], translation[1], translation[2]),(rotation[0], rotation[1], rotation[2], rotation[3]),rospy.Time.now(),"object","camera_link")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        rospy.spin()
if __name__ == '__main__':
    operation = TF_publish()