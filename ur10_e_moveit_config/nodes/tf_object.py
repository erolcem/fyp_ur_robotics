#!/usr/bin/env python  
import roslib
roslib.load_manifest('ur10e_e_moveit_config')
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class TF_publish(object):
    def __init__(self):
        rospy.init_node('turtle_tf_broadcaster')
        sub = rospy.Subscriber("image_recognition", PoseStamped, self.callback, queue_size=10)
        pub = tr.TransformBroadcaster()
        pub.sendTransform((object_target.pose.position.x, object_target.pose.position.y, object_target.pose.position.z),
                        (object_target.pose.rotation.x, object_target.pose.rotation.y, object_target.pose.rotation.z, object_target.pose.rotation.w),rospy.Time.now(),"object","camer_link")
        rospy.spin()
    
    def callback(self, msg):
        self.object_target = msg

if __name__ == '__main__':
operation = TF_publish()