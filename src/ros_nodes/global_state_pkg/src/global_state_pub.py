#!/usr/bin/env python3

from genpy import message
import rospy

from global_state_node.msg import GlobalState
from std_msgs.msg import Header

rospy.init_node('global_state_node')
pub = rospy.Publisher('/global_state',GlobalState, queue_size=10)
rate = rospy.Rate(100) # Publish at 100 Hz


message = GlobalState()
message.custom_msg = "Hello World"

while not rospy.is_shutdown():
    pub.publish(message)
    rate.sleep()