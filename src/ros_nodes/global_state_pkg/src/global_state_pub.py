#!/usr/bin/env python3
import rospy


from global_state_pkg.msg import GlobalState # Custom message format
from std_msgs.msg import Header

from gazebo_msgs.srv import GetLinkStateRequest,GetLinkState # Message formats for service request and return

## INIT NODE AND PUBLISHER
rospy.init_node('global_state_pub', anonymous=True)
pub = rospy.Publisher('/global_state', GlobalState, queue_size=10)
rate = rospy.Rate(100) # 100hz This runs at 100hz in simulation time


## WAIT AND INIT SERVICE REQUEST
rospy.wait_for_service('/gazebo/get_link_state')
get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)

## INIT 
state_msg = GlobalState()

header = Header()
header.frame_id='Gazebo_Global_State'

link = GetLinkStateRequest()

rotors = ['rotor_1','rotor_2','rotor_3','rotor_4']


while not rospy.is_shutdown():
    header.stamp = rospy.Time.now()
    state_msg.header = header

    link.link_name = 'base_link'
    result = get_link_srv(link)
    state_msg.global_pose = result.link_state.pose
    state_msg.global_twist = result.link_state.twist

    motorspeed = []
    for rotor in rotors:
        link.link_name = rotor
        result = get_link_srv(link)
        ms = result.link_state.twist.angular.z
        motorspeed.append(abs(ms)*10)
    state_msg.motorspeeds = motorspeed
    

    rospy.loginfo(state_msg)
    pub.publish(state_msg)
    rate.sleep()

