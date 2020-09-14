#!/usr/bin/env python

import rospy 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

## Note: I'm not familiar w/ publishing ROS data but this works
# the timeline it connects with the controller is off though

def gazebo_IC(x=0,y=0,z=0,vx=0,vy=0,vz=0):
    # rospy.init_node('set_pose',anonymous=True)

    state_msg = ModelState()
    state_msg.model_name = 'crazyflie_landing_gears'

    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = z
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0


    state_msg.twist.linear.x = vx
    state_msg.twist.linear.y = vy
    state_msg.twist.linear.z = vz
    state_msg.twist.angular.x = 0
    state_msg.twist.angular.y = 0
    state_msg.twist.angular.z = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException:
        # print("Service call failed: %s" %e)
        pass 

if __name__ == '__main__':
    try:
        gazebo_IC(z = 0.25, vx = 1,vy = 1)
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")