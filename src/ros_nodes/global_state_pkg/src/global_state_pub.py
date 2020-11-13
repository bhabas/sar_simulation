#!/usr/bin/env python3
import rospy
from mypkg.msg import GlobalState
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelStateRequest,GetModelState


rospy.init_node('global_state_pub', anonymous=True)
pub = rospy.Publisher('/global_state', GlobalState, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)

rate = rospy.Rate(100) # 100hz This runs at 100hz in simulation time
msg = GlobalState()

header = Header()
header.frame_id='Gazebo_Global_State'

model = GetModelStateRequest()
model.model_name = 'crazyflie_landing_gears'


while not rospy.is_shutdown():
    result = get_model_srv(model)
    

    header.stamp = rospy.Time.now()
    msg.header = header

    msg.global_pose = result.pose
    msg.global_twist = result.twist
    

    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()

