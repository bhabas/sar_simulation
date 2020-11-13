#!/usr/bin/env python3
import rospy
from mypkg.msg import mycustom
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelStateRequest,GetModelState

def talker():
    rospy.init_node('my_publisher', anonymous=True)
    pub = rospy.Publisher('my_topic', mycustom, queue_size=10)

    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)

    rate = rospy.Rate(10) # 10hz
    msg = mycustom()

    header = Header()
    header.frame_id='test'

    model = GetModelStateRequest()
    model.model_name = 'crazyflie_landing_gears'
    

    while not rospy.is_shutdown():
        result = get_model_srv(model)
        

        header.stamp = rospy.Time.now()
        msg.header = header
        msg.direction.x = 1
        msg.direction.y = 2
        msg.direction.z = 3

        msg.normal = result.pose
        msg.normal2 = result.twist
        

        # msg.normal.position.x = 3

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass