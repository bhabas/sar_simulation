#!/usr/bin/env python3
import rospy

## EXAMPLE VIDEOS (!!!MUST WATCH!!!!)
# https://www.youtube.com/watch?v=I_5leJK8vhQ
# https://www.youtube.com/watch?v=ZPmlFNb7v4Y
# https://www.youtube.com/watch?v=OFFzmVz800k


from gazebo_communication_pkg.msg import GlobalState # Custom message format
from std_msgs.msg import Header # Standard format for header
from gazebo_msgs.srv import GetLinkStateRequest,GetLinkState # Message formats for service request and return
# Note: Find message formats with 'rqt -s rqt_msg'

def global_state_publisher():
    ## INIT NODE AND PUBLISHER
    rospy.init_node('global_state_pub', anonymous=False)
    pub = rospy.Publisher('/global_state', GlobalState, queue_size=10)
    rate = rospy.Rate(500,reset=True) # This runs at 100hz in simulation time


    ## WAIT AND INIT SERVICE REQUEST
    rospy.wait_for_service('/gazebo/get_link_state')
    get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)

    ## INIT MSG FORMATS
    state_msg = GlobalState() # state_msg is in format of GlobalState.msg
    header = Header() # header is in format of std_msgs/Header
    header.frame_id='Gazebo_GlobalState'

    link_msg = GetLinkStateRequest() 
    h_ceiling = 5.0



    while not rospy.is_shutdown():
        ## DEFINE STATE_MSG HEADER
        header.stamp = rospy.Time.now()
        state_msg.header = header

        ## DEFINE STATE_MSG POSE/TWIST FROM BASE_LINK USING SERVICE
        link_msg.link_name = 'crazyflie_body'
        result = get_link_srv(link_msg) # Use service to get pose and twist from base_link
        state_msg.global_pose = result.link_state.pose
        state_msg.global_twist = result.link_state.twist

        ## DEFINE STATE_MSG OF VALUES
        d = h_ceiling - result.link_state.pose.position.z
        
        state_msg.OF_x = -result.link_state.twist.linear.y/d # OF_x = -Vy/d
        state_msg.OF_y = -result.link_state.twist.linear.x/d # OF_y = -Vx/d
        state_msg.RREV = result.link_state.twist.linear.z/d  # RREV =  Vz/d


        pub.publish(state_msg)
        rate.sleep() # Dynamic sleep to match Rate [100Hz]

if __name__ == '__main__':
    try:
        global_state_publisher()
    except rospy.ROSInterruptException:
        pass

