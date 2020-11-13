#! /usr/bin/env python3

import rospy

## IMPORT MSG FORMATS
from nav_msgs.msg import Odometry # Import odometry msg format we'll be using
from std_msgs.msg import Header # Import header msg format 
from gazebo_msgs.srv import GetModelState # Type of msg received from service?
    # found looking up service w/ "rosservice info /gazebo/get_model_state"
from gazebo_msgs.srv import GetModelStateRequest # Type of msg sent to service
    # found looking up service w/ "rqt -s rqt_service"


## INIT NODE AND PUBLISHER
rospy.init_node('odom_pub')
odom_pub = rospy.Publisher('/odom',Odometry,queue_size=10) #Publish topic= '/odom', msg=Odometry
r = rospy.Rate(100) # Publish every 100Hz

## INIT SERVICE LOCATION AND WAIT FOR SERVICE 
rospy.wait_for_service('/gazebo/get_model_state') # Wait here till service is ready
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState) # service call and msg type received


## INIT EMPTY MSG'S TO BE FILLED
odom = Odometry() # Empty msg that we'll fill w/ data
header = Header() # Empty and will contain time stamp and seq
header.frame_id = '/odom' # Fill in frame_id w/ static string


## INIT MSG TO BE SENT TO SERVICE
model = GetModelStateRequest()
model.model_name = 'crazyflie_landing_gears' # Fill in msg fieldS that we'll send to the Service


while not rospy.is_shutdown(): # run till ROS is shutdown 

    result = get_model_srv(model) # Call to service and return answer

    # Take pose and twist and put into Odometry msg
    odom.pose.pose = result.pose # There are two layers of pose in Odometry msg (check rqt -s rqt_msg)
    odom.twist.twist = result.twist

    # Fill header (time)stamp with current time from ROS
    header.stamp = rospy.Time.now()
    odom.header = header # Insert header into odom.header  

    odom_pub.publish(odom) # Publish create odom message 

    r.sleep() 





