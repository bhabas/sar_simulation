import rospy
import numpy as np

from sensor_msgs.msg import Image

def customFunc(x): #function for parsing the camera data
    return x

def Callback(msg):
    # val = customFunc(msg.header.stamp.secs,msg.header.stamp.nsecs)
    val = customFunc(msg.data)
    print(val)


def listener():

    rospy.init_node('Camera_Data', anonymous = True)#start
    sub = rospy.Subscriber("cf/camera/image_raw",Image,Callback,queue_size=1)

    rospy.spin() #run until this program is shutdown