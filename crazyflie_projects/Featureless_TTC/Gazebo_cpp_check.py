import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation

from example_msgs.msg import CustomMessage

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160


class CameraClass:

    def __init__(self):

        rospy.init_node('Camera_Data',anonymous=True)

        rospy.Subscriber("/MyPub_cpp",CustomMessage,self.cpp_callback,queue_size=1)
        

    def cpp_callback(self,msg):

        # self.t = np.round(msg.header.stamp.to_sec(),4)
        self.Cur_img = np.frombuffer(msg.Camera_data, np.uint8,).reshape(WIDTH_PIXELS,HEIGHT_PIXELS)
        self.Prev_img = np.frombuffer(msg.Prev_img, np.uint8,).reshape(WIDTH_PIXELS,HEIGHT_PIXELS)
        
        
if __name__ == '__main__':

    CameraClass() #initialize the class when run
    rospy.spin() #run until this program is shutdown
