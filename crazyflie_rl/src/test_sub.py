import rospy
from crazyflie_msgs.msg import ImpactData,CtrlData,PadConnect
import numpy as np
import math

rospy.init_node("crazyflie_env_node") 



def ctrlCallback(ctrl_msg): ## Callback to parse data received from controller
        
        ## SET & TRIM CTRL VALUES FROM CTRL_DATA TOPIC

        FM = np.asarray(ctrl_msg.FM)   # Force/Moments [N,N*mm]
        FM = np.round(FM,3)       # Round data for logging

        if math.isnan(FM[0]):
            print(FM)

        


ctrl_Subscriber = rospy.Subscriber('/ctrl_data',CtrlData,ctrlCallback,queue_size=1)    

rospy.spin()


