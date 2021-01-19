#!/usr/bin/env python3



import threading,os
from crazyflie_env import CrazyflieEnv
import rospy




os.system("clear")

       


def main():
    
    # ============================
    ##     Sim Initialization 
    # ============================

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv()
    cmd_thread = threading.Thread(target=env.cmd_send,args=())
    cmd_thread.start()   

    rospy.spin()
        

   
       


if __name__ == '__main__':
    

    ## START MAIN SCRIPT
    main()