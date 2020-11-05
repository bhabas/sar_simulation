#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import threading
from multiprocessing import Process,Array,Value
from scipy.spatial.transform import Rotation
import threading


from crazyflie_env import CrazyflieEnv
from utility.dashboard import runGraph


os.system("clear")

def cmd_send(env):
    while True:
        # Converts input number into action name
        cmd_dict = {0:'home',1:'pos',2:'vel',3:'att',4:'omega',5:'stop',6:'gains'}
        val = float(input("\nCmd Type (0:home,1:pos,2:vel,3:att,4:omega,5:stop,6:gains): "))
        action = cmd_dict[val]

        if action=='home' or action == 'stop': # Execute home or stop action
            ctrl_vals = [0,0,0]
            ctrl_flag = 1
            env.step(action,ctrl_vals,ctrl_flag)

        elif action=='gains': # Execture Gain changer
            
            vals = input("\nControl Gains (kp_x,kd_x,kp_R,kd_R): ") # Take in comma-seperated values and convert into list
            vals = [float(i) for i in vals.split(',')]
            ctrl_vals = vals[0:3]
            ctrl_flag = vals[3]

            env.step(action,ctrl_vals,ctrl_flag)
            
        elif action == 'omega': # Execture Angular rate action

            ctrl_vals = input("\nControl Vals (x,y,z): ")
            ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
            ctrl_flag = 1.0

            env.step('omega',ctrl_vals,ctrl_flag)


        else:
            ctrl_vals = input("\nControl Vals (x,y,z): ")
            ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
            ctrl_flag = float(input("\nController On/Off (1,0): "))
            env.step(action,ctrl_vals,ctrl_flag)

   
       
     
## Idea to delete and respawn model instead of restarting Gazebo after collison-crash
# rosservice call /gazebo/delete_model '{model_name: crazyflie_landing_gears}'
# rosrun gazebo_ros spawn_model -file /home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo/models/crazyflie_landing_gears/crazyflie_landing_gears.sdf -sdf -model crazyflie_landing_gears




def main():

    # ============================
    ##     Sim Initialization 
    # ============================

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv()
    print("Environment done")

    ## INIT STATE RECIEVING THREAD
    state_thread = threading.Thread(target=env.get_state,args=(STATE,))
    state_thread.start() # Start thread that continually recieves state array from Gazebo

    cmd_thread = threading.Thread(target=cmd_send,args=(env,))
    cmd_thread.start()


    ## SIM PARAMETERS
    ep_start = 0 # Default episode start position
    h_ceiling = 1.5 # [m]]



    while True:
        
        ## DEFINE CURRENT STATE
        state = np.array(STATE[:])
        position = STATE[1:4] # [x,y,z]
        orientation_q = STATE[4:8] # Orientation in quat format
        vel = STATE[8:11]
        vx,vy,vz = vel
        omega = STATE[11:14]
        d = h_ceiling - position[2] # distance of drone from ceiling

        ## ORIENTATION DATA FROM STATE
        qw,qx,qy,qz = orientation_q
        R = Rotation.from_quat([qx,qy,qz,qw])
        R = R.as_matrix() # [b1,b2,b3] Body vectors

        RREV, OF_y, OF_x = vz/d, vx/d, vy/d # OF_x,y are estimated optical flow vals assuming no body rotation
        sensor_data = [RREV, OF_y, OF_x] # simplified for data recording


if __name__ == '__main__':
    STATE = Array('d',14) # Global state array for Multiprocessing
    REWARD = Value('d',0) 
    REWARD_AVG = Value('d',0)
    K_RUN = Value('i',0)
    K_EP = Value('i',0)
    N_ROLLOUTS = Value('i',0)

    ## START PLOTTING PROCESS
    p1 = Process(target=runGraph,args=(STATE,K_EP,K_RUN,REWARD,REWARD_AVG,N_ROLLOUTS))
    p1.start()

    ## START MAIN SCRIPT
    main()