#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


import threading
import time,os,getpass
import os
from scipy.spatial.transform import Rotation


from crazyflie_env import CrazyflieEnv


os.system("clear")
username = getpass.getuser()

# ============================
##     Sim Initialization 
# ============================


## Initialize the environment
env = CrazyflieEnv()
print("Environment done")

state = env.reset_pos()
t_step = 0
start_time_rolloout = env.getTime()
done = False


def cmd_send():
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

input_thread = threading.Thread(target=cmd_send,args=())
input_thread.start()




size = 200
buffer = np.arange(size)/100
y_vec1 = np.zeros(len(buffer))
y_vec2 = np.zeros(len(buffer))
line1 = []
line2 = []

while True:

            
    time.sleep(5e-4) # Time step size

    ## Define current state
    state = env.state_current
    print(state)

    t = state[0]
    pos = state[1:4]
    orientation_q = state[4:8]
    vel = state[8:11]
    vx,vy,vz = vel
    omega = state[11:14]
    


    qw,qx,qy,qz = orientation_q
    # R = Rotation.from_quat([qx,qy,qz,qw])
    # yaw,roll,pitch = R.as_euler('zxy',degrees=True)

    

    # ============================
    ##      Control Profiles
    # ============================


    # if (3.0 <= t <= 3.1): 
    #     action = {'type':'pos', 'x':0.5, 'y':0.0, 'z':1.5, 'ctrl_flag':1}
    #     env.step(action)


    # if (3.0 <= t):
    #     # turn on attitude control and pause sim
    #     action = {'type':'att', 'x':0.0, 'y':0.0, 'z':0.0, 'ctrl_flag':1} 
    #     env.step(action) # Set 5 deg pitch
   



    # ============================
    ##          Errors  
    # ============================
    ## If nan is found in state vector repeat sim run
    if any(np.isnan(state)): # gazebo sim becomes unstable, relaunch simulation
        print("NAN found in state vector")
        error_str = "Error: NAN found in state vector"
        repeat_run = True
        break

    # elif env.timeout():
    #     print("Controller reciever thread timed out")
    #     error_str = "Error: Controller Timeout"
    #     repeat_run = True
    #     break

    t_step += 1





        







