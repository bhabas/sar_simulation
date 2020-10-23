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
env = CrazyflieEnv(port_RL=18050, port_Ctrl=18060,username =username)
print("Environment done")





state = env.reset()
t_step = 0
start_time_rolloout = env.getTime()
done = False





def live_plotter(x_vec,y1_data,line1,y2_data,line2):
    if line1==[]:
        plt.ion()
        fig, ax1 = plt.subplots()
        
        ax1.grid()
        ax1.set_ylim([-45,90])   
        ax1.set_ylabel("Pitch: [deg]") 
        line1, = ax1.plot(x_vec,y1_data)

        ax2 = ax1.twinx()
        color = 'tab:red'
        ax2.set_ylabel('Pos: Z [m]', color=color)
        ax2.set_ylim([0,1.5]) 
        ax2.tick_params(axis='y', labelcolor=color)
        line2, = ax2.plot(x_vec,y2_data,color = color)

        plt.show()

    line1.set_ydata(y1_data)    
    line2.set_ydata(y2_data)

    plt.pause(0.000001)
    
    return line1,line2




# filename = "src/4. rl/src/log/Gazebo_acc_test.csv"
# with open(filename,mode='w') as csvfile:
#     writer = csv.writer(csvfile,delimiter = ',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#     # writer.writerow(['Time [s]','Omega_x','Omega_y','Omega_z'])


def cmd_send():
    while True:
        cmd_dict = {0:'home',1:'pos',2:'vel',3:'att',4:'omega',5:'stop'}
        val = float(input("\nCmd Type (0:home,1:pos,2:vel,3:att,4:omega,5:stop): "))
        cmd_str = cmd_dict[val]

        if cmd_str=='home' or cmd_str == 'stop':
            ctrl_vals = [0,0,0]
            ctrl_flag = 0
        else:
            try:
                ctrl_vals = input("\nControl Vals (x,y,z): ")
                ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
                if len(ctrl_vals) != 3:
                    raise Exception()
            except:
                print("Error: x,y,z")
            ctrl_flag = float(input("\nController On/Off (1,0): "))

        action = {'type':cmd_str, 'x':ctrl_vals[0], 'y':ctrl_vals[1], 'z':ctrl_vals[2], 'ctrl_flag':ctrl_flag}
        env.step(action)
     



input_thread = threading.Thread(target=cmd_send,args=())
input_thread.start()




size = 200
x_vec = np.arange(size)/100
y_vec1 = np.zeros(len(x_vec))
y_vec2 = np.zeros(len(x_vec))
line1 = []
line2 = []

while True:

    
        
            
    time.sleep(5e-4) # Time step size

    ## Define current state
    state = env.state_current

    pos = state[1:4]
    orientation_q = state[4:8]
    vel = state[8:11]
    vx,vy,vz = vel
    omega = state[11:14]
    t = state[0]


    qw,qx,qy,qz = orientation_q
    R = Rotation.from_quat([qx,qy,qz,qw])
    yaw,roll,pitch = R.as_euler('zxy',degrees=True)

    
    # y_vec1[-1] = pitch
    # y_vec2[-1] = pos[2]
    # line1,line2 = live_plotter(x_vec,y_vec1,line1,y_vec2,line2)
    # y_vec1 = np.append(y_vec1[1:],0.0)
    # y_vec2 = np.append(y_vec2[1:],0.0)



        

    #     with open(filename,mode='a') as csvfile:
    #         writer = csv.writer(csvfile,delimiter = ',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #         writer.writerow([t,pos[2],vz])
        



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





        







