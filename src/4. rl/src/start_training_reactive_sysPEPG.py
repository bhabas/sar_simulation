#!/usr/bin/env python3

import numpy as np
import time,copy,os
import matplotlib.pyplot as plt
from math import sin,cos,pi,sqrt
import os
from scipy.spatial.transform import Rotation
from numpy.core.fromnumeric import repeat

from crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive
from rl_cma import CMA_basic,CMA

'''
mu = [5.267,-10.228,-4.713]
z -> 2.5 3.5 seems to fail at lower vz
x -> -0.5 0.5

'''

'''
## Learning rates and agent
alpha_mu = np.array([[0.1],[0.1] ]) #,[0.8] ,[0.8]])
alpha_sigma = np.array([[0.05],[0.05] ]) #,[0.03],[0.03]])
agent = rlsysPEPGAgent_reactive(alpha_mu=alpha_mu, alpha_sigma=alpha_sigma, gamma=0.95, n_rollout=5)

## Define initial parameters for gaussian function
agent.mu = np.array([[3.5], [-5.0] ])#,[-6.0],[7.0]])   # Initial estimates of mu: size (2 x 1)
agent.sigma = np.array([[2.0], [3.0] ]) #,[0.5],[0.5]])      # Initial estimates of sigma: size (2 x 1)
agent.mu_history = copy.copy(agent.mu)  # Creates another
'''
'''
mu = np.array([[2.0],[-2.0] ])#,[0.0]])   # Initial estimates of mu: 
sigma = np.array([[1.0],[1.0]   ])#,[1.0]])
'''

# Enter username here ********
username = "bader"

## Initialize the environment
env = CrazyflieEnv(port_self=18050, port_remote=18060)
print("Environment done")


## Learning rate
alpha_mu = np.array([[3.0],[2.0] ])#[2.0]] )#,[0.1]])
alpha_sigma = np.array([[2.0],[3.0] ])#, [1.0]])#,[0.05]])

# seems to be unstable if mu is close to zero (coverges to deterinistic)
## Initial parameters for gaussian function
#mu = np.array([[3.0],[-3.0] ])#,[1.5]])   # Initial estimates of mu: 
#sigma = np.array([[1.0],[1.0],[-0.8] ])#, [0.5]])      # Initial estimates of sigma: 
#agent = rlsysPEPGAgent_reactive(alpha_mu, alpha_sigma, mu,sigma, gamma=0.95,n_rollout=7)

#agent = CMA_basic(mu,sigma,N_best=0.3,n_rollout=10)
agent = CMA(n=2)
n_rollout = 6

h_ceiling = 1.5 # meters



start_time = time.strftime('_%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
file_name = '/home/'+username+'/catkin_ws/src/crazyflie_simulation/src/4. rl/src/log/' + username + start_time + '.csv'
#env.create_csv(file_name)

## Initial figure setup
fig = plt.figure()
plt.ion()  # interactive on
plt.grid()
plt.xlim([-1,80])
plt.ylim([-1,25])
plt.xlabel("Episode")
plt.ylabel("Reward")
plt.title("Episode: %d Run: %d" %(0,0))
plt.show() 

# Variables for current and previous image from camera sensor
image_prev = np.array(0)
image_now = np.array(0)

# ============================
##          Episode 
# ============================
for k_ep in range(1000):

    # os.system("python3 start_training_reactive_sysPEPG > output.txt")

    print("=============================================")
    print("STARTING Episode # %d" %k_ep)
    print("=============================================")

    print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
    #mu = agent.mu
    #sigma = agent.sigma
    #print(mu)
    #print(sigma)

    mu = agent.xmean
    sigma = np.array([agent.C[0,0],agent.C[1,1],agent.C[0,1]])
    print("RREV=%.3f, \t theta1=%.3f, \t theta2=%.3f, \t theta3=%.3f" %(mu[0], mu[1],mu[1],mu[1]))
    print("sig1=%.3f, \t sig2=%.3f, \t sig12=%.3f, \t sig2=%.3f," %(sigma[0], sigma[1],sigma[2],sigma[1]))
    print()

    done = False
    #reward = np.zeros(shape=(2*agent.n_rollout,1))
    reward = np.zeros(shape=(agent.n_rollout,1))
    reward[:] = np.nan  # initialize reward to be NaN array, size n_rollout x 1
    #theta_rl, epsilon_rl = agent.get_theta()
    theta_rl = agent.get_theta()

    print( "theta_rl = ")

    np.set_printoptions(precision=2, suppress=True)
    print(theta_rl[0,:], "--> RREV")
    print(theta_rl[1,:], "--> Gain")
    #print(theta_rl[2,:], "--> omega_x Gain")
    #print(theta_rl[3,:], "--> omega_y Gain")

    ct = env.getTime()

    # ============================
    ##          Run 
    # ============================
    k_run = 0
    while k_run < agent.n_rollout:#2*agent.n_rollout:
        
        # take initial camera measurements 
        #image_now = env.cv_image.flatten()
        #image_prev = env.cv_image.flatten()
        repeat_run= False
        error_str = ""

        vz_ini = np.random.uniform(low=2.5, high=3.5)   # [2.75, 3.75]
        vx_ini = 0.0#np.random.uniform(low=-1.5, high=1.5)  # [-0.5, 0.5]
        vy_ini = 0.0#np.random.uniform(low=-1.5, high=1.5) # [-0.5, 0.5]

        # try adding policy parameter for roll pitch rate for vy ( roll_rate = gain3*omega_x)

        # iniitial condition in terms of mag and angle
        '''radius = 2.0*np.random.rand()
        direction = 2.0*pi*np.random.rand() 
        vx_ini = radius*sin(direction)
        vy_ini = radius*cos(direction)'''



        '''
        vx        vy       vz         suceess?
          1   |  1    |   3      |       yes (off by .2)
          -1  |  1    |   3      |      sometimes rests improperly (off by .2)
          -1  |  -1   |   3      |       more unstable(off by .2)
         1    |  -1   |   3      |       yes (off by .2)



        '''
        print("\n!-------------------Episode # %d run # %d-----------------!" %(k_ep,k_run))
        print("RREV: %.3f \t gain1: %.3f \t gain2: %.3f \t gain3: %.3f" %(theta_rl[0,k_run], theta_rl[1,k_run],theta_rl[1,k_run],theta_rl[1,k_run]))

        print("Vz_ini: %.3f \t Vx_ini: %.3f \t Vy_ini: %.3f" %(vz_ini, vx_ini, vy_ini))

        state = env.reset()
        print(state[2],state[3])
        if abs(state[2]) > 0.1 or abs(state[3]) > 0.1:
            state = env.reset()
        #action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
        #env.step(action)

        k_step = 0
        done_rollout = False
        start_time_rollout = env.getTime()
        start_time_pitch = None
        pitch_triggered = False
        
        flip_flag = False

        state_history = None
        env.logDataFlag = True

        # ============================
        ##          Rollout 
        # ============================
        action = {'type':'vel', 'x':vx_ini, 'y':vy_ini, 'z':vz_ini, 'additional':0.0}
        #action = {'type':'pos', 'x':0.0, 'y':0.0, 'z':1.0, 'additional':0.0}

        env.step(action=action)
        
        RREV_trigger = theta_rl[0, k_run]


        while True:
                
            time.sleep(5e-4) # Time step size
            k_step = k_step + 1 # Time step
            ## Define current state
            state = env.state_current
            
            position = state[1:4]
            orientation_q = state[4:8]
            vel = state[8:11]
            vz, vx, vy= vel[2], vel[0], vel[1]
            omega = state[11:14]

            
            d = h_ceiling - position[2]

            # Use noisy distance measurement from sensor
            #d = env.laser_dist
            #print(d)
            RREV, omega_y ,omega_x = vz/d, vx/d, vy/d

            qw = orientation_q[0]
            qx = orientation_q[1]
            qy = orientation_q[2]
            qz = orientation_q[3]

            #image_now=env.cv_image.flatten() # collect current image and flatten to 1d
            #print(image_now)
            # concatente previous image, current image, and visual cues for training data
            #training_data = np.concatenate((image_prev,image_now,np.array([RREV*1000,omega_y*1000]))).astype(int)
            #image_prev = image_now # update image

            #print(data)

            R = Rotation.from_quat([qx,qy,qz,qw])
            R = R.as_matrix()
            angle = R[1,1]
            b3 = R[2,:]
            #print(angle, b3[0],b3[1],b3[2])
            if b3[2] < 0.0 and not flip_flag:
                # check if crazyflie flipped 

                #print(angle)
                action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
                env.step(action)
                # shut off motors for the rest of the run
                print("Flipped!! Shut motors")
                flip_flag = True
            
            #b3 = r.as_matrix()[:,2] # body z-axis
            #b3y =  np.dot(b3, np.array([0,0,1]))
            b3y=0
            r = [0,0,0]#r.as_euler('zyx', degrees=True)
            eul1,eul2,eul3 = r[0],r[1],r[2]
            #print(r)
            theta = np.arcsin( -2*(qx*qz-qw*qy) ) # obtained from matlab "edit quat2eul.m"


            # ============================
            ##    Pitch Criteria 
            # ============================
            if (RREV > RREV_trigger) and (pitch_triggered == False):
                start_time_pitch = env.getTime()
                env.enableSticky(1)

                wn = sqrt(omega_x**2 + omega_y**2)
                # add term to adjust for tilt 
                qRREV = theta_rl[1,k_run] * RREV 
                #qomega = theta_rl[2,k_run]*wn
                q_d = -qRREV# + qomega #omega_x#*(1-b3y)#sin(r[1]*3.14159/180)
                # torque on x axis to adjust for vy
                r_d = 0.0 #theta_rl[3,k_run] * omega_y

                print('----- pitch starts -----')
                print('vz=%.3f, vx=%.3f, vy=%.3f' %(vz, vx,vy))
                print('r[0]=%.3f, r[1]=%.3f, r[2]=%.3f, b3y=%.3f' %(r[0],r[1],r[2],b3y))
                print('RREV=%.3f, omega_y=%.3f, omega_x=%.3f, qd=%.3f' %( RREV, omega_y, omega_x,q_d) )   
                print("Pitch Time: %.3f" %start_time_pitch)
                #print('wy = %.3f , qomega = %.3f , qRREV = %.3f' %(omega[1],qomega,qRREV))

                # randomly sample noise delay with mean = 30ms and sigma = 5
                t_delay = np.random.normal(30.0,5.0)
                print("t_delay = %.3f" %(t_delay))
                env.delay_env_time(t_start=start_time_pitch,t_delay=t_delay) # Artificial delay to mimic communication lag [ms]
                
                action = {'type':'rate', 'x':r_d, 'y':q_d, 'z':0.0, 'additional':0.0}
                env.step(action) # Start rotation and mark rotation triggered
                pitch_triggered = True


            # ============================
            ##    Termination Criteria 
            # ============================

            ## If time since triggered pitch exceeds [0.7s]   
            if pitch_triggered and ((env.getTime()-start_time_pitch) > 0.7):
                # I don't like this formatting, feel free to improve on
                error_1 = "Rollout Completed: Pitch Triggered  "
                error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch))
                # print(error_1 + "\n" + error_2)

                error_str = error_1 + error_2
                done_rollout = True

            ## If time since rollout start exceeds [1.5s]
            if (env.getTime() - start_time_rollout) > 1.5:
                error_1 = "Rollout Completed: Time Exceeded   "
                error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout))
                # print(error_1 + "\n" + error_2)

                error_str = error_1 + error_2
                done_rollout = True
            

            # ============================
            ##          Errors  
            # ============================
            ## If nan is found in state vector repeat sim run
            if any(np.isnan(state)) or env.timeout: # gazebo sim becomes unstable, relaunch simulation
                print("NAN found in state vector")
                print(env.timeout)
                print(state)
                time.sleep(1)
                env.logDataFlag = False
                env.close_sim()
                env.launch_sim()
                error_str = "Error: NAN found in state vector"
                repeat_run = True
                break

            
            if (np.abs(position[0]) > 3.5) or (np.abs(position[1]) > 3.5):
                print("Reset improperly!!!!!")
                # env.pause()
                env.logDataFlag = False
                error_str = "Reset improperly/Position outside bounding box"
                repeat_run = True
                break
            
            ## Keep record of state vector every 10 time steps
            ## Each iteration changes state vector from [14,] into (state2)[14,1] 
            ##      First iteration: track_state = state2 vector
            ##      Every 10 iter: append track_state columns with current state2 vector 
            state2 = state[1:,np.newaxis]
            if state_history is None:
                state_history = state2 # replace w/ state_history variable for track_state
            else:
                if k_step%10==0:
                    state_history = np.append(state_history, state2, axis=1)
                    #env.append_csv(agent,np.around(state,decimals=3),k_ep,k_run)

            if done_rollout:
                action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
                env.step(action)
                env.logDataFlag = False
                reward[k_run] = agent.calculate_reward(state=state_history, h_ceiling=h_ceiling)
                time.sleep(0.01)
                print("Reward = %.3f" %(reward[k_run]))
                print("!------------------------End Run------------------------! \n")
                ## Episode Plotting
                plt.plot(k_ep,reward[k_run],marker = "_", color = "black", alpha = 0.5) 
                plt.title("Episode: %d Run: %d # Rollouts: %d" %(k_ep,k_run+1,agent.n_rollout))
                # If figure gets locked on fullscreen, press ctrl+f untill it's fixed (there's lag due to process running)
                plt.draw()
                plt.pause(0.001)
                # fig.canvas.flush_events()                
                break
        
        
        #env.append_csv(agent,np.around(state,decimals=3),k_ep,k_run,reward[k_run,0],error=error_str)
        #env.append_csv_blank()

        if repeat_run == True:
            # return to previous run to catch potential missed glitches in gazebo (they are usually caught in the next run)
            if k_run > 0:
                k_run -= 1 # Repeat run w/ same parameters
        else:
            k_run += 1 # Move on to next run


    if not any( np.isnan(reward) ):
        print("Episode # %d training, average reward %.3f" %(k_ep, np.mean(reward)))
        #agent.train(theta_rl,reward,epsilon_rl)
        agent.train(theta_rl,reward)
        print(reward)
        plt.plot(k_ep,np.mean(reward),'ro')
        plt.draw()
        plt.pause(0.001)


