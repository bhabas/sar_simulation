#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt
from math import sin,cos,pi,sqrt,atan,atan2
import os
from scipy.spatial.transform import Rotation
from numpy.core.fromnumeric import repeat

from crazyflie_env import CrazyflieEnv
from rl_EM import rlEM_PEPGAgent, rlEM_PEPG_CovAgent, rlEMsys_PEPGAgent, rlEM_AdaptiveCovAgent
from rl_syspepg import rlsysPEPGAgent_reactive ,rlsysPEPGAgent_adaptive

np.random.seed(0)
os.system("clear")

# ============================
##     Sim Initialization 
# ============================


## Initialize the environment
username = getpass.getuser()
env = CrazyflieEnv(port_self=18050, port_remote=18060,username = username)
print("Environment done")

## Initialize the user and data recording
start_time = time.strftime('_%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
file_name = '/home/'+username+'/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/' + username + start_time + '.csv'
env.create_csv(file_name,record = True)


## Initial figure setup
fig = plt.figure()
plt.ion()  # interactive on
plt.grid()
plt.xlim([-1,40])
plt.ylim([-1,25])  # change limit depending on reward function defenition
plt.xlabel("Episode")
plt.ylabel("Reward")
plt.title("Episode: %d Run: %d" %(0,0))
plt.show() 

## Sim Parameters
ep_start = 0 # Default episode start position
h_ceiling = 1.5 # meters
extra_time = 1.0


# ============================
##          EM PEPG 
# ============================

## Initial parameters for gaussian function
mu = np.array([[4.2],[-5.0]  ])# ,[1.5]])#,[1.5]])   # Initial estimates of mu: 
sigma = np.array([[0.1],[0.1] ,[0.0]])# ,[0.75]])      # Initial estimates of sigma: 

alpha_mu = np.array([[0.2],[0.2]])#[2.0]] )#,[0.1]])
alpha_sigma = np.array([[0.1],[0.1]  ])#, [1.0]])#,[0.05]])

#agent = rlEM_PEPG_CovAgent(mu,sigma,n_rollout=5)
#agent = rlsysPEPGAgent_reactive(alpha_mu,alpha_sigma,mu,sigma,n_rollout=5)
#agent = rlEM_PEPGAgent(mu,sigma,n_rollout=4)
#agent = rlEMsys_PEPGAgent(mu,sigma,n_rollout=5)
agent = rlEM_AdaptiveCovAgent(mu,sigma,n_rollout=5)
#agent = rlsysPEPGAgent_adaptive(alpha_mu,alpha_sigma,mu,sigma,n_rollout=5)
# ============================
##          Episode 
# ============================
for k_ep in range(ep_start,1000):


    mu = agent.mu
    sigma = agent.sigma

    done = False

    # os.system("python3 start_training_reactive_sysPEPG > output.txt")

    print("=============================================")
    print("STARTING Episode # %d" %k_ep)
    print("=============================================")

    print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )

    print("RREV=%.3f, \t theta1=%.3f, \t theta2=%.3f, \t theta3=%.3f" %(mu[0], mu[1], mu[1], mu[1]))
    print("sig1=%.3f, \t sig2=%.3f, \t sig12=%.3f, \t sig2=%.3f," %(sigma[0], sigma[1], sigma[2], sigma[1]))
    print('\n')


    
    reward = np.zeros(shape=(2*agent.n_rollout,1))
    #reward = np.zeros(shape=(agent.n_rollout,1))
    reward[:] = np.nan  # initialize reward to be NaN array, size n_rollout x 1
    theta_rl,epsilon_rl = agent.get_theta()

    print("theta_rl = ")

    np.set_printoptions(precision=2, suppress=True)
    print(theta_rl[0,:], "--> RREV")
    print(theta_rl[1,:], "--> Gain")
    #print(theta_rl[2,:], "--> v_x Gain")
    #print(theta_rl[3,:], "--> omega_y Gain")



    # ============================
    ##          Run 
    # ============================
    k_run = 0
    while k_run < 2*agent.n_rollout:
        
        # take initial camera measurements 
        #image_now = env.cv_image.flatten()
        #image_prev = env.cv_image.flatten()
        

        vz_ini = np.random.uniform(low=2.0, high=3.0)
        vx_ini = 0.0#np.random.uniform(low=-0.5, high=0.5)
        #vx_ini = np.random.normal(0.0,1.0)
        vy_ini = 0.0#np.random.uniform(low=-0.5, high=0.5)
        v_ini = [vz_ini,vx_ini,vy_ini]
        # try adding policy parameter for roll pitch rate for vy ( roll_rate = gain3*omega_x)

        #v_mag = np.random.uniform(low=0.0,high=2.0)
        #angle = np.random.uniform(low=0.0,high=2.0*pi)

        #vx_ini = v_mag*cos(angle)
        #vy_ini = v_mag*sin(angle)

        print("\n!-------------------Episode # %d run # %d-----------------!" %(k_ep,k_run))
        print("RREV: %.3f \t gain1: %.3f \t gain2: %.3f \t gain3: %.3f" %(theta_rl[0,k_run], theta_rl[1,k_run],theta_rl[1,k_run],theta_rl[1,k_run]))
        print("Vz_ini: %.3f \t Vx_ini: %.3f \t Vy_ini: %.3f" %(vz_ini, vx_ini, vy_ini))



        state = env.reset()
        env.IC_csv(agent,state,'sim',k_run,v_ini = v_ini)



        ## If spawn position is off then reset position again
        x_pos,y_pos = state[2],state[3]
        print("Spawn Pos: x=%.3f \t y=%.3f" %(x_pos,y_pos))
        if abs(x_pos) > 0.1 or abs(y_pos) > 0.1:
            state = env.reset()


        action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
        env.step(action)

        t_step = 0
        
        done_rollout = False
        start_time_rollout = env.getTime()
        start_time_pitch = None
        pitch_triggered = False
        flip_flag = False
        crash_flag = False
        wall_flag = False

        state_history = None
        repeat_run= False
        error_str = ""

        vx_avg = 0

        # ============================
        ##          Rollout 
        # ============================
        action = {'type':'vel', 'x':vx_ini, 'y':vy_ini, 'z':vz_ini, 'additional':0.0}
        #action = {'type':'pos', 'x':0.0, 'y':0.0, 'z':1.0, 'additional':0.0}

        env.step(action=action)
        
        RREV_trigger = theta_rl[0, k_run]


        while True:
                
            time.sleep(5e-4) # Time step size
            t_step = t_step + 1 # Time step
            ## Define current state

            if not wall_flag: # If hit wall, assume stuck and keep old state (velocities not needed)
                state = env.state_current
            
            position = state[1:4]
            orientation_q = state[4:8]
            vel = state[8:11]
            vz, vx, vy= vel[2], vel[0], vel[1]
            omega = state[11:14]
            d = h_ceiling - position[2]

            RREV, OF_y, OF_x = vz/d, vx/d, vy/d # OF_x,y are optical flows of the ceiling assuming no body rotation
            sensor_data = [RREV, OF_y, OF_x] # simplifying for data recording
            qw = orientation_q[0]
            qx = orientation_q[1]
            qy = orientation_q[2]
            qz = orientation_q[3]

            R = Rotation.from_quat([qx,qy,qz,qw])
            R = R.as_matrix()
            angle = R[1,1]
            b3 = R[2,:]
            # ============================
            ##    Motor Shutdown Criteria 
            # ============================
            if b3[2] < 0.0 and not flip_flag:
                # check if crazyflie flipped 
                action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
                env.step(action)
                # shut off motors for the rest of the run
                print("Flipped!! Shut motors")
                flip_flag = True
                        
            if ((d < 0.05) and (not crash_flag) and (not flip_flag)): # might need to adjust this 
                action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
                env.step(action)
                print("Crashed!! Shut motors")
                crash_flag = True

            if ((abs(position[0]) > 0.5) or (abs(position[1]) > 0.5)) and not wall_flag:
                action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
                env.step(action)
                print("Hit wall!! Shut motors")
                wall_flag = True
            
            # ============================
            ##    Pitch Criteria 
            # ============================
            if (RREV > RREV_trigger) and (pitch_triggered == False):
                start_time_pitch = env.getTime()
                env.enableSticky(1)

                qRREV = theta_rl[1,k_run]*RREV
                q_pitch = qRREV*np.sign(OF_y)
                q_roll = 0.0 

                print('----- pitch starts -----')
                print('vz=%.3f, vx=%.3f, vy=%.3f' %(vz,vx,vy))
                print('RREV=%.3f, OF_y=%.3f, OF_x=%.3f, q_pitch=%.3f' %( RREV, OF_y, OF_x,q_pitch) )   
                print("Pitch Time: %.3f" %start_time_pitch)

                # randomly sample noise delay with mean = 30ms and sigma = 5
                #t_delay = max(np.random.normal(30.0,10.0),0.0)
                #print("t_delay = %.3f" %(t_delay))
                #env.delay_env_time(t_start=start_time_pitch,t_delay=t_delay) # Artificial delay to mimic communication lag [ms]
                
                action = {'type':'rate', 'x':q_roll, 'y':q_pitch, 'z':0.0, 'additional':0.0}    
                env.step(action) 
                pitch_triggered = True


            # ============================
            ##    Termination Criteria 
            # ============================

            ## If time since triggered pitch exceeds [0.7s]   
            if pitch_triggered and ((env.getTime()-start_time_pitch) > (0.7 + extra_time)):
                # I don't like this formatting, feel free to improve on
                error_1 = "Rollout Completed: Pitch Triggered  "
                error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch))
                # print(error_1 + "\n" + error_2)

                error_str = error_1 + error_2
                done_rollout = True

            ## If time since rollout start exceeds [1.5s]
            if (env.getTime() - start_time_rollout) > (1.5 + extra_time):
                error_1 = "Rollout Completed: Time Exceeded   "
                error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout))
                # print(error_1 + "\n" + error_2)

                error_str = error_1 + error_2
                done_rollout = True
            

            # ============================
            ##          Errors  
            # ============================
            ## If nan is found in state vector repeat sim run
            if any(np.isnan(state)): # gazebo sim becomes unstable, relaunch simulation
                print("NAN found in state vector")
                error_str = "Error: NAN found in state vector"
                repeat_run = True
                break

            elif env.timeout:
                print("Controller reciever thread timed out")
                error_str = "Error: Controller Timeout"
                repeat_run = True
                break
            
            elif (np.abs(position[0]) > 5.0) or (np.abs(position[1]) > 5.0):
                # might just need to increase bounds (not necessarily an error)
                print("Reset improperly / Out of bounds !!!!!")
                error_str = "Reset improperly/Position outside bounding box"
                repeat_run = True
                break
            

            # ============================
            ##      Record Keeping  
            # ============================
            ## Keep record of state vector every 10 time steps
            state2 = state[1:, np.newaxis]
            if state_history is None:
                state_history = state2 
            else:
                if t_step%10==0: # Append state_history columns with current state2 vector 
                    state_history = np.append(state_history, state2, axis=1)
                    env.append_csv(agent,state,k_ep,k_run,sensor_data)

            if done_rollout:
                print('Final Position = (%.3f , %.3f) m' %(position[0],position[1]))
                action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
                env.step(action)
                reward[k_run] = agent.calculate_reward(state_history,h_ceiling)
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
        
        env.append_csv(agent,state,k_ep,k_run,sensor_data,reward=reward[k_run,0],error=error_str)
        env.append_csv_blank()
        time.sleep(0.01)

        if repeat_run == True:
            time.sleep(1)
            env.close_sim()
            time.sleep(1)
            env.close_sim()
            env.launch_sim()
            # return to previous run to catch potential missed glitches in gazebo (they are usually caught in the next run)
            if k_run > 0:
                k_run -= 1 # Repeat previous (not current) run 
        else:
            k_run += 1 # Move on to next run

    print(reward)

    if not any( np.isnan(reward) ):
        print("Episode # %d training, average reward %.3f" %(k_ep, np.mean(reward)))
        #env.append_csv(agent,reward_avg = np.mean(reward))
        #env.append_csv_blank()
        agent.train(theta_rl,reward,epsilon_rl)
        plt.plot(k_ep,np.mean(reward),'ro')
        plt.draw()
        plt.pause(0.001)