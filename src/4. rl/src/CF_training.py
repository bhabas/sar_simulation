#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from multiprocessing import Process,Array
from math import sin,cos,pi,sqrt,atan,atan2
import os
from scipy.spatial.transform import Rotation
from numpy.core.fromnumeric import repeat

from crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive ,rlsysPEPGAgent_cov, rlsysPEPGAgent_adaptive
from rl_EM import rlEM_PEPGAgent, rlEM_PEPG_CovAgent, rlEM_OutlierAgent , rlEMsys_PEPGAgent,rlEM_AdaptiveCovAgent, rlEM_AdaptiveCovAgent3D, rlEM_AdaptiveAgent
from rl_cma import CMA_basic,CMA,CMA_sym

os.system("clear")

def runGraph():
    # Parameters
    print('show')
    x_len = 200         # Number of points to display
    y_range = [0,4]  # Range of possible Y values to display

    # Create figure for plotting
    fig2 = plt.figure(2)
    ax = fig2.add_subplot(1, 1, 1)
    xs = list(range(0, 200))
    ys = [0] * x_len
    ax.set_ylim(y_range)

    # Create a blank line. We will update the line in animate
    line, = ax.plot(xs, ys)

    # Add labels
    plt.xlabel('Samples')
    plt.ylabel('Z-Position [m]')

    # This function is called periodically from FuncAnimation
    def animate(i, ys):

        pos_z = state_mp[3]

        ys.append(pos_z)  # Add y to list
        ys = ys[-x_len:] # Limit y list to set number of items

        line.set_ydata(ys) # Update line with new Y values

        return line,


    # Set up plot to call animate() function periodically

    ani = animation.FuncAnimation(fig2,
        animate,
        fargs=(ys,),
        interval=50,
        blit=True)
    plt.show()


def main():

    # ============================
    ##     Sim Initialization 
    # ============================


    ## Initialize the environment
    username = getpass.getuser()
    env = CrazyflieEnv()
    print("Environment done")

    ## Initialize the user and data recording
    start_time = time.strftime('_%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
    file_name = '/home/'+username+'/catkin_ws/src/crazyflie_simulation/src/4. rl/src/log/' + username + start_time + '.csv'
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





    # ============================
    ##           PEPG 
    # ============================

    ## Learning rate
    alpha_mu = np.array([[0.2]])#[2.0]] )#,[0.1]])
    alpha_sigma = np.array([[0.1]])#, [1.0]])#,[0.05]])

    ## Initial parameters for gaussian function
    mu = np.array([[4.5],[-4.5], [1.5] ])# ,[1.5]])#,[1.5]])   # Initial estimates of mu: 
    sigma = np.array([[1.5],[1.5] ,[0.25] ])# ,[0.75]])      # Initial estimates of sigma: 

    # noise tests all started at:
    #mu = np.array([[5.0],[-5.0] ])
    #sigma = np.array([[3.0],[3.0] ])
    #  
    agent = rlsysPEPGAgent_reactive(alpha_mu, alpha_sigma, mu,sigma, gamma=0.95,n_rollout=2)
    #agent = rlsysPEPGAgent_cov(alpha_mu, alpha_sigma, mu,sigma, gamma=0.95,n_rollout=4)
    #agent = rlEM_PEPGAgent(mu,sigma,n_rollout=5)
    #agent = rlsysPEPGAgent_adaptive(alpha_mu,alpha_sigma,mu,sigma,n_rollout=10)
    #agent = rlEM_OutlierAgent(mu,sigma,n_rollout=5) # diverges?
    #agent = rlEM_PEPG_CovAgent(mu,sigma,n_rollout=5)

    #agent = rlEMsys_PEPGAgent(mu,sigma,n_rollout=5)
    #agent = rlEM_AdaptiveCovAgent(mu,sigma,gamma=0.95,n_rollout=5)
    #agent = rlEM_AdaptiveCovAgent3D(mu,sigma,gamma=0.95,n_rollout=5)


    #agent = rlEM_AdaptiveAgent(mu,sigma,n_rollout=5)
    # ============================
    ##           CMA 
    # ============================

    # seems to be unstable if mu is close to zero (coverges to deterimistic)
    ## Initial parameters for gaussian function
    #mu = np.array([[3.0],[-3.0] ])#,[1.5]])   # Initial estimates of mu: 
    #sigma = np.array([[1.0],[1.0] , [0.5]])      # Initial estimates of sigma: 

    # # For CMA_basic, make sure simga has 3 inputs / for pepg it must be 2
    # agent = CMA_basic(mu,sigma,N_best=0.3,n_rollout=10)

    #agent = CMA(n=2,gamma = 0.9) # number of problem dimensions
    #agent = CMA_sym(n=2,gamma=0.9)




    # ============================
    ##          Episode 
    # ============================
    for k_ep in range(ep_start,1000):

        np.set_printoptions(precision=2, suppress=True)
        done = False

        mu = agent.mu
        sigma = agent.sigma

        reward = np.zeros(shape=(2*agent.n_rollout,1))
        reward[:] = np.nan  # initialize reward to be NaN array, size n_rollout x 1
        theta_rl,epsilon_rl = agent.get_theta()

        #mu = agent.xmean
        #sigma = np.array([agent.C[0,0],agent.C[1,1],agent.C[0,1]])


        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )

        print("RREV=%.3f, \t theta1=%.3f, \t theta2=%.3f, \t theta3=%.3f" %(mu[0], mu[0], mu[0], mu[0]))
        print("sig1=%.3f, \t sig2=%.3f, \t sig12=%.3f, \t sig2=%.3f," %(sigma[0], sigma[0], sigma[0], sigma[0]))
        print('\n')

        print("theta_rl = ")
        print(theta_rl[0,:], "--> RREV")
        print(theta_rl[1,:], "--> Gain")
        # print(theta_rl[2,:], "--> v_x Gain")
        # print(theta_rl[3,:], "--> omega_y Gain")



        # ============================
        ##          Run 
        # ============================
        k_run = 0
        while k_run < 2*agent.n_rollout:

            ## RESET TO INITIAL STATE
            env.step('home',ctrl_flag=1) # Reset control vals and functionality
            state = env.reset_pos() # Reset Gazebo pos
            time.sleep(3.0) # time for CF to settle


            ## INITIATE RUN PARAMETERS
            RREV_trigger = theta_rl[0, k_run] # FOV expansion velocity [rad/s]
            G1 = theta_rl[1, k_run]
            G2 = theta_rl[2, k_run]
            policy = theta_rl[:,k_run]
            policy = policy[:,np.newaxis]
            # policy = np.reshape(policy,(-1,1)) # reshaping for data logging

        
            vz_d = np.random.uniform(low=2.5, high=3.0)
            vx_d = np.random.uniform(low=-2.0, high=2.0)
            vy_d = 0 
            v_d = [vx_d,vy_d,vz_d] # [m/s]
            # try adding policy parameter for roll pitch rate for vy ( roll_rate = gain3*omega_x)


            ## INIT RUN FLAGS
            t_step = 0
            z_max = 0 # [m]
            
            done_rollout = False
            start_time_rollout = env.getTime()
            start_time_pitch = None
            pitch_triggered = False
            flip_flag = False
            crash_flag = False

            state_history = None
            repeat_run= False
            error_str = ""


            print("\n!-------------------Episode # %d run # %d-----------------!" %(k_ep,k_run))
            print("RREV: %.3f \t Gain_1: %.3f \t Gain_2: %.3f \t Gain_3: %.3f" %(RREV_trigger, G1, G2, 0))
            print("Vz_d: %.3f \t Vx_d: %.3f \t Vy_d: %.3f" %(vz_d, vx_d, vy_d))



            # ============================
            ##          Rollout 
            # ============================
            
            env.step('vel',v_d,ctrl_flag=1) # Set desired vel
            env.step('pos',ctrl_flag=0) # turn off pos control
            
            while True:
                    
                time.sleep(5e-4) # Time step size [Not sure if this is needed]
                t_step += 1


                ## DEFINE CURRENT STATE [Can we thread this to get states even when above]
                state = env.state_current
                print(state)
                
                position = state[1:4] # [x,y,z]
                orientation_q = state[4:8] # Orientation in quat format
                vel = state[8:11]
                vx,vy,vz = vel
                omega = state[11:14]
                d = h_ceiling - position[2] # distance of drone from ceiling

                ## Orientation data from state
                qw,qx,qy,qz = orientation_q
                R = Rotation.from_quat([qx,qy,qz,qw])
                R = R.as_matrix()
                b3 = R[2,:] # Vertical body axis in Global axes
                b2 = R[1,:] # Horizontal body axis
                b1 = R[0,:] # Forward body axis (related to yaw)



                RREV, OF_y, OF_x = vz/d, vx/d, vy/d # OF_x,y are estimated optical flow vals assuming no body rotation
                sensor_data = [RREV, OF_y, OF_x] # simplified for data recording


                # ============================
                ##   Motor Shutdown Criteria 
                # ============================
                if b3[2] < 0.0 and not flip_flag: # If b3_z is negative then CF flipped
                    print("Flipped!!")
                    flip_flag = True
                
                if ((d < 0.05) and (not crash_flag) and (not flip_flag)): # might need to adjust this 
                    env.step('stop')
                    print("Crashed!!")
                    crash_flag = True

                
                # ============================
                ##    Pitch Criteria 
                # ============================
                if (RREV > RREV_trigger) and (pitch_triggered == False):
                    start_time_pitch = env.getTime()
                    env.enableSticky(1)

                    omega_yd = (G1*RREV + G2*abs(OF_y))*np.sign(OF_y)# + qomega #omega_x#*(1-b3y)#sin(r[1]*3.14159/180)
                    omega_xd = 0.0 #theta_rl[3,k_run] * omega_y
                    omega_zd = 0.0

                    omega_d = [omega_xd,omega_yd,omega_zd]

                    print('----- pitch starts -----')
                    print('vz=%.3f, vx=%.3f, vy=%.3f' %(vz,vx,vy))
                    print('RREV=%.3f, OF_y=%.3f, OF_x=%.3f, q_pitch=%.3f' %( RREV, OF_y, OF_x, omega_yd) )   
                    print("Pitch Time: %.3f" %start_time_pitch)
                    #print('d_est = %.3f, d_act = %.3f' %(d,d_acutal))
                    #print('wy = %.3f , qomega = %.3f , qRREV = %.3f' %(omega[1],qomega,qRREV))

                    #t_delay = np.random.normal(30.0,5.0)
                    #print("t_delay = %.3f" %(t_delay))
                    #env.delay_env_time(t_start=start_time_pitch,t_delay=t_delay) # Artificial delay to mimic communication lag [ms]
                    

                    ## Start rotation and mark rotation as triggered
                    env.step('omega',omega_d,ctrl_flag=1) # set ang rate control
                    pitch_triggered = True


                # ============================
                ##    Termination Criteria 
                # ============================

                # If time since triggered pitch exceeds [0.7s]   
                if pitch_triggered and ((env.getTime()-start_time_pitch) > (0.3)):
                    # I don't like this formatting, feel free to improve on
                    error_1 = "Rollout Completed: Pitch Triggered  "
                    error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch))
                    print(error_1 + "\n" + error_2)

                    error_str = error_1 + error_2
                    done_rollout = True

                # If time since rollout start exceeds [1.5s]
                if (env.getTime() - start_time_rollout) > (2.5):
                    error_1 = "Rollout Completed: Time Exceeded   "
                    error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout))
                    print(error_1 + "\n" + error_2)
                

                    error_str = error_1 + error_2
                    done_rollout = True

                # If position falls below max height (There is lag w/ this)
                z_max = max(position[2],z_max)
                if position[2] <= 0.75*z_max:
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

                    env.step('stop')
                    reward[k_run] = agent.calculate_reward(state_history,h_ceiling)
                    time.sleep(0.01)
                    print("Reward = %.3f" %(reward[k_run]))
                    print("!------------------------End Run------------------------! \n")
                    ## Episode Plotting
                    plt.plot(k_ep,reward[k_run],marker = "_", color = "black", alpha = 0.5) 
                    plt.title("Episode: %d Run: %d Rollouts: %d" %(k_ep,k_run+1,agent.n_rollout))
                    # If figure gets locked on fullscreen, press ctrl+f untill it's fixed (there's lag due to process running)
                    plt.draw()
                    plt.pause(0.001)
                    # fig.canvas.flush_events()                
                    break
            
            
            env.append_csv(agent,state,k_ep,k_run,sensor_data)
            env.IC_csv(agent,state,k_ep,k_run,policy,v_d,omega_d,reward[k_run,0],error_str)
            env.append_csv_blank()
            time.sleep(0.01)
            if repeat_run == True:
                env.close_sim()
                time.sleep(1)
                env.launch_sim()
                # return to previous run to catch potential missed glitches in gazebo (they are usually caught in the next run)
                if k_run > 0:
                    k_run -= 1 # Repeat previous (not current) run 
            else:
                k_run += 1 # Move on to next run

            ## =======  RUN COMPLETED  ======= ##    



        if not any( np.isnan(reward) ):
            print("Episode # %d training, average reward %.3f" %(k_ep, np.mean(reward)))
            agent.train(theta_rl,reward,epsilon_rl)
            plt.plot(k_ep,np.mean(reward),'ro')
            plt.draw()
            plt.pause(0.001)
        
        ## =======  EPISODE COMPLETED  ======= ##



    ## =======  MAX TRIALS COMPLETED  ======= ##

if __name__ == '__main__':
    state_mp = Array('d',14)
    p1 = Process(target=runGraph)
    p1.start()
    main()

