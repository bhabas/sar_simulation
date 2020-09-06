#!/usr/bin/env python3

import numpy as np
import time,copy
from crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive
import matplotlib.pyplot as plt


env = CrazyflieEnv(port_self=18050, port_remote=18060)
print("Environment done")
alpha_mu = np.array([[0.1],[0.3]])
alpha_sigma = np.array([[0.1],[0.3]])
agent = rlsysPEPGAgent_reactive(_alpha_mu=alpha_mu, _alpha_sigma=alpha_sigma, _gamma=0.95, _n_rollout=5)

## Define initial parameters for gaussian function
agent.mu_ = np.array([[4.0], [-5.0]])   # Initial estimates of mu: size (2 x 1)
agent.sigma_ = np.array([[0.5], [2.0]])      # Initial estimates of sigma: size (2 x 1)
agent.mu_history_ = copy.copy(agent.mu_)  # Creates another array of self.mu_ and attaches it to self.mu_history_
agent.sigma_history_ = copy.copy(agent.sigma_)


h_ceiling = 1.5 # meters
t_delay = 30 # ms

username = 'bhabas' # change to system user
start_time0 = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
file_name = '/home/'+username+'/catkin_ws/src/src/crazyflie_simulation/4. rl/src/log/' + start_time0 + '.xls'
#file_log, sheet = env.create_xls(start_time=start_time0, sigma=sigma, alpha=alpha, file_name=file_name)


## Initial figure setup
plt.ion()  # interactive on
fig = plt.figure()
plt.grid()
plt.xlim([-10,100])
plt.ylim([0,150])
plt.xlabel("Episode")
plt.ylabel("Reward")
plt.title("Episode: %d Run: %d" %(0,0))
plt.show() 


# ============================
##          Episode 
# ============================
for k_ep in range(1000):
    
    vz_ini = 3.0 #+ np.random.rand()
    vx_ini = 0.0 #np.random.rand()

    print("=============================================")
    print("STARTING Episode # %d" %k_ep)
    print("=============================================")

    print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
    print("Vz_ini: %.3f \t Vx_ini: %.3f" %(vz_ini, vx_ini))
    mu = agent.mu_
    sigma = agent.sigma_
    print("RREV=%.3f, \t theta2=%.3f" %(mu[0], mu[1]))
    print("sig1=%.3f, \t sig2  =%.3f" %(sigma[0], sigma[1]))
    print()

    done = False
    reward = np.zeros(shape=(2*agent.n_rollout_,1))
    reward[:] = np.nan                      # initialize reward to be NaN array, size n_rollout x 1
    theta_rl, epsilon_rl = agent.get_theta()
    print( "theta_rl = ")
    np.set_printoptions(precision=3, suppress=True)
    print(theta_rl[0,:], "--> RREV")
    print(theta_rl[1,:], "--> Gain")



    # ============================
    ##          Run 
    # ============================
    k_run = 0
    while k_run < 2*agent.n_rollout_:

        print("Episode # %d run # %d" %(k_ep,k_run))
        state = env.reset()

        k_step = 0
        done_rollout = False
        start_time_rollout = env.getTime()
        start_time_pitch = None
        pitch_triggered = False
        track_state = None
        
        env.logDataFlag = True

        # ============================
        ##          Rollout 
        # ============================
        action = {'type':'vel', 'x':vx_ini, 'y':0.0, 'z':vz_ini, 'additional':0.0}
        env.step(action=action)
        
        RREV_trigger = theta_rl[0, k_run]

        # image_now = env.cv_image
        while True:
            time.sleep(5e-4) # Time step size
            state = env.state_current_
            k_step = k_step + 1
            position = state[1:4]
            orientation_q = state[4:8]
            vel = state[8:11]
            vz, vx = vel[2], vel[0]
            omega = state[11:14]

            # image_prev = image_now
            # image_now = env.cv_image
            # d = env.laser_dist # added distance sensor with noise

            d = h_ceiling - position[2]
            RREV, omega_y = vz/d, vx/d

            qw = orientation_q[0]
            qx = orientation_q[1]
            qy = orientation_q[2]
            qz = orientation_q[3]
            theta = np.arcsin( -2*(qx*qz-qw*qy) )         # obtained from matlab "edit quat2eul.m"

            ## Enable sticky feet and rotation
            if (RREV > RREV_trigger) and (not pitch_triggered):
                print('------------- pitch starts -------------')
                print( 'vz=%.3f, vx=%.3f, RREV=%.3f, d=%.3f' %(vz, vx, RREV, d) )   
                start_time_pitch = env.getTime()
                env.enableSticky(1)

                q_d = theta_rl[1,k_run] * RREV
                
                env.delay_env_time(t_start=start_time_pitch,t_delay=t_delay) # added delay
                action = {'type':'rate', 'x':0.0, 'y':q_d, 'z':0.0, 'additional':0.0}
                env.step(action)
                pitch_triggered = True

            ## If time exceed limits complete rollout    
            if pitch_triggered and ((env.getTime()-start_time_pitch) > 0.7):
                done_rollout = True

            if (env.getTime() - start_time_rollout) > 1.5:
                done_rollout = True
            
            ## Error collection
            if any( np.isnan(state) ):     # gazebo sim becomes unstable, relaunch simulation
                print("Find NAN!!!!!")
                env.logDataFlag = False
                env.close_sim()
                env.launch_sim()
                if k_run > 0:
                    k_run = k_run - 1
                break

            if (np.abs(position[0]) > 0.6) or (np.abs(position[1]) > 0.6):
                print("Reset improperly!!!!!")
                env.logDataFlag = False
                break

            ## Each iteration changes state vector from [14,] into (state2)[14,1] 
            ##      First iteration: track_state = state2 vector
            ##      Every 10 iter: append track_state columns with current state2 vector 
            state2 = state[1:,np.newaxis]
            if track_state is None:
                track_state = state2
            else:
                if k_step%10==0:
                    track_state = np.append(track_state, state2, axis=1)

            if done_rollout:
                env.logDataFlag = False
                reward[k_run] = agent.calculate_reward(_state=track_state, _h_ceiling=h_ceiling)

                ## Episode Plotting
                plt.plot(k_ep,reward[k_run],marker = "_", color = "black", alpha = 0.5) 
                plt.title("Episode: %d Run: %d" %(k_ep, k_run+1))
                # If figure gets locked on fullscreen, press ctrl+f untill it's fixed (there's lag due to process running)
                plt.draw()
                fig.canvas.flush_events()
                
                k_run = k_run + 1
                #print( 'x=%.3f, y=%.3f, z=%.3f' %(position[0], position[1], position[2]) )
                break

    if not any( np.isnan(reward) ):
        print("Episode # %d training, average reward %.3f" %(k_ep, np.mean(reward)))
        agent.train(_theta = theta_rl, _reward=reward, _epsilon=epsilon_rl)

        plt.plot(k_ep,np.mean(reward),'ro')
        plt.draw()
        plt.pause(0.001)

        # env.add_record_xls(file_log=file_log, sheet=sheet, file_name=file_name,
        #     k_ep=k_ep, start_time1=start_time11, start_time2=start_time12,
        #     vz_ini=vz_ini, vx_ini=vx_ini, state=state, action=action[0],
        #     reward=reward, info=info, theta=theta)

