#!/usr/bin/env python3
import numpy as np
import time
from crazyflie_env import CrazyflieEnv
from rl_dmp import rlsysPEPGAgent_reactive, DMP
import matplotlib.pyplot as plt


env = CrazyflieEnv(port_self=18050, port_remote=18060)
print("Environment done")
alpha_mu = np.array([[0.1],[0.3]])
alpha_sigma = np.array([[0.1],[0.3]])
agent = rlsysPEPGAgent_reactive(_alpha_mu=alpha_mu, _alpha_sigma=alpha_sigma, _gamma=0.95, _n_rollout=5)


h_ceiling = 1.5

username = 'bader' # change to system user
start_time0 = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
file_name = '/home/'+username+'/catkin_ws/src/4. rl/src/log/' + start_time0 + '.xls'
#file_log, sheet = env.create_xls(start_time=start_time0, sigma=sigma, alpha=alpha, file_name=file_name)

plt.figure()
plt.xlim([0,100])
plt.ylim([0,150])
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.ion()       # interactive on
plt.show()

for k_ep in range(1000):
    
    vz_ini = 3.0 #+ np.random.rand()
    vx_ini = 0.0#np.random.rand()

    print("=============================================")
    print("=============================================")
    print("STARTING Episode # %d" %k_ep)
    print( time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) )
    print("Vz_ini: %.3f --- Vx_ini: %.3f" %(vz_ini, vx_ini))
    mu = agent.mu_
    sigma = agent.sigma_
    print( 'RREV=%.3f, theta2=%.3f' %(mu[0], mu[1]) )
    print( 'sig1=%.3f, sig2  =%.3f' %(sigma[0], sigma[1]) )
    
    done = False
    reward = np.zeros(shape=(2*agent.n_rollout_,1))
    reward[:] = np.nan                      # initialize reward to be NaN array, size n_rollout x 1
    theta_rl, epsilon_rl = agent.get_theta()
    print( 'theta_rl = ' )
    print(theta_rl)

    k_run = 0
    while k_run < 2*agent.n_rollout_:

        #print("Episode # %d run # %d" %(k_ep,k_run))
        state = env.reset()

        k_step = 0
        done_rollout = False
        start_time_rollout = env.getTime()
        start_time_pitch = None
        pitch_triggered = False
        track_state = None
        
        env.logDataFlag = True

        action = {'type':'vel', 'x':vx_ini, 'y':0.0, 'z':vz_ini, 'additional':0.0}
        env.step(action=action)
        
        RREV_trigger = theta_rl[0, k_run]

        while True:
            time.sleep(5e-4)
            state = env.state_current_
            k_step = k_step + 1
            position = state[1:4]
            orientation_q = state[4:8]
            vel = state[8:11]
            omega = state[11:14]
            d = h_ceiling - position[2]
            vz, vx = vel[2], vel[0]
            RREV, omega_y = vz/d, vx/d

            qw = orientation_q[0]
            qx = orientation_q[1]
            qy = orientation_q[2]
            qz = orientation_q[3]
            theta = np.arcsin( -2*(qx*qz-qw*qy) )         # obtained from matlab "edit quat2eul.m"

            if (RREV > RREV_trigger) and (not pitch_triggered):
                print('------------- pitch starts -------------')
                print( 'vz=%.3f, vx=%.3f, RREV=%.3f, d=%.3f' %(vz, vx, RREV, d) )
                start_time_pitch = env.getTime()
                env.enableSticky(1)

                q_d = theta_rl[1,k_run] * RREV
                t_delay = 500 # ms
                while (env.getTime() - start_time_pitch < t_delay/10000):
                    print(env.getTime() - start_time_pitch)
                    pass 
                action = {'type':'rate', 'x':0.0, 'y':q_d, 'z':0.0, 'additional':0.0}
                env.step(action)
                pitch_triggered = True
                
            if pitch_triggered and ((env.getTime()-start_time_pitch) > 0.7):
                done_rollout = True

            if (env.getTime() - start_time_rollout) > 1.5:
                done_rollout = True

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

            state2 = state[1:,np.newaxis]
            if track_state is None:
                track_state = state2
            else:
                if k_step%10==0:
                    track_state = np.append(track_state, state2, axis=1)

            if done_rollout:
                env.logDataFlag = False
                reward[k_run] = agent.calculate_reward(_state=track_state, _h_ceiling=h_ceiling)
                plt.plot(k_ep,reward[k_run],'k*')
                plt.draw()
                plt.pause(0.001)
                
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

        '''plt.figure()
        #plt.hold()
        plt.plot(track_dmp1[0,:],track_dmp1[1,:], track_dmp1[0,:],track_dmp1[2,:])
        #plt.plot(track_dmp2[0,:],track_dmp2[1,:]*180/3.14, track_dmp2[0,:],track_dmp2[2,:]*180/3.14)
        plt.show()'''


input("Input Enter to exit")

