#!/usr/bin/env python

import rospy
import numpy as np
import time
from quadcopter_env import QuadCopterEnv
from rl_dmp import PolicySearch, DMP
import matplotlib.pyplot as plt

rospy.init_node('drone_gym', anonymous=True)

env = QuadCopterEnv()
rospy.loginfo( "Environment done")

sigma = 10
alpha = 1000
agent = PolicySearch(n_states=2, sigma=sigma, alpha=alpha)
rospy.loginfo( "Policy Initiated")

dmp1 = DMP('DMP_upward')
dmp2 = DMP('DMP_rotation')
# print(dmp1.params)
# print(dmp1.basisFcnParams)
# print(dmp2.params)
# print(dmp2.basisFcnParams)

h_ceiling = 1.5

start_time0 = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
file_name = '/home/pan/catkin_ws/src/robot landing/4. rl/src/log/' + start_time0 + '.xls'
file_log, sheet = env.create_xls(start_time=start_time0, sigma=sigma, alpha=alpha, file_name=file_name)

for k_ep in range(1000):
    pose = env.reset()
    vz_ini = 1.0 + np.random.rand()
    vx_ini = 0.0#np.random.rand()

    rospy.loginfo("=============================================")
    rospy.loginfo("=============================================")
    rospy.loginfo("STARTING Episode # "+str(k_ep))
    rospy.loginfo( time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) )
    rospy.loginfo("Vz_ini: "+str(np.around(vz_ini,3))+" --- Vx_ini: "+str(np.around(vx_ini,3)))
    
    done = False
    start_time = time.time()
    start_time11 = time.strftime('%Y-%m-%d', time.localtime(start_time))
    start_time12 = time.strftime('%H:%M:%S', time.localtime(start_time))
    reward = 0.0
    dmp1_flag = True
    start_time_dmp1 = None
    start_time_dmp2 = None
    #time_dmp2 = 0
    k_step = 0
    track_dmp1 = np.zeros(shape=(3,1))
    track_dmp2 = np.zeros(shape=(3,1))
    
    while True:
        k_step = k_step + 1
        position = pose[0:3]
        vel = pose[3:6]
        orientation_q = pose[6:10]
        omega = pose[10:13]
        d = h_ceiling - position[2]
        vz, vx = vel[2], vel[0]
        RREV, omega_y = vz/d, vx/d

        if (time.time() - start_time) < 0.2:
            pose = env.keep_velocity(vx = vx_ini, vz = vz_ini)
        else:
            if dmp1_flag:
                if start_time_dmp1 is None:
                    print('------------- dmp 1 starts -------------')
                    print( 'vz=%.3f, vx=%.3f, RREV=%.3f, d=%.3f' %(vz, vx, RREV, d) )
                    dmp1_traj_duration = dmp1.params['tau']*30
                    dmp1_traj_ini = [RREV,0]
                    dmp1_traj_tau = dmp1.params['tau']*30
                    dmp1_traj_goal = dmp1.y_demo[1,-1]
                    dmp1_traj = dmp1.transformationSystem(dmp1_traj_duration, dmp1_traj_ini, dmp1_traj_tau, dmp1_traj_goal)

                    start_time_dmp1 = rospy.get_time()
                    
                    # env.mavros_offb.vel_thread_done = True
                                    
                time_dmp1 = rospy.get_time() - start_time_dmp1
                if k_step%10 == 0:
                    print('simulation time is %.3f, dmp1 time is %.3f' %(rospy.get_time(),time_dmp1) )
                
                if time_dmp1 < dmp1_traj_duration:
                    index = int(time_dmp1*5000)
                    RREV_d = dmp1_traj[1,index]
                    # vz_d = d*RREV_d
                    vz_d = 50*(1-RREV/RREV_d)
                    action = {'dmp':1, 'vz_d':vz_d, 'vx_d':vx_ini}
                    if k_step%10 == 0:
                        print('dmp1...: index=%d, RREV_d=%.3f, RREV=%.3f, vz_d=%.3f, vz=%.3f'
                        %(index, RREV_d, RREV, vz_d, vz) )
                        track_dmp1 = np.append(track_dmp1, [[time_dmp1],[RREV_d],[RREV]], axis=1)
                else:
                    dmp1_flag = False
                
            else:
                if start_time_dmp2 is None:
                    print('------------- dmp 2 starts -------------')
                    print( 'vz=%.3f, vx=%.3f, RREV=%.3f, d=%.3f' %(vz, vx, RREV, d) )
                    dmp2_traj_duration = dmp2.params['tau']*5
                    dmp2_traj_ini = [0,0]
                    dmp2_traj_tau = dmp2.params['tau']*5
                    dmp2_traj_goal = dmp2.y_demo[1,-1]
                    dmp2_traj = dmp2.transformationSystem(dmp2_traj_duration, dmp2_traj_ini, dmp2_traj_tau, dmp2_traj_goal)

                    start_time_dmp2 = rospy.get_time()
                
                time_dmp2 = rospy.get_time() - start_time_dmp2
                if k_step%10 == 0:
                    print('simulation time is %.3f, dmp2 time is %.3f' %(rospy.get_time(),time_dmp2) )

                if time_dmp2 < dmp2_traj_duration:
                    index = int(time_dmp2*5000)
                    q_d = -dmp2_traj[2,index]
                    action = {'dmp':2, 'q_d':q_d }
                    if k_step%10 == 0:
                        print('dmp2...: index=%d, q_d=%.3f, q=%.3f' %(index, q_d, omega[1]) )
                else:
                    done = True

            pose_, reward, _, info = env.step(action)

            pose = pose_

        if done: #or (time.time() - start_time) > 60 or time_dmp2 > 15:
            rospy.loginfo('Reward: ' + str(np.around(reward,3)))
            rospy.loginfo('Distance: ' + str(np.around(info[0],3)) + ' --- Angle: ' + str(np.around(info[1],3)) )
            # if not choose_action_flag:
            #     theta = agent.learning(state=state, action=action, reward=reward)
            # rospy.loginfo('Theta: '+ str(np.around(theta,3)) )
            
            if done:
                rospy.loginfo("====== "+'Episode #' + str(k_ep) + ' DONE'+" =========")
            elif (time.time() - start_time) > 60:
                rospy.loginfo("==========="+' Episode OVERTIME 60s '+"=============")
            else:
                rospy.loginfo("==========="+' Episode OVERTIME 15s '+"============")
            
            # env.add_record_xls(file_log=file_log, sheet=sheet, file_name=file_name,
            #     k_ep=k_ep, start_time1=start_time11, start_time2=start_time12,
            #     vz_ini=vz_ini, vx_ini=vx_ini, state=state, action=action[0],
            #     reward=reward, info=info, theta=theta)

            plt.figure()
            #plt.hold()
            plt.plot(track_dmp1[0,:],track_dmp1[1,:], track_dmp1[0,:],track_dmp1[2,:])
            plt.show()

            break      

        
