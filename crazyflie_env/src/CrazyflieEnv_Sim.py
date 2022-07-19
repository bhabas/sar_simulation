#!/usr/bin/env python3
import numpy as np
import warnings
import gym
from gym import logger,spaces

import os
import time
import sys
import subprocess
import rospy
from CrazyflieEnv_Base import CrazyflieEnv_Base

## ROS MESSAGES AND SERVICES
from std_srvs.srv import Empty
from crazyflie_msgs.srv import domainRand,domainRandRequest


from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty,EmptyRequest

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
ENDC = '\033[m'

class CrazyflieEnv_Sim(CrazyflieEnv_Base):
    metadata = {'render.modes': ['human']}
    def __init__(self,gazeboTimeout=False):
        super(CrazyflieEnv_Sim, self).__init__()        

        self.env_name = "CF_Gazebo"
        self.k_ep = 0
        self.Flip_thr = 2.0
        
        
        
        self.d_min = 50.0
        self.Tau_trg = 50.0
        self.done = False

        high = np.array(
            [
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
            ],
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-5,-1]), high=np.array([5,1]), shape=(2,), dtype=np.float32)
        self.My_space = spaces.Box(low=np.array([-0e-3]), high=np.array([-8e-3]), shape=(1,), dtype=np.float32)

        ## GAZEBO SIMULATION INITIALIZATION            
        self.gazeboTimeout = gazeboTimeout
        rospy.init_node("crazyflie_env_node")

        ## LAUNCH GAZEBO
        self.launch_Gazebo() 
        rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
        
        ## LAUNCH CONTROLLER
        self.launch_Node_Controller()
        rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=5)

        ## LAUNCH CF_DC
        self.launch_Node_CF_DC()
        rospy.wait_for_service("/CF_DC/Cmd_CF_DC",timeout=5)

        print("[INITIATING] Gazebo simulation started")
        

    def step(self,action):

        Tau,OFy,d_ceil  = self.obs
        

        ## START IMPACT TERMINATION TIMERS
        if ((self.impact_flag or self.BodyContact_flag) and self.onceFlag_impact == False):
            self.start_time_impact = self.getTime()
            self.onceFlag_impact = True

        if action[0] < self.Flip_thr:

            ## UPDATE STATE
            self.iter_step()

            ## UPDATE OBSERVATION
            self.obs = (self.Tau,self.OFy,self.d_ceil)

            ## CHECK FOR DONE
            self.done = bool(
                self.t - self.start_time_rollout > 2.0                # EPISODE TIMEOUT
                or self.t - self.start_time_impact > 0.5            # IMPACT TIMEOUT
                or (self.velCF[2] <= -0.5 and self.posCF[2] <= 1.5) # FREE-FALL TERMINATION
            )         

            if not self.done:
                if self.d_ceil <= self.d_min:
                    self.d_min = self.d_ceil 

            ## ERROR TERMINATIONS
            if (time.time() - self.start_time_ep) > 120.0 and self.gazeboTimeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.Restart()
                self.done = True

            if any(np.isnan(self.velCF)): 
                print('\033[93m' + "[WARNING] NaN in State Vector" + '\x1b[0m')
                self.Restart([True,True,True])
                print('\033[93m' + "[WARNING] Resuming Flight" + '\x1b[0m')
                self.done = True

            reward = 0

        elif action[0] >= self.Flip_thr:

            # self.Once_flag = True
            self.Tau_trg = Tau
            reward = self.finish_sim(action)
            self.done = True
        
        return np.array(self.obs,dtype=np.float32), reward, self.done, {}

    def finish_sim(self,action):

        ## CONVERT ACTION RANGE TO MOMENT RANGE
        action_scale = (self.My_space.high[0]-self.My_space.low[0])/(self.action_space.high[1]-self.action_space.low[1])
        My = (action[1]-self.action_space.low[1])*action_scale + self.My_space.low[0]
        # My = -7.3e-3

        self.SendCmd("Moment",[0,My*1e3,0],cmd_flag=1)
        self.gazebo_unpause_physics()

        while not self.done:
            ## START IMPACT TERMINATION TIMERS
            if ((self.impact_flag or self.BodyContact_flag) and self.onceFlag_impact == False):
                self.start_time_impact = self.getTime()
                self.onceFlag_impact = True

            self.done = bool(
                self.t - self.start_time_rollout > 2.0                # EPISODE TIMEOUT
                or self.t - self.start_time_impact > 0.5            # IMPACT TIMEOUT
                or (self.velCF[2] <= -0.5 and self.posCF[2] <= 1.5) # FREE-FALL TERMINATION
            )

            if not self.done:
                if self.d_ceil <= self.d_min:
                    self.d_min = self.d_ceil 

            ## ERROR TERMINATIONS
            if (time.time() - self.start_time_ep) > 20.0 and self.gazeboTimeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.Restart()
                self.done = True

            if any(np.isnan(self.velCF)): 
                print('\033[93m' + "[WARNING] NaN in State Vector" + '\x1b[0m')
                self.Restart([True,False,False])
                print('\033[93m' + "[WARNING] Resuming Flight" + '\x1b[0m')
                self.done = True
        
        return self.CalcReward()

    def iter_step(self,n_steps:int = 10):
        os.system(f'gz world --multi-step={n_steps}')

    def reset(self):

        self.gazebo_unpause_physics()
        ## DISABLE STICKY LEGS (ALSO BREAKS CURRENT CONNECTION JOINTS)
        self.SendCmd('Tumble',cmd_flag=0)
        self.SendCmd('StickyPads',cmd_flag=0)

        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.sleep(0.01)

        self.SendCmd('Tumble',cmd_flag=1)
        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.SendCmd('Tumble',cmd_flag=1)
        self.sleep(1.0)
        self.SendCmd('StickyPads',cmd_flag=1)

        self.gazebo_pause_physics()

        ## RESET REWARD CALC VALUES
        self.done = False
        self.d_min = 50.0  # Reset max from ceiling [m]
        self.Tau_trg = 50.0

        ## RESET/UPDATE RUN CONDITIONS
        self.start_time_rollout = self.getTime()
        self.start_time_pitch = np.nan
        self.start_time_impact = np.nan

        self.start_time_ep = time.time()

        ## RESET LOGGING CONDITIONS 
        self.onceFlag_flip = False    # Ensures flip data recorded only once
        self.onceFlag_impact = False   # Ensures impact data recorded only once 

        ## RESET STATE
        vel = np.random.uniform(low=1.5,high=3.5)
        phi = np.random.uniform(low=30,high=90)

        vx_0 = vel*np.cos(np.deg2rad(phi))
        vz_0 = vel*np.sin(np.deg2rad(phi))

        ## RESET OBSERVATION
        Tau_0 = 0.4
        d_ceil_0 = Tau_0*vz_0 + 1e-3

        z_0 = self.h_ceiling - d_ceil_0
        x_0 = 0.0
        self.Vel_Launch([x_0,0.0,z_0],[vx_0,0,vz_0])
        self.iter_step(10)


        ## RESET OBSERVATION
        self.obs = (self.Tau,self.OFy,self.d_ceil)
        self.k_ep += 1

        return np.array(self.obs,dtype=np.float32)


    def ParamOptim_reset(self):

        ## RESET REWARD CALC VALUES
        self.done = False
        self.d_min = 50.0  # Reset max from ceiling [m]

        ## DISABLE STICKY LEGS (ALSO BREAKS CURRENT CONNECTION JOINTS)
        self.SendCmd('Tumble',cmd_flag=0)
        self.SendCmd('StickyPads',cmd_flag=0)

        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.SendCmd('Ctrl_Reset')
        self.reset_pos()

        self.SendCmd('Tumble',cmd_flag=1)
        self.sleep(0.25)

        obs = None
        return obs

    def ParamOptim_Flight(self,Tau,My,vel,phi):

        ## RESET/UPDATE RUN CONDITIONS
        start_time_rollout = self.getTime()
        start_time_pitch = np.nan
        start_time_impact = np.nan

        start_time_ep = time.time()

        ## RESET LOGGING CONDITIONS 
        onceFlag_flip = False    # Ensures flip data recorded only once
        onceFlag_impact = False   # Ensures impact data recorded only once 


        self.SendCmd('StickyPads',cmd_flag=1)

        vz = vel*np.sin(np.deg2rad(phi))
        vx = vel*np.cos(np.deg2rad(phi))

        tau_0 = 0.5
        z_0 = self.h_ceiling - tau_0*vz

        self.Vel_Launch([0,0,z_0],[vx,0,vz])
        self.sleep(0.05)
        self.SendCmd("Policy",cmd_vals=[Tau,My,0.0],cmd_flag=1)


        

        while not self.done: 

            t_now = self.getTime()

            ## START FLIP AND IMPACT TERMINATION TIMERS
            if (self.flip_flag == True and onceFlag_flip == False):
                start_time_pitch = t_now # Starts countdown for when to reset run
                onceFlag_flip = True # Turns on to make sure this only runs once per rollout

            if ((self.impact_flag or self.BodyContact_flag) and onceFlag_impact == False):
                start_time_impact = t_now
                onceFlag_impact = True

            # ============================
            ##    Termination Criteria 
            # ============================

            ## PITCH TIMEOUT  
            if (t_now-start_time_pitch) > 2.25:
                self.error_str = "Rollout Completed: Pitch Timeout"
                self.done = True
                # print(self.error_str)


            ## IMPACT TIMEOUT
            elif (t_now-start_time_impact) > 0.5:
                self.error_str = "Rollout Completed: Impact Timeout"
                self.done = True
                # print(self.error_str)



            ## ROLLOUT TIMEOUT
            elif (t_now - start_time_rollout) > 5.0:
                self.error_str = "Rollout Completed: Time Exceeded"
                self.done = True
                # print(self.error_str)

            ## FREE FALL TERMINATION
            elif self.velCF[2] <= -0.5 and self.posCF[2] <= 1.5: 
                self.error_str = "Rollout Completed: Falling Drone"
                self.done = True
                # print(self.error_str)



            if (time.time() - start_time_ep) > 15.0 and self.gazeboTimeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.Restart()
                self.done = True


            if any(np.isnan(self.velCF)): 
                self.error_str = "Rollout Completed: NaN in State Vector"

                print('\033[93m' + "[WARNING] NaN in State Vector" + '\x1b[0m')
                self.Restart([True,True,True])
                print('\033[93m' + "[WARNING] Resuming Flight" + '\x1b[0m')
                self.done = True
                # print(self.error_str)

        reward = self.CalcReward()

        return None,reward,self.done,None

    def CalcReward(self):

        R0 = np.clip(1/np.abs(self.Tau_trg-0.2),0,20)/20
        R0 *= 0.1

        ## DISTANCE REWARD 
        R1 = np.clip(1/np.abs(self.d_min+1e-3),0,10)/10
        R1 *= 0.05

        ## IMPACT ANGLE REWARD
        R2 = np.clip(np.abs(self.eulCF_impact[1])/120,0,1)
        R2 *= 0.2

        ## PAD CONTACT REWARD
        if self.pad_connections >= 3: 
            if self.BodyContact_flag == False:
                R3 = 0.65
            else:
                R3 = 0.4
        elif self.pad_connections == 2: 
            if self.BodyContact_flag == False:
                R3 = 0.2
            else:
                R3 = 0.1
        else:
            R3 = 0.0

        return R0 + R1 + R2 + R3
        
    def callService(self,addr,srv,srv_type,retries=5):

        FailureModes = self.diagnosticTest()

        if any(FailureModes):
            self.Restart(FailureModes)
            self.done = True

        for retry in range(retries):
            try:
                service = rospy.ServiceProxy(addr, srv_type)
                service(srv)
                return True

            except rospy.ServiceException as e:
                print(f"[WARNING] {addr} service call failed (callService)")
                print(f"[WARNING] {e}")
                FailureModes[2] = False

                self.Restart(FailureModes)
                self.done = True
        
        return False

    def diagnosticTest(self):
        FailureModes = [False,False,False]
        sys.stdout.write(YELLOW)
        
        ## CHECK THAT GAZEBO IS FUNCTIONING
        try:
            rospy.wait_for_service("/gazebo/pause_physics",timeout=1)
        except rospy.ROSException as e:
            print(f"[WARNING] /gazebo/pause_physics wait for service failed (callService)")
            print(f"[WARNING] {e}")
            FailureModes[0] = True

        ## CHECK THAT CONTROLLER IS FUNCTIONING
        try:
            rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=1)
        except rospy.ROSException as e:
            print(f"[WARNING] /CTRL/Cmd_ctrl wait for service failed (callService)")
            print(f"[WARNING] {e}")
            FailureModes[1] = True

        ## CHECK THAT CF_DC IS FUNCTIONING
        try:
            rospy.wait_for_service('/CF_DC/Cmd_CF_DC',timeout=1)
        except rospy.ROSException as e:
            print(f"[WARNING] /CF_DC/Cmd_CF_DC wait for service failed (callService)")
            print(f"[WARNING] {e}")
            FailureModes[2] = True

        sys.stdout.write(ENDC)
        return FailureModes

    def Restart(self,FailureModes=[True,False,False]):
        sys.stdout.write(YELLOW)

        if FailureModes == [True,False,False]:
            for retry in range(3):
                try:
                    print(f"[WARNING] Gazebo Restart. Attempt: {retry+1}/3 (restart_Gazebo)")
                    self.launch_Gazebo()
                    rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
                    print(GREEN+ f"[WARNING] Gazebo restart successful. (restart_Gazebo)" + ENDC)
                    return True

                except rospy.ROSException as e:
                    print(f"[WARNING] Gazebo restart Failed. (restart_Gazebo)")
                    print(f"[WARNING] {e}")
            
        sys.stdout.write(RED)
        for retry in range(5):
            try:
                print(f"[WARNING] Full Simulation Restart. Attempt: {retry+1}/5 (Restart Sim)")

                ## KILL EVERYTHING

                os.system("killall -9 gzserver gzclient")
                os.system("rosnode kill /gazebo /gazebo_gui /Controller_Node /CF_DataConverter_Node")
                time.sleep(2.0)
                
                subprocess.Popen( # Gazebo Process
                    "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
                    start_new_session=True, shell=True)
                rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
        
                ## LAUNCH CONTROLLER
                subprocess.Popen( # Controller Process
                    "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_control controller",
                    close_fds=True, preexec_fn=os.setsid, shell=True)
                rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=5)

                ## LAUNCH CF_DC
                subprocess.Popen( # CF_DC Process
                    "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_data_converter CF_DataConverter",
                    close_fds=True, preexec_fn=os.setsid, shell=True)
                rospy.wait_for_service("/CF_DC/Cmd_CF_DC",timeout=5)

                ## UNPAUSE GAZEBO
                rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
                print(GREEN + f"[WARNING] Gazebo restart successful. (Restart Sim)" + ENDC)

                return True

            except rospy.ROSException as e:
                print(f"[WARNING] Gazebo restart Failed. (Restart Sim)")
                print(f"[WARNING] {e}")

            print('\x1b[0m')

    def sleep(self,time_s):
        """
        Sleep in terms of Gazebo sim seconds not real time seconds
        """

        t_start = self.t
        while self.t - t_start <= time_s:
            # print("Sleeping....")
            FailureModes = self.diagnosticTest()
            if any(FailureModes):
                self.Restart(FailureModes)
                self.done = True
                return False

            ## NEGATIVE TIME DELTA
            if self.t < t_start:
                self.done = True
                return False

        return True
        
    def Vel_Launch(self,pos_0,vel_d,quat_0=[0,0,0,1]): 
        """Launch crazyflie from the specified position/orientation with an imparted velocity.
        NOTE: Due to controller dynamics, the actual velocity will NOT be exactly the desired velocity

        Args:
            pos_0 (list): Launch position [m]   | [x,y,z]
            vel_d (list): Launch velocity [m/s] | [Vx,Vy,Vz]
            quat_0 (list, optional): Orientation at launch. Defaults to [0,0,0,1].
        """        

        ## SET DESIRED VEL IN CONTROLLER
        self.gazebo_pause_physics()
        self.SendCmd('Pos',cmd_flag=0)
        self.iter_step(2)
        self.SendCmd('Vel',cmd_vals=vel_d,cmd_flag=1)
        self.iter_step(2)
        

        ## CREATE SERVICE MESSAGE
        state_srv = ModelState()
        state_srv.model_name = self.modelName

        ## INPUT POSITION AND ORIENTATION
        state_srv.pose.position.x = pos_0[0]
        state_srv.pose.position.y = pos_0[1]
        state_srv.pose.position.z = pos_0[2]

        state_srv.pose.orientation.x = quat_0[0]
        state_srv.pose.orientation.y = quat_0[1]
        state_srv.pose.orientation.z = quat_0[2]
        state_srv.pose.orientation.w = quat_0[3]

        ## INPUT LINEAR AND ANGULAR VELOCITY
        state_srv.twist.linear.x = vel_d[0]
        state_srv.twist.linear.y = vel_d[1]
        state_srv.twist.linear.z = vel_d[2]

        state_srv.twist.angular.x = 0
        state_srv.twist.angular.y = 0
        state_srv.twist.angular.z = 0

        ## PUBLISH MODEL STATE SERVICE REQUEST
        self.callService('/gazebo/set_model_state',state_srv,SetModelState)
        # self.gazebo_unpause_physics()

    def reset_pos(self,z_0=0.358): # Disable sticky then places spawn_model at origin
        """Reset pose/twist of simulated drone back to home position. 
        As well as turning off stickyfeet

        Args:
            z_0 (float, optional): Starting height of crazyflie. Defaults to 0.379.
        """        
        
        ## RESET POSITION AND VELOCITY
        state_srv = ModelState()
        state_srv.model_name = self.modelName
        state_srv.pose.position.x = 0.0
        state_srv.pose.position.y = 0.0
        state_srv.pose.position.z = z_0

        state_srv.pose.orientation.w = 1.0
        state_srv.pose.orientation.x = 0.0
        state_srv.pose.orientation.y = 0.0
        state_srv.pose.orientation.z = 0.0
        
        state_srv.twist.linear.x = 0.0
        state_srv.twist.linear.y = 0.0
        state_srv.twist.linear.z = 0.0

        state_srv.twist.angular.x = 0.0
        state_srv.twist.angular.y = 0.0
        state_srv.twist.angular.z = 0.0

        self.callService('/gazebo/set_model_state',state_srv,SetModelState)

    def updateInertia(self):

        ## CREATE SERVICE REQUEST MSG
        srv = domainRandRequest() 
        srv.mass = self.mass
        srv.Inertia.x = self.Ixx
        srv.Inertia.y = self.Iyy
        srv.Inertia.z = self.Izz

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/CF_Internal/DomainRand',srv,domainRand)



    

    # ============================
    ##      GAZEBO TECHNICAL
    # ============================
    def launch_Gazebo(self):
        """ Launches Gazebo environment with crazyflie drone
        """        

        print("[STARTING] Starting Gazebo Process...")
        term_command = "rosnode kill /gazebo /gazebo_gui"
        os.system(term_command)
        time.sleep(1.0)

        term_command = "killall -9 gzserver gzclient"
        os.system(term_command)
        time.sleep(1.0)
        
        subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
            start_new_session=True, shell=True)

    def launch_Node_Controller(self):
        """ 
        Kill previous controller node if active and launch controller node
        """        
        
        print("[STARTING] Starting Controller Process...")
        os.system("rosnode kill /Controller_Node")
        time.sleep(0.5)
        subprocess.Popen( # Controller Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_control controller",
            close_fds=True, preexec_fn=os.setsid, shell=True)

    def launch_Node_CF_DC(self):
        """ 
        Kill previous CF_DC node if active and launch CF_DC node
        """        
        
        print("[STARTING] Starting CF_DC Process...")
        os.system("rosnode kill /CF_DataConverter_Node")
        time.sleep(0.5)
        subprocess.Popen( # CF_DC Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_data_converter CF_DataConverter",
            close_fds=True, preexec_fn=os.setsid, shell=True)

    def gazebo_pause_physics(self):
        srv = EmptyRequest()
        self.callService("/gazebo/pause_physics",srv,Empty)

    def gazebo_unpause_physics(self):
        srv = EmptyRequest()
        self.callService("/gazebo/unpause_physics",srv,Empty)
        


if __name__ == "__main__":

    # env = CrazyflieEnv(gazeboTimeout=True)

    # for ii in range(1000):
    #     tau = np.random.uniform(low=0.15,high=0.27)
    #     env.ParamOptim_reset()
    #     obs,reward,done,info = env.ParamOptim_Flight(0.23,7,2.5,60)
    #     print(f"Ep: {ii} \t Reward: {reward:.02f}")

    env = CrazyflieEnv_Sim(gazeboTimeout=False)
    for ep in range(25):
        env.reset()
        done = False
        while not done:
            obs,reward,done,info = env.step(env.action_space.sample())
        print(f"Episode: {ep} \t Reward: {reward:.3f}")

