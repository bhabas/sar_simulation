#!/usr/bin/env python3
import numpy as np
from gym import spaces
import rospy
import time


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))

from crazyflie_env import CrazyflieEnv_Sim


class CrazyflieEnv_DeepRL_LDA(CrazyflieEnv_Sim):
    metadata = {'render.modes': ['human']}
    def __init__(self,GZ_Timeout=True,Vel_range=[1.5,3.5],Phi_range=[0,90],Tau_0=0.4):
        """_summary_

        Args:
            GZ_Timeout (bool, optional): Determines if Gazebo will restart if it freezed. Defaults to False.
            Vel_range (list, optional): Range of flight velocities. Defaults to [1.5,3.5].
            Phi_range (list, optional): Range of flight angles. Defaults to [0,90].
            Tau_0 (float, optional): Flight position will start at this Tau value. Defaults to 0.4.
        """        
        CrazyflieEnv_Sim.__init__(self)          

        ## ENV CONFIG SETTINGS
        self.env_name = "CF_Gazebo"
        self.GZ_Timeout = GZ_Timeout

        ## TESTING CONDITIONS
        self.Tau_0 = Tau_0          
        self.Vel_range = Vel_range  
        self.Phi_range = Phi_range  

        ## RESET INITIAL VALUES
        self.k_ep = 0
        self.Flip_thr = 1.5
        self.D_min = 50.0
        self.Tau_trg = 50.0

        self.done = False

        ## DEFINE OBSERVATION SPACE
        obs_lim = np.array( 
            [
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
            ],
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(low=-obs_lim, high=obs_lim, dtype=np.float32)

        ## DEFINE ACTION SPACE
        self.action_space = spaces.Box(low=np.array([-1,0]), high=np.array([1,8]), shape=(2,), dtype=np.float32)

    def step(self,action):

        Tau,Theta_x,D_perp  = self.obs
        action[0] = np.arctanh(action[0])
        
        if action[0] < self.Flip_thr:

            ## UPDATE STATE AND OBSERVATION
            self.iter_step()
            self.obs = (self.Tau,self.Theta_x,self.D_perp)

            ## CHECK FOR DONE
            self.done = bool(
                self.t - self.start_time_rollout > 0.7              # EPISODE TIMEOUT
                or (self.impact_flag or self.BodyContact_flag)
                or (self.vel[2] <= -0.5 and self.pos[2] <= 1.5) # FREE-FALL TERMINATION
            )         

            if not self.done:
                if self.D_perp <= self.D_min:
                    self.D_min = self.D_perp 

            ## ERROR TERMINATIONS
            if (time.time() - self.start_time_ep) > 300.0 and self.GZ_Timeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.Restart()
                self.done = True

            if any(np.isnan(self.vel)): 
                print('\033[93m' + "[WARNING] NaN in State Vector" + '\x1b[0m')
                self.Restart([True,True,True])
                print('\033[93m' + "[WARNING] Resuming Flight" + '\x1b[0m')
                self.done = True

            reward = 0

        elif action[0] >= self.Flip_thr:

            self.Tau_trg = Tau
            reward = self.finish_sim(action)
            self.done = True
        
        return np.array(self.obs,dtype=np.float32), reward, self.done, {}

    def finish_sim(self,action):

        My = -action[1]

        self.SendCmd("Moment",[0,My,0],cmd_flag=1)
        self.gazebo_unpause_physics()

        while not self.done:
            ## START IMPACT TERMINATION TIMERS
            if ((self.impact_flag or self.BodyContact_flag) and self.onceFlag_impact == False):
                self.start_time_impact = self.getTime()
                self.onceFlag_impact = True

            self.done = bool(
                self.t - self.start_time_rollout > 1.0              # EPISODE TIMEOUT
                or self.t - self.start_time_impact > 0.5            # IMPACT TIMEOUT
                or (self.vel[2] <= -0.5 and self.pos[2] <= 1.5) # FREE-FALL TERMINATION
            )

            if not self.done:
                if self.D_perp <= self.D_min:
                    self.D_min = self.D_perp 

            ## ERROR TERMINATIONS
            if (time.time() - self.start_time_ep) > 300.0 and self.GZ_Timeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.Restart()
                self.done = True

            if any(np.isnan(self.vel)): 
                print('\033[93m' + "[WARNING] NaN in State Vector" + '\x1b[0m')
                self.Restart([True,False,False])
                print('\033[93m' + "[WARNING] Resuming Flight" + '\x1b[0m')
                self.done = True
        
        return self.CalcReward()


    def reset(self):

        self.gazebo_unpause_physics()
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

        ## DOMAIN RANDOMIZATION (UPDATE INERTIA VALUES)
        self.Iyy = rospy.get_param(f"/CF_Type/{self.CF_Type}/Config/{self.CF_Config}/Iyy") + np.random.normal(0,1.5e-6)
        self.mass = rospy.get_param(f"/CF_Type/{self.CF_Type}/Config/{self.CF_Config}/Mass") + np.random.normal(0,0.0005)
        self.updateInertia()



        ## RESET REWARD CALC VALUES
        self.done = False
        self.D_min = 50.0  # Reset max from ceiling [m]
        self.Tau_trg = 50.0

        ## RESET/UPDATE RUN CONDITIONS
        self.start_time_rollout = self.getTime()
        self.start_time_pitch = np.nan
        self.start_time_impact = np.nan

        self.start_time_ep = time.time()

        ## RESET LOGGING CONDITIONS 
        self.onceFlag_flip = False      # Ensures flip data recorded only once
        self.onceFlag_impact = False    # Ensures impact data recorded only once 

        ## SAMPLE VELOCITY VECTOR
        vel = np.random.uniform(low=self.Vel_range[0],high=self.Vel_range[1])
        phi = np.random.uniform(low=self.Phi_range[0],high=self.Phi_range[1])

        vx_0 = vel*np.cos(np.deg2rad(phi))
        vz_0 = vel*np.sin(np.deg2rad(phi))
        Vel_0 = np.array([vx_0,0,vz_0])  # Flight Velocity vector

        
        ## RESET POSITION (Derivation: Research_Notes_Book_2.pdf (12/30/22))
        r_p = np.array(self.Plane_Pos)                              # Plane Position
        theta_rad = np.radians(self.Plane_Angle)                    # Plane angle
        n_hat = np.array([np.sin(theta_rad),0,-np.cos(theta_rad)])  # Plane normal vector

        D_perp_0 = self.Tau_0*(Vel_0.dot(n_hat)) # Initial distance
        r_0 = r_p - D_perp_0*n_hat          # Initial quad position (World coords)

        self.Vel_Launch(r_0,Vel_0)
        self.iter_step(10)


        ## RESET OBSERVATION
        self.obs = (self.Tau,self.Theta_x,self.D_perp)
        self.k_ep += 1

        return np.array(self.obs,dtype=np.float32)



    def CalcReward(self):

        ## TAU TRIGGER REWARD
        R0 = np.clip(1/np.abs(self.Tau_trg-0.2),0,15)/15
        R0 *= 0.1

        ## DISTANCE REWARD 
        R1 = np.clip(1/np.abs(self.D_min+1e-3),0,15)/15
        R1 *= 0.05

        ## IMPACT ANGLE REWARD
        # R2 = np.clip(np.abs(self.eulCF_impact[1])/120,0,1)
        # R2 *= 0.2

        R2 = 0.5*np.cos(self.eul_impact.y-self.Plane_Angle_rad*np.sign(np.cos(self.Plane_Angle_rad)))+0.5
        # R2 = 0.5*np.cos(self.eulCF_impact[1]-self.Plane_Angle_rad)+0.5
        R2 *= 0.2


        ## PAD CONTACT REWARD
        if self.pad_connections >= 3: 
            if self.BodyContact_flag == False:
                R3 = 0.65
            else:
                R3 = 0.2
        elif self.pad_connections == 2: 
            if self.BodyContact_flag == False:
                R3 = 0.4
            else:
                R3 = 0.1
        else:
            R3 = 0.0

        return R0 + R1 + R2 + R3

    def render(self):
        pass
        
    

if __name__ == "__main__":

    # vel = 2
    # phi = 45

    env = CrazyflieEnv_DeepRL_LDA(GZ_Timeout=False)
    for ep in range(25):
        env.reset()
        done = False
        while not done:
            action = env.action_space.sample()
            action = np.array([0,0])
            obs,reward,done,info = env.step(action)
        print(f"Episode: {ep} \t Reward: {reward:.3f}")

