#!/usr/bin/env python3
import numpy as np
from gym import spaces
import rospy
import time
from Core_Envs.CrazyflieEnv_Sim import CrazyflieEnv_Sim


class CrazyflieEnv_DeepRL(CrazyflieEnv_Sim):
    metadata = {'render.modes': ['human']}
    def __init__(self,GZ_Timeout=False):
        CrazyflieEnv_Sim.__init__(self)          

        self.env_name = "CF_Gazebo"
        self.GZ_Timeout = GZ_Timeout
        self.k_ep = 0
        self.Flip_thr = 1.5

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
        self.action_space = spaces.Box(low=np.array([-1,0]), high=np.array([1,8]), shape=(2,), dtype=np.float32)

    def step(self,action):

        Tau,OFy,d_ceil  = self.obs
        action[0] = np.arctanh(action[0])
        
        if action[0] < self.Flip_thr:

            ## UPDATE STATE AND OBSERVATION
            self.iter_step()
            self.obs = (self.Tau,self.OFy,self.d_ceil)

            ## CHECK FOR DONE
            self.done = bool(
                self.t - self.start_time_rollout > 0.7                # EPISODE TIMEOUT
                or (self.impact_flag or self.BodyContact_flag)
                or (self.velCF[2] <= -0.5 and self.posCF[2] <= 1.5) # FREE-FALL TERMINATION
            )         

            if not self.done:
                if self.d_ceil <= self.d_min:
                    self.d_min = self.d_ceil 

            ## ERROR TERMINATIONS
            if (time.time() - self.start_time_ep) > 300.0 and self.GZ_Timeout == True:
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
                or (self.velCF[2] <= -0.5 and self.posCF[2] <= 1.5) # FREE-FALL TERMINATION
            )

            if not self.done:
                if self.d_ceil <= self.d_min:
                    self.d_min = self.d_ceil 

            ## ERROR TERMINATIONS
            if (time.time() - self.start_time_ep) > 300.0 and self.GZ_Timeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.Restart()
                self.done = True

            if any(np.isnan(self.velCF)): 
                print('\033[93m' + "[WARNING] NaN in State Vector" + '\x1b[0m')
                self.Restart([True,False,False])
                print('\033[93m' + "[WARNING] Resuming Flight" + '\x1b[0m')
                self.done = True
        
        return self.CalcReward()


    def reset(self,vel=None,phi=None):

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

        ## DOMAIN RANDOMIZATION (UPDATE INERTIA VALUES)
        self.Iyy = rospy.get_param("/Iyy") + np.random.normal(0,1.5e-6)
        self.mass = rospy.get_param("/CF_Mass") + np.random.normal(0,0.0005)
        self.updateInertia()


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
        if vel == None:
            vel = np.random.uniform(low=1.5,high=3.5)
            
        if phi == None:
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



    def CalcReward(self):

        ## TAU TRIGGER REWARD
        R0 = np.clip(1/np.abs(self.Tau_trg-0.2),0,15)/15
        R0 *= 0.1

        ## DISTANCE REWARD 
        R1 = np.clip(1/np.abs(self.d_min+1e-3),0,15)/15
        R1 *= 0.05

        ## IMPACT ANGLE REWARD
        R2 = np.clip(np.abs(self.eulCF_impact[1])/120,0,1)
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

    env = CrazyflieEnv_DeepRL(GZ_Timeout=False)
    for ep in range(25):
        env.reset()
        done = False
        while not done:
            obs,reward,done,info = env.step(env.action_space.sample())
        print(f"Episode: {ep} \t Reward: {reward:.3f}")

