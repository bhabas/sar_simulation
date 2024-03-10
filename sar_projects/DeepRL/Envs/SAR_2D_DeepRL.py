import gymnasium as gym
import numpy as np
from gymnasium import spaces

import time
import math
import pygame as pg
import os
import rospy


YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = '\033[34m'  
RESET = '\033[0m'  # Reset to default color

from sar_env import SAR_2D_Sim_Interface

class SAR_2D_Env(SAR_2D_Sim_Interface,gym.Env):

    def __init__(self,Ang_Acc_range=[-100,100],V_mag_range=[1.5,3.5],V_angle_range=[5,175],Plane_Angle_range=[0,180],Render=True,GZ_Timeout=False):
        SAR_2D_Sim_Interface.__init__(self,Render)
        gym.Env.__init__(self)

        if self.Policy_Type != "DEEP_RL_SB3":
            str_input = self.userInput(YELLOW + "Incorrect Policy Activated. Continue? (y/n): " + RESET,str)
            if str_input.lower() == 'n':
                raise Exception('[ERROR] Incorrect Policy Type Activated')
            else:
                pass
        
        ######################
        #    GENERAL CONFIGS
        ######################
        
        ## ENV CONFIG SETTINGS
        self.Env_Name = "SAR_2D_DeepRL_Env"

        ## TESTING CONDITIONS     
        self.V_mag_range = V_mag_range  
        self.V_angle_range = V_angle_range
        self.Plane_Angle_range = Plane_Angle_range
        self.setAngAcc_range(Ang_Acc_range)


        ## TIME CONSTRAINTS
        self.t_rot_max = np.sqrt(np.radians(360)/np.max(np.abs(self.Ang_Acc_range))) # Allow enough time for a full rotation [s]
        self.t_impact_max = 2.0     # [s]
        self.t_ep_max = 5.0         # [s]
        self.t_real_max = 120       # [s]


        ## INITIAL LEARNING/REWARD CONFIGS
        self.Initial_Step = False
        self.K_ep = 0
        self.Pol_Trg_Threshold = 0.5
        self.Done = False
        self.reward = 0
        self.reward_vals = np.array([0,0,0,0,0,0])
        self.reward_weights = {
            "W_Dist":0.1,
            "W_tau_cr":0.1,
            "W_LT":1.0,
            "W_GM":1.0,
            "W_Phi_rel":2.0,
            "W_Legs":2.0
        }
        self.W_max = sum(self.reward_weights.values())

        self.D_perp_CR_min = np.inf
        self.Tau_CR_trg = np.inf
        self.Tau_trg = np.inf

        ## DOMAIN RANDOMIZATION
        self.Mass_std = 0.00*self.Ref_Mass
        self.Iyy_std = 0.00*self.Ref_Iyy

        ## DEFINE OBSERVATION SPACE
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.obs_trg = np.zeros(self.observation_space.shape,dtype=np.float32) # Obs values at triggering

        ## DEFINE ACTION SPACE
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.action_trg = np.zeros(self.action_space.shape,dtype=np.float32) # Action values at triggering

    

    def reset(self,seed=None,options=None):

        self.start_time_real = time.time()
        self.resetPose()
        self.Initial_Step = False
       
        return self._getObs(), {}
    
    def _resetParams(self):

        ######################
        #    GENERAL CONFIGS
        ######################

        ## RESET LEARNING/REWARD CONDITIONS
        self.K_ep += 1
        self.Done = False
        self.reward = 0
        self.reward_vals = np.array([0,0,0,0,0,0])

        self.D_perp_CR_min = np.inf
        self.Tau_CR_trg = np.inf
        self.Tau_trg = np.inf

        self.obs_trg = np.full(self.obs_trg.shape[0],np.nan)
        self.action_trg = np.full(self.action_trg.shape[0],np.nan)

        self.a_Trg_trg = np.nan
        self.a_Rot_trg = np.nan


        ##########   2D ENV CONFIGS  ##########
        #
        self.Trg_Flag = False
        self.State_trg = np.full(6,np.nan)
        self.State_Impact = np.full(6,np.nan)
        self.M_thrust_prev = np.full(4,self.Ref_Mass*self.g/4)

        self.V_B_P_impact_Ext = np.full(3,np.nan)
        self.Eul_B_O_impact_Ext = np.full(3,np.nan)
        self.Eul_P_B_impact_Ext = np.full(3,np.nan)
        self.Omega_B_P_impact_Ext = np.full(3,np.nan)
        self.Rot_Sum_impact_Ext = 0

        self.Impact_Flag_Ext = False
        self.BodyContact_Flag = False
        self.ForelegContact_Flag = False
        self.HindlegContact_Flag = False
        self.Pad_Connections = 0
        self.MomentCutoff = False
        #
        #######################################

    def _setTestingConditions(self):

        ## SAMPLE SET PLANE POSE
        Plane_Angle_Low = self.Plane_Angle_range[0]
        Plane_Angle_High = self.Plane_Angle_range[1]
        Plane_Angle = np.random.uniform(Plane_Angle_Low,Plane_Angle_High)
        self._setPlanePose(self.r_P_O,Plane_Angle)

        ## SAMPLE VELOCITY AND FLIGHT ANGLE
        V_mag,V_angle = self._sampleFlightConditions(self.V_mag_range,self.V_angle_range)
        self.V_mag = V_mag
        self.V_angle = V_angle
    
    def _initialStep(self):

        ## DOMAIN RANDOMIZATION (UPDATE INERTIAL VALUES)
        # NOTE: GZ sim updates base mass and inertia values
        Iyy = self.Ref_Iyy + np.random.normal(0,self.Iyy_std)
        Mass = self.Ref_Mass + np.random.normal(0,self.Mass_std)
        self._setModelInertia(Mass,[self.Ref_Ixx,Iyy,self.Ref_Izz])

        ## CALC STARTING VELOCITY IN GLOBAL COORDS
        V_tx = self.V_mag*np.cos(np.deg2rad(self.V_angle))
        V_perp = self.V_mag*np.sin(np.deg2rad(self.V_angle))
        V_B_P = np.array([V_tx,0,V_perp])               # {t_x,n_p}
        V_B_O = self.R_PW(V_B_P,self.Plane_Angle_rad)   # {X_W,Z_W}

        ## CALCULATE STARTING TAU VALUE
        self.Tau_CR_start = self.t_rot_max*np.random.uniform(1.9,2.1) # Add noise to starting condition
        self.Tau_Body_start = (self.Tau_CR_start + self.Collision_Radius/V_perp) # Tau read by body
        self.Tau_Accel_start = 1.0 # Acceleration time to desired velocity conditions [s]

        ## CALC STARTING POSITION IN GLOBAL COORDS
        # (Derivation: Research_Notes_Book_3.pdf (9/17/23))

        r_P_O = np.array(self.r_P_O)                                        # Plane Position wrt to Origin - {X_W,Z_W}
        r_P_B = np.array([(self.Tau_CR_start + self.Tau_Accel_start)*V_tx,
                          0,
                          (self.Tau_Body_start + self.Tau_Accel_start)*V_perp])  # Body Position wrt to Plane - {t_x,n_p}
        r_B_O = r_P_O - self.R_PW(r_P_B,self.Plane_Angle_rad)                   # Body Position wrt to Origin - {X_W,Z_W}

        ## LAUNCH QUAD W/ DESIRED VELOCITY
        self.initial_state = (r_B_O.copy(),V_B_O.copy())
        self.Sim_VelTraj(pos=r_B_O,vel=V_B_O)
        self._iterStep(n_steps=1000)

        ## RESET/UPDATE TIME CONDITIONS
        self.start_time_ep = self._getTime()
        self.start_time_trg = np.nan
        self.start_time_impact = np.nan
        self.t_flight_max = self.Tau_Body_start*2.0   # [s]
        self.t_trg_max = self.Tau_Body_start*3.0 # [s]

    def step(self, action):

        # 1. TAKE ACTION
        # 2. UPDATE STATE
        # 3. CALC REWARD
        # 4. CHECK TERMINATION
        # 5. RETURN VALUES

        if self.Initial_Step == False:
            self._resetParams()
            self._setTestingConditions()
            self._initialStep()
            self.Initial_Step = True

        a_Trg = action[0]
        a_Rot = self.scaleValue(action[1],original_range=[-1,1], target_range=self.Ang_Acc_range)

        ########## POLICY PRE-TRIGGER ##########
        if a_Trg <= self.Pol_Trg_Threshold:

            ## 2) UPDATE STATE
            self._iterStep(n_steps=10)
            t_now = self._getTime()

            # UPDATE RENDER
            self.render()

            # GRAB NEXT OBS
            next_obs = self._getObs()

            # 3) CALCULATE REWARD
            reward = 0.0

            # 4) CHECK TERMINATION/TRUNCATED

            # ============================
            ##    Termination Criteria 
            # ============================

            if self.Done == True:
                self.error_str = "Episode Completed: Done [Terminated]"
                terminated = True
                truncated = False
                # print(YELLOW,self.error_str,RESET)
            
            ## IMPACT TERMINATION
            elif self.Impact_Flag_Ext == True:
                self.error_str = "Episode Completed: Impact [Terminated]"
                terminated = True
                truncated = False
                # print(YELLOW,self.error_str,RESET)

            ## EPISODE TIMEOUT
            elif (t_now - self.start_time_ep) > self.t_flight_max:
                self.error_str = "Episode Completed: Time Exceeded [Truncated]"
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,RESET)

            ## REAL-TIME TIMEOUT
            elif (time.time() - self.start_time_real) > self.t_real_max:
                self.error_str = "Episode Completed: Episode Time Exceeded [Truncated] "
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,f"{(time.time() - self.start_time_real):.3f} s",RESET)

            else:
                terminated = False
                truncated = False
            
            # 5) RETURN VALUES
            return(
                next_obs,
                reward,
                terminated,
                truncated,
                {},
            )

        ########## POLICY POST-TRIGGER ##########
        elif a_Trg >= self.Pol_Trg_Threshold:

            # GRAB TERMINAL OBS/ACTION
            self.obs_trg = self._getObs()
            self.action_trg = action

            ## *** 2D Env ***
            self.Trg_Flag = True
            self.Tau_trg = self.Tau
            self.Tau_CR_trg = self.Tau_CR
            self.Theta_x_trg = self.Theta_x
            self.D_perp_trg = self.D_perp
            self.D_perp_CR_trg = self.D_perp_CR

            self.a_Trg_trg = a_Trg
            self.a_Rot_trg = a_Rot

            r_B_O,Phi_B_O,V_B_O,dPhi = self._getState()
            
            self.Vel_mag_B_O_trg = np.linalg.norm(V_B_O)
            self.Vel_angle_B_O_trg = np.degrees(np.arctan2(V_B_O[0],V_B_O[2]))

            V_tx,_,V_perp = self.R_WP(V_B_O,self.Plane_Angle_rad)
            self.Vel_mag_B_P_trg = np.linalg.norm([V_tx,V_perp])
            self.Vel_angle_B_P_trg = np.degrees(np.arctan2(V_tx,V_perp))

            self.State_trg = self._getState()
            ## ***

            # 2) FINISH EPISODE
            self.start_time_trg = self._getTime()
            terminated,truncated = self._finishSim(a_Rot)
            self.Done = terminated 

            # 3) CALC REWARD
            try:
                reward = self._CalcReward()  
            except (UnboundLocalError,ValueError):
                reward = 0.0


            # 5) RETURN VALUES
            return(
                self.obs_trg,
                reward,
                terminated,
                truncated,
                {},
            )
        

        return obs, reward, terminated, truncated, {}

    
    def _finishSim(self,a_Rot):

        OnceFlag_Trg = False
        OnceFlag_Impact = False

        terminated = False
        truncated = False

        while not (terminated or truncated):

            t_now = self._getTime()

            self._iterStep(a_Rot=a_Rot)
            self.render()
            
            ## START TRIGGER AND IMPACT TERMINATION TIMERS
            if (self.Trg_Flag == True and OnceFlag_Trg == False):
                self.start_time_trg = t_now     # Starts countdown for when to reset run
                OnceFlag_Trg = True             # Turns on to make sure this only runs once per rollout

            if (self.Impact_Flag_Ext and OnceFlag_Impact == False):
                self.start_time_impact = t_now
                OnceFlag_Impact = True


            # 4) CHECK TERMINATION/TRUNCATED

            # ============================
            ##    Termination Criteria 
            # ============================

            if self.Done == True:
                self.error_str = "Episode Completed: Done [Terminated] "
                terminated = True
                truncated = False
                # print(YELLOW,self.error_str,RESET)

            ## TRIGGER TIMEOUT  
            elif (t_now - self.start_time_trg) > self.t_trg_max:
                self.error_str = "Episode Completed: Pitch Timeout [Truncated] "
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,f"{(t_now - self.start_time_trg):.3f} s",RESET)

            ## IMPACT TIMEOUT
            elif (t_now - self.start_time_impact) > self.t_impact_max:
                self.error_str = "Episode Completed: Impact Timeout [Truncated] "
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,f"{(t_now - self.start_time_impact):.3f} s",RESET)

            elif self._getState()[2][2] <= -8: # Vz < -8 m/s
                self.error_str = "Episode Completed: Falling [Terminated]"
                terminated = True
                truncated = False
                # print(YELLOW,self.error_str,RESET)

            ## REAL-TIME TIMEOUT
            elif (time.time() - self.start_time_real) > self.t_real_max:
                self.error_str = "Episode Completed: Episode Time Exceeded [Truncated] "
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,f"{(time.time() - self.start_time_real):.3f} s",RESET)

            else:
                terminated = False
                truncated = False

        return terminated,truncated



    def _CalcReward(self):

        if self.Impact_Flag_Ext:
            V_B_O_impact = self.R_PW(self.V_B_P_impact_Ext,self.Plane_Angle_rad)    # {X_W,Y_W,Z_W}
            V_hat_impact = V_B_O_impact/np.linalg.norm(V_B_O_impact)                # {X_W,Y_W,Z_W}

            temp_angle_rad = np.arctan2(V_hat_impact[0],V_hat_impact[2])
            temp_angle_deg = np.degrees(temp_angle_rad)

            if temp_angle_deg < 0:
                Phi_B_P_Impact_Condition = -1
            elif temp_angle_deg > 0:
                Phi_B_P_Impact_Condition = 1
            elif math.isnan(temp_angle_deg):
                Phi_B_P_Impact_Condition = 1

        else:
            ## CALC REWARD VALUES
            R_LT = 0
            R_GM = 0
            R_Phi = 0


        if self.ForelegContact_Flag:

            ## CALC IMPACT ANGLE CONDITIONS
            rot_dir = np.sign(self.Rot_Sum_impact_Ext)
            num_rev = self.Rot_Sum_impact_Ext // (rot_dir*360+1e-6)  # Integer division to get full revolutions
            Phi_B_O_impact_deg = self.Eul_B_O_impact_Ext[1] + rot_dir*(num_rev*360)

            Phi_B_P_impact_deg = Phi_B_O_impact_deg - self.Plane_Angle_deg
            Phi_P_B_impact_deg = -Phi_B_P_impact_deg

            Beta1_deg = -Phi_P_B_impact_deg - self.Gamma_eff + 90
            Beta1_rad = np.radians(Beta1_deg)

            ## CALC LEG DIRECTION VECTOR
            r_B_C1 = np.array([-self.L_eff,0,0]) # {e_r1,0,e_beta1}
            r_B_C1 = self.R_PW(self.R_C1P(r_B_C1,Beta1_rad),self.Plane_Angle_rad)   # {X_W,Y_W,Z_W}

            r_C1_B = -r_B_C1                        # {X_W,Y_W,Z_W}
            e_r_hat = r_C1_B/np.linalg.norm(r_C1_B) # {X_W,Y_W,Z_W}

            ## MOMENTUM TRANSFER REWARD
            CP_LT = np.cross(V_hat_impact,e_r_hat) # {X_W,Y_W,Z_W}
            DP_LT = np.dot(V_hat_impact,e_r_hat)
            CP_LT_angle_deg = np.degrees(np.arctan2(CP_LT,DP_LT))[1]
            R_LT = self.Reward_LT(CP_LT_angle_deg,self.ForelegContact_Flag,self.HindlegContact_Flag)

            
            ## GRAVITY MOMENT REWARD
            g_hat = np.array([0,0,-1])              # {X_W,Y_W,Z_W}
            CP_GM = np.cross(g_hat,e_r_hat)         # {X_W,Y_W,Z_W}
            DP_GM = np.dot(g_hat,e_r_hat)
            CP_GM_angle_deg = np.degrees(np.arctan2(CP_GM,DP_GM))[1]
            R_GM = self.Reward_GravityMoment(CP_GM_angle_deg,self.ForelegContact_Flag,self.HindlegContact_Flag)

            ## PHI IMPACT REWARD
            R_Phi = self.Reward_ImpactAngle(Phi_P_B_impact_deg,self.Phi_P_B_impact_Min_deg,Phi_B_P_Impact_Condition)

        elif self.HindlegContact_Flag:

            ## CALC IMPACT ANGLE CONDITIONS
            rot_dir = np.sign(self.Rot_Sum_impact_Ext)
            num_rev = self.Rot_Sum_impact_Ext // (rot_dir*360+1e-6)  # Integer division to get full revolutions
            Phi_B_O_impact_deg = self.Eul_B_O_impact_Ext[1] + rot_dir*(num_rev*360)

            Phi_B_P_impact_deg = Phi_B_O_impact_deg - self.Plane_Angle_deg
            Phi_P_B_impact_deg = -Phi_B_P_impact_deg

            Beta2_deg = self.Gamma_eff + -Phi_P_B_impact_deg  + 90
            Beta2_rad = np.radians(Beta2_deg)

            ## CALC LEG DIRECTION VECTOR
            r_B_C2 = np.array([-self.L_eff,0,0]) # {e_r2,0,e_beta2}
            r_B_C2 = self.R_PW(self.R_C2P(r_B_C2,Beta2_rad),self.Plane_Angle_rad)   # {X_W,Y_W,Z_W}

            r_C2_B = -r_B_C2                        # {X_W,Y_W,Z_W}
            e_r_hat = r_C2_B/np.linalg.norm(r_C2_B) # {X_W,Y_W,Z_W}

            ## MOMENTUM TRANSFER REWARD
            CP_LT = np.cross(V_hat_impact,e_r_hat) # {X_W,Y_W,Z_W}
            DP_LT = np.dot(V_hat_impact,e_r_hat)
            CP_LT_angle_deg = np.degrees(np.arctan2(CP_LT,DP_LT))[1]
            R_LT = self.Reward_LT(CP_LT_angle_deg,self.ForelegContact_Flag,self.HindlegContact_Flag)

            
            ## GRAVITY MOMENT REWARD
            g_hat = np.array([0,0,-1])              # {X_W,Y_W,Z_W}
            CP_GM = np.cross(g_hat,e_r_hat)         # {X_W,Y_W,Z_W}
            DP_GM = np.dot(g_hat,e_r_hat)
            CP_GM_angle_deg = np.degrees(np.arctan2(CP_GM,DP_GM))[1]
            R_GM = self.Reward_GravityMoment(CP_GM_angle_deg,self.ForelegContact_Flag,self.HindlegContact_Flag)

            ## PHI IMPACT REWARD
            R_Phi = self.Reward_ImpactAngle(Phi_P_B_impact_deg,self.Phi_P_B_impact_Min_deg,Phi_B_P_Impact_Condition)

        elif self.BodyContact_Flag:

            ## CALC IMPACT ANGLE CONDITIONS
            rot_dir = np.sign(self.Rot_Sum_impact_Ext)
            num_rev = self.Rot_Sum_impact_Ext // (rot_dir*360+1e-6)  # Integer division to get full revolutions
            Phi_B_O_impact_deg = self.Eul_B_O_impact_Ext[1] + rot_dir*(num_rev*360)

            Phi_B_P_impact_deg = Phi_B_O_impact_deg - self.Plane_Angle_deg
            Phi_P_B_impact_deg = -Phi_B_P_impact_deg

            ## CALC REWARD VALUES
            R_LT = 0
            R_GM = 0
            R_Phi = self.Reward_ImpactAngle(Phi_P_B_impact_deg,self.Phi_P_B_impact_Min_deg,Phi_B_P_Impact_Condition)


        ## REWARD: MINIMUM DISTANCE AFTER TRIGGER
        if self.Tau_CR_trg < np.inf:
            R_dist = self.Reward_Exp_Decay(self.D_perp_CR_min,0)
        else:
            R_dist = 0

        ## REWARD: TAU_CR TRIGGER
        R_tau_cr = self.Reward_Exp_Decay(self.Tau_CR_trg,0.15)

        ## REWARD: PAD CONNECTIONS
        if self.Pad_Connections >= 3: 
            R_Legs = 1.0
        elif self.Pad_Connections == 2:
            R_Legs = 0.2
        else:
            R_Legs = 0.0

        if self.BodyContact_Flag:
            R_Legs = R_Legs*0.5

        self.reward_vals = [R_dist,R_tau_cr,R_LT,R_GM,R_Phi,R_Legs]
        R_t = np.dot(self.reward_vals,list(self.reward_weights.values()))
        self.reward = R_t/self.W_max
        # print(f"R_t_norm: {R_t/self.W_max:.3f}")
        # print(np.round(self.reward_vals,2))

        return R_t/self.W_max
    
    def Reward_Exp_Decay(self,x,threshold,k=10):
        if x < threshold:
            return 1
        elif threshold <= x:
            return np.exp(-k*(x-threshold))
        else:
            return 0
    
        
    def Reward_ImpactAngle(self,Phi_deg,Phi_min,Impact_condition):

        if Impact_condition == -1:
            Phi_deg = -Phi_deg

        ## NOTE: THESE ARE ALL RELATIVE ANGLES
        Phi_TD = 180
        Phi_w = Phi_TD - Phi_min
        Phi_b = Phi_w/2

        if Phi_deg <= -2*Phi_min:
            return 0.0
        elif -2*Phi_min < Phi_deg <= Phi_min:
            return 0.5/(3*Phi_min - 0) * (Phi_deg - Phi_min) + 0.50
        elif Phi_min < Phi_deg <= Phi_min + Phi_b:
            return 0.5/((Phi_min + Phi_b) - Phi_min) * (Phi_deg - Phi_min) + 0.5
        elif Phi_min + Phi_b < Phi_deg <= Phi_TD:
            return -0.25/(Phi_TD - (Phi_min + Phi_b)) * (Phi_deg - Phi_TD) + 0.75
        elif Phi_TD < Phi_deg <= Phi_TD + Phi_b:
            return 0.25/((Phi_TD + Phi_b) - Phi_TD) * (Phi_deg - Phi_TD) + 0.75
        elif (Phi_TD + Phi_b) < Phi_deg <= (Phi_TD + Phi_w):
            return -0.5/((Phi_TD + Phi_w) - (Phi_TD + Phi_b)) * (Phi_deg - (Phi_TD + Phi_w)) + 0.5
        elif (Phi_TD + Phi_w) < Phi_deg <= (360 + 2*Phi_min):
            return -0.5/(3*Phi_min) * (Phi_deg - ((Phi_TD + Phi_w))) + 0.5
        elif (360 + 2*Phi_min) <= Phi_deg:
            return 0.0

    def Reward_LT(self,CP_angle_deg,ForelegContact_Flag,HindlegContact_Flag):

        if HindlegContact_Flag == True:
            CP_angle_deg = -CP_angle_deg  # Reflect function across the y-axis
        elif ForelegContact_Flag == True:
            CP_angle_deg = CP_angle_deg
        else:
            return 0

        ## CALCULATE REWARD
        if -180 <= CP_angle_deg <= 0:
            return -np.sin(np.radians(CP_angle_deg)) * 1/2 + 0.5
        elif 0 < CP_angle_deg <= 180:
            return -1.0/180 * CP_angle_deg * 1/2 + 0.5
        
    def Reward_GravityMoment(self,CP_angle_deg,ForelegContact_Flag,HindlegContact_Flag):

        if HindlegContact_Flag == True:
            CP_angle_deg = -CP_angle_deg  # Reflect function across the y-axis
        elif ForelegContact_Flag == True:
            CP_angle_deg = CP_angle_deg
        else:
            return 0

        ## CALCULATE REWARD
        return -np.sin(np.radians(CP_angle_deg)) * 1/2 + 0.5








if __name__ == '__main__':
    env = SAR_2D_Env(Ang_Acc_range=[-100,100],V_mag_range=[1.5,3.5],V_angle_range=[5,175],Plane_Angle_range=[0,180],Render=True)

    for ep in range(50):

        V_mag = 2.5
        V_angle = 120
        Plane_Angle = 0

        if V_mag != None:
            env.V_mag_range = [V_mag,V_mag]

        if V_angle != None:
            env.V_angle_range = [V_angle,V_angle]

        if Plane_Angle != None:
            env.Plane_Angle_range = [Plane_Angle,Plane_Angle]

        obs,_ = env.reset()

        Done = False
        while not Done:

            action = env.action_space.sample() # obs gets passed in here
            action[0] = 0
            action[1] = 1.0
            if env.Tau_CR <= 0.255:
                action[0] = 1
            obs,reward,terminated,truncated,_ = env.step(action)
            Done = terminated or truncated

        print(f"Episode: {ep} \t Reward: {reward:.3f} \t Reward_vec: ",end='')
        print(' '.join(f"{val:.2f}" for val in env.reward_vals))




                