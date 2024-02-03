import gymnasium as gym
import numpy as np
from gymnasium import spaces

import time
import pygame as pg
import os
import rospy

EPS = 1e-6 # Epsilon (Prevent division by zero)
COORD_FLIP = -1  # Swap sign to match proper coordinate notation

EPS = 1e-6 # Epsilon (Prevent division by zero)
YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = '\033[34m'  
RESET = '\033[0m'  # Reset to default color

from sar_env import SAR_2D_Base_Interface

class SAR_2D_Env(SAR_2D_Base_Interface,gym.Env):

    def __init__(self,Ang_Acc_range=[-100,100],V_mag_range=[1.5,3.5],V_angle_range=[5,175],Plane_Angle_range=[0,180],Render=True):
        SAR_2D_Base_Interface.__init__(self,Render)
        gym.Env.__init__(self)
        
        ######################
        #    GENERAL CONFIGS
        ######################
        
        ## ENV CONFIG SETTINGS
        self.Env_Name = "SAR_Env_2D"

        ## TESTING CONDITIONS     
        self.V_mag_range = V_mag_range  
        self.V_angle_range = V_angle_range
        self.Plane_Angle_range = Plane_Angle_range
        self.setAngAcc_range(Ang_Acc_range)


        ## TIME CONSTRAINTS
        self.t_rot_max = np.sqrt(np.radians(360)/np.max(np.abs(self.Ang_Acc_range))) # Allow enough time for a full rotation [s]
        self.t_impact_max = 1.0     # [s]
        self.t_ep_max = 5.0         # [s]
        self.t_real_max = 120      # [s]


        ## INITIAL LEARNING/REWARD CONFIGS
        self.K_ep = 0
        self.Pol_Trg_Threshold = 0.5
        self.Done = False
        self.reward = 0
        self.reward_vals = np.array([0,0,0,0,0,0])
        self.reward_weights = {
            "W_Dist":0.5,
            "W_tau_cr":0.5,
            "W_LT":1.0,
            "W_GM":1.0,
            "W_Phi_rel":2.0,
            "W_Legs":2.0
        }
        self.W_max = sum(self.reward_weights.values())

        self.D_perp_min = np.inf
        self.Tau_trg = np.inf
        self.Tau_CR_trg = np.inf

        ## DOMAIN RANDOMIZATION
        self.Base_Iyy_std = 0.1
        self.Base_Mass_std = 0.1

        ## DEFINE OBSERVATION SPACE
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.obs_trg = np.zeros(self.observation_space.shape,dtype=np.float32) # Obs values at triggering

        ## DEFINE ACTION SPACE
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.action_trg = np.zeros(self.action_space.shape,dtype=np.float32) # Action values at triggering

        

    def reset(self, seed=None, options=None, V_mag=None,V_angle=None,Plane_Angle=None):

        ######################
        #    GENERAL CONFIGS
        ######################

        ## RESET LEARNING/REWARD CONDITIONS
        self.K_ep += 1
        self.Done = False
        self.reward = 0

        self.D_perp_min = np.inf
        self.Tau_trg = np.inf
        self.Tau_CR_trg = np.inf

        self.start_time_real = time.time()

        self.resetPose()

        ##########   2D ENV CONFIGS  ##########
        #
        self.Trg_Flag = False
        self.action_trg = np.full(self.action_trg.shape[0],np.nan)
        self.obs_trg = np.full(self.obs_trg.shape[0],np.nan)
        self.state_trg = np.full(6,np.nan)
        self.impact_state = np.full(6,np.nan)

        self.Impact_Flag_Ext = False
        self.BodyContact_Flag = False
        self.Pad_Connections = 0
        self.MomentCutoff = False

        #
        #######################################

        ## SET PLANE POSE
        if Plane_Angle == None:

            Plane_Angle_Low = self.Plane_Angle_range[0]
            Plane_Angle_High = self.Plane_Angle_range[1]
            Plane_Angle = np.random.uniform(Plane_Angle_Low,Plane_Angle_High)
            self._setPlanePose(self.Plane_Pos,Plane_Angle)
            self._iterStep(n_steps=2)

        else:
            self._setPlanePose(self.Plane_Pos,Plane_Angle)
            self._iterStep(n_steps=2)

        ## SAMPLE VELOCITY AND FLIGHT ANGLE
        if V_mag == None or V_angle == None:
            V_mag,V_angle = self._sampleFlightConditions(self.V_mag_range,self.V_angle_range)

        else:
            V_mag = V_mag       # Flight velocity
            V_angle = V_angle   # Flight angle  

        self.V_mag = V_mag
        self.V_angle = V_angle

        ## CALC STARTING VELOCITY IN GLOBAL COORDS
        V_tx = V_mag*np.cos(np.deg2rad(V_angle))
        V_perp = V_mag*np.sin(np.deg2rad(V_angle))
        V_B_P = np.array([V_tx,0,V_perp])               # {t_x,n_p}
        V_B_O = self.R_PW(V_B_P,self.Plane_Angle_rad)   # {X_W,Z_W}

        ## CALCULATE STARTING TAU VALUE
        self.Tau_CR_start = self.t_rot_max*np.random.uniform(1.9,2.1) # Add noise to starting condition
        self.Tau_Body_start = (self.Tau_CR_start + self.Collision_Radius/V_perp) # Tau read by body
        self.Tau_Accel_start = 1.0 # Acceleration time to desired velocity conditions [s]

        ## CALC STARTING POSITION IN GLOBAL COORDS
        # (Derivation: Research_Notes_Book_3.pdf (9/17/23))

        r_P_O = np.array(self.Plane_Pos)                                        # Plane Position wrt to Origin - {X_W,Z_W}
        r_P_B = np.array([(self.Tau_CR_start + self.Tau_Accel_start)*V_tx,
                          0,
                          (self.Tau_Body_start + self.Tau_Accel_start)*V_perp])  # Body Position wrt to Plane - {t_x,n_p}
        r_B_O = r_P_O - self.R_PW(r_P_B,self.Plane_Angle_rad)                   # Body Position wrt to Origin - {X_W,Z_W}

        ## LAUNCH QUAD W/ DESIRED VELOCITY
        self.initial_state = (r_B_O,V_B_O)
        self.GZ_VelTraj(pos=r_B_O,vel=V_B_O)
        self._iterStep(n_steps=1000)

        # ## DOMAIN RANDOMIZATION (UPDATE INERTIA VALUES)
        # self.Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Iyy") + np.random.normal(0,1.5e-6)
        # self.Mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Mass") + np.random.normal(0,0.0005)
        # self.setModelInertia()

        ## ROUND OUT STEPS TO BE IN SYNC WITH CONTROLLER
        if self._getTick()%10 != 0:
            n_steps = 10 - (self._getTick()%10)
            self._iterStep(n_steps=n_steps)

        ## RESET/UPDATE TIME CONDITIONS
        self.start_time_ep = self._getTime()
        self.start_time_trg = np.nan
        self.start_time_impact = np.nan
        self.t_flight_max = self.Tau_Body_start*2.0   # [s]
        self.t_trg_max = self.Tau_Body_start*1.5 # [s]

        ######################
        #   2D ENV CONFIGS
        ######################

        # UPDATE RENDER
        if self.RENDER:
            self.render()
        
        return self._get_obs(), {}


    def step(self, action):

        # 1. TAKE ACTION
        # 2. UPDATE STATE
        # 3. CALC REWARD
        # 4. CHECK TERMINATION
        # 5. RETURN VALUES

        ## ROUND OUT STEPS TO BE IN SYNC WITH CONTROLLER
        if self._getTick()%10 != 0:
            n_steps = 10 - (self._getTick()%10)
            self._iterStep(n_steps=n_steps)


        a_Trg = action[0]
        a_Rot = 0.5 * (action[1] + 1) * (self.Ang_Acc_range[1] - self.Ang_Acc_range[0]) + self.Ang_Acc_range[0]
        
        if self._get_obs()[0] <= 0.20:
            a_Trg = 1

        ########## POLICY PRE-TRIGGER ##########
        if a_Trg <= self.Pol_Trg_Threshold:

            ## 2) UPDATE STATE
            self._iterStep(n_steps=10)
            t_now = self._getTime()

            # UPDATE RENDER
            if self.RENDER:
                self.render()

            # GRAB NEXT OBS
            next_obs = self._get_obs()

            # CHECK FOR IMPACT
            x,z,phi_B_O,Vx,Vz,dphi_B_O = self._get_state()
            self.Impact_Flag_Ext,Impact_Conditions = self._get_impact_conditions(x,z,phi_B_O)

            # UPDATE MINIMUM DISTANCE
            D_perp = next_obs[2]
            if D_perp <= self.D_perp_min:
                self.D_perp_min = D_perp 


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
                print(YELLOW,self.error_str,RESET)
            
            ## IMPACT TERMINATION
            elif self.Impact_Flag_Ext == True:
                self.error_str = "Episode Completed: Impact [Terminated]"
                terminated = True
                truncated = False
                print(YELLOW,self.error_str,RESET)

            ## EPISODE TIMEOUT
            elif (t_now - self.start_time_ep) > self.t_flight_max:
                self.error_str = "Episode Completed: Time Exceeded [Truncated]"
                terminated = False
                truncated = True
                print(YELLOW,self.error_str,RESET)

            ## REAL-TIME TIMEOUT
            elif (time.time() - self.start_time_real) > self.t_real_max:
                self.error_str = "Episode Completed: Episode Time Exceeded [Truncated] "
                terminated = False
                truncated = True
                print(YELLOW,self.error_str,f"{(time.time() - self.start_time_real):.3f} s",RESET)

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
            self.obs_trg = self._get_obs()
            self.action_trg = action

            ## *** 2D Env ***
            self.Trg_Flag = True
            self.Tau_trg = self.obs_trg[0]
            self.Tau_CR_trg = self.obs_trg[0]
            self.state_trg = self._get_state()
            ## ***

            # 2) FINISH EPISODE
            self.start_time_trg = self._getTime()
            terminated,truncated = self._finishSim(a_Rot)
            self.Done = terminated 

            # 3) CALC REWARD
            reward = self._CalcReward()  


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

        ## SEND TRIGGER ACTION TO CONTROLLER
        # self.sendCmd("Policy",[0,a_Rot,0],cmd_flag=1)

        ## RUN REMAINING STEPS AT FULL SPEED
        # self.pausePhysics(False)

        while not (terminated or truncated):

            t_now = self._getTime()
            self.Step(a_Rot)
            
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
                print(YELLOW,self.error_str,RESET)

            ## TRIGGER TIMEOUT  
            elif (t_now - self.start_time_trg) > self.t_trg_max:
                self.error_str = "Episode Completed: Pitch Timeout [Truncated] "
                terminated = False
                truncated = True
                print(YELLOW,self.error_str,f"{(t_now - self.start_time_trg):.3f} s",RESET)

            ## IMPACT TIMEOUT
            elif (t_now - self.start_time_impact) > self.t_impact_max:
                self.error_str = "Episode Completed: Impact Timeout [Truncated] "
                terminated = False
                truncated = True
                print(YELLOW,self.error_str,f"{(t_now - self.start_time_impact):.3f} s",RESET)

            ## REAL-TIME TIMEOUT
            elif (time.time() - self.start_time_real) > self.t_real_max:
                self.error_str = "Episode Completed: Episode Time Exceeded [Truncated] "
                terminated = False
                truncated = True
                print(YELLOW,self.error_str,f"{(time.time() - self.start_time_real):.3f} s",RESET)

            else:
                terminated = False
                truncated = False

        return terminated,truncated

    def Step(self,a_Rot):


        ## CHECK FOR IMPACT
        x,z,Phi_B_O,vx,vz,dPhi_B_O = self._get_state()
        Gamma_eff_rad,L_eff,Forward_Reach,M,Iyy,I_c = self.params
        self.Impact_Flag_Ext,self.Impact_Conditions = self._get_impact_conditions(x,z,Phi_B_O)

        ## NO IMPACT
        if self.Impact_Flag_Ext == False:

            ## UPDATE ROTATION FLIGHT STEP
            self._iter_step_Rot(a_Rot)

            # UPDATE MINIMUM DISTANCE
            D_perp = self._get_obs()[2]
            if D_perp <= self.D_perp_min:
                self.D_perp_min = D_perp 

            # UPDATE RENDER
            if self.RENDER:
                self.render()

        if self.Impact_Flag_Ext == True:


            ## GRAB IMPACT STATE
            (BodyContact_Flag,ForelegContact_Flag,HindlegContact_Flag) = self.Impact_Conditions
            self.impact_state = self._get_state()

            ## BODY CONTACT
            if BodyContact_Flag == True:
                self.BodyContact_Flag = True
                self.Pad_Connections = 0
                self.Done = True

            ## LEG 1 CONTACT
            elif ForelegContact_Flag == True:

                Beta_1,dBeta_1 = self._impact_conversion(self.state,self.params,self.Impact_Conditions)

                ## FIND IMPACT COORDS (wrt World-Coords)
                r_C1_O = self._get_pose()[1]

                while not self.Done:

                    ## ITERATE THROUGH SWING
                    Beta_1,dBeta_1 = self._iter_step_Swing(Beta_1,dBeta_1,impact_leg=1)

                    ## CHECK FOR END CONDITIONS
                    if Beta_1 <= -self._beta_landing(impact_leg=1):
                        self.BodyContact_Flag = False
                        self.Pad_Connections = 4
                        self.Done = True

                    elif Beta_1 >= -self._beta_prop(impact_leg=1):
                        self.BodyContact_Flag = True
                        self.Pad_Connections = 2
                        self.Done = True

                    elif self.t - start_time_impact >= self.t_impact_max:
                        self.BodyContact_Flag = False
                        self.Pad_Connections = 2
                        self.Done = True

                    ## CONVERT BODY BACK TO WORLD COORDS
                    r_B_C1 = np.array([-L_eff,0])                                           # {e_r1,e_beta1}
                    r_B_C1 = self.R_PW(self.R_C1P(r_B_C1,Beta_1),self.Plane_Angle_rad)  # {X_W,Z_W}
                    r_B_O = r_C1_O + r_B_C1                                             # {X_W,Z_W}

                    v_B_C1 = np.array([0,L_eff*dBeta_1])                                    # {e_r1,e_beta1}
                    v_B_C1 = self.R_PW(self.R_C1P(v_B_C1,Beta_1),self.Plane_Angle_rad)  # {X_W,Z_W}

                    Phi_B_O = np.arctan2(-np.cos(Beta_1 + Gamma_eff_rad + self.Plane_Angle_rad), \
                                        np.sin(Beta_1 + Gamma_eff_rad + self.Plane_Angle_rad))
                    self.state = (r_B_O[0],r_B_O[1],Phi_B_O,v_B_C1[0],v_B_C1[1],0)
                    

                    if self.RENDER:
                        self.render()


            ## LEG 2 CONTACT
            elif HindlegContact_Flag == True:

                Beta_2,dBeta_2 = self._impact_conversion(self.state,self.params,self.Impact_Conditions)

                ## FIND IMPACT COORDS (wrt World-Coords)
                r_C2_O = self._get_pose()[2]

                while not self.Done:

                    ## ITERATE THROUGH SWING
                    Beta_2,dBeta_2 = self._iter_step_Swing(Beta_2,dBeta_2,impact_leg=2)

                    ## CHECK FOR END CONDITIONS
                    if Beta_2 >= -self._beta_landing(impact_leg=2):
                        self.BodyContact_Flag = False
                        self.Pad_Connections = 4
                        self.Done = True

                    elif Beta_2 <= -self._beta_prop(impact_leg=2):
                        self.BodyContact_Flag = True
                        self.Pad_Connections = 2
                        self.Done = True

                    elif self.t - start_time_impact >= self.t_impact_max:
                        self.BodyContact_Flag = False
                        self.Pad_Connections = 2
                        self.Done = True

                    ## CONVERT BODY BACK TO WORLD COORDS
                    r_B_C2 = np.array([-L_eff,0])                                           # {e_r1,e_beta1}
                    r_B_C2 = self.R_PW(self.R_C2P(r_B_C2,Beta_2),self.Plane_Angle_rad)  # {X_W,Z_W}
                    r_B_O = r_C2_O + r_B_C2                                             # {X_W,Z_W}

                    v_B_C2 = np.array([0,L_eff*dBeta_2])                                    # {e_r1,e_beta1}
                    v_B_C2 = self.R_PW(self.R_C2P(v_B_C2,Beta_2),self.Plane_Angle_rad)  # {X_W,Z_W}

                    Phi_B_O = np.arctan2(-np.cos(Beta_2 - Gamma_eff_rad + self.Plane_Angle_rad), \
                                        np.sin(Beta_2 - Gamma_eff_rad + self.Plane_Angle_rad))
                    self.state = (r_B_O[0],r_B_O[1],Phi_B_O,v_B_C2[0],v_B_C2[1],0)

                    ## UPDATE MINIMUM DISTANCE
                    D_perp = self._get_obs()[2]
                    if not self.Done:
                        if D_perp <= self.D_perp_min:
                            self.D_perp_min = D_perp 

                    if self.RENDER:
                        self.render()




    def _CalcReward(self):

        ## LOAD PARAMS
        gamma_rad,L,PD,M,Iyy,I_c = self.params

        ## GAMMA
        gamma_rad = gamma_rad
        gamma_deg = np.degrees(gamma_rad)

        ## PLANE ANGLE
        Plane_Angle_deg = self.Plane_Angle_deg
        Plane_Angle_rad = np.radians(Plane_Angle_deg)


        ## SOLVE FOR MINIMUM BETA ANGLE VIA GEOMETRIC CONSTRAINTS
        a = np.sqrt(PD**2 + L**2 - 2*PD*L*np.cos(np.pi/2-gamma_rad))
        Beta_min_rad = np.arccos((L**2 + a**2 - PD**2)/(2*a*L))

        ## CALC BETA BOUNDARY ANGLE
        Beta_min_rad = Beta_min_rad * COORD_FLIP 
        Beta_min_deg = np.degrees(Beta_min_rad)

        ## CALC PHI BOUNDARY ANGLE
        Phi_Impact_min_deg = Beta_min_deg + gamma_deg + Plane_Angle_deg - 90


        ## CALC RELATIVE BOUNDARY ANGLE
        Phi_P_B_Impact_min_deg = Plane_Angle_deg - Phi_Impact_min_deg


        ## CALC IMPACT VELOCITY VECTOR
        _,_,Phi_impact_rad,Vx_impact,Vz_impact,dPhi_impact = self.impact_state
        V_B_O_impact = np.array([Vx_impact,Vz_impact])
        V_hat_impact = V_B_O_impact/np.linalg.norm(V_B_O_impact)

        Vel_Angle_impact_rad = np.arctan2(V_hat_impact[1],V_hat_impact[0])*COORD_FLIP
        Vel_Angle_impact_deg = np.degrees(Vel_Angle_impact_rad)



        ## FLIGHT VELOCITY ANGLE RELATIVE TO WORLD ("ORIGINAL BODY ORIENTATION")
        V_Angle_Body_0 = Vel_Angle_impact_deg + Plane_Angle_deg # {X_B,Z_B}
        if V_Angle_Body_0 < -90:
            Phi_B_P_Impact_Condition = -1
        elif -90 <= V_Angle_Body_0:
            Phi_B_P_Impact_Condition = 1


        (Body_Contact,ForeLeg_Contact,HindLeg_Contact) = self.Impact_Conditions
        if ForeLeg_Contact:

            ## CALC IMPACT ANGLE CONDITIONS
            Phi_impact_deg = np.degrees(Phi_impact_rad)
            Phi_B_P_deg = (Phi_impact_deg - Plane_Angle_deg)
            Phi_P_B_deg = -Phi_B_P_deg # Phi_rel of plane w.r.t. body

            Beta1_deg = Phi_impact_deg - gamma_deg - Plane_Angle_deg + 90
            Beta1_rad = np.radians(Beta1_deg)

            ## CALC LEG DIRECTION VECTOR
            r_B_C1 = np.array([-L,0])                                           # {e_r1,e_beta1}
            r_B_C1 = self.R_PW(self.R_C1P(r_B_C1,Beta1_rad),Plane_Angle_rad)    # {X_W,Z_W}

            r_C1_B = -r_B_C1                       # {X_W,Z_W}
            e_r1 = r_C1_B/np.linalg.norm(r_C1_B)   # {X_W,Z_W}
            e_r_hat = np.array([e_r1[0],e_r1[1]])

            ## MOMENTUM TRANSFER REWARD
            CP_LT = -np.cross(V_hat_impact,e_r_hat) # Swap sign for consitent notation
            DP_LT = np.dot(V_hat_impact,e_r_hat)
            CP_LT_angle_deg = np.degrees(np.arctan2(CP_LT,DP_LT))
            R_LT = self.Reward_LT(CP_LT_angle_deg,Leg_Num=1)

            ## GRAVITY MOMENT REWARD
            g_hat = np.array([0,-1]) # {X_W,Z_W}
            CP_GM = -np.cross(g_hat,e_r_hat) # Swap sign for consitent notation
            DP_GM = np.dot(g_hat,e_r_hat)
            CP_GM_angle_deg = np.degrees(np.arctan2(CP_GM,DP_GM))
            R_GM = self.Reward_GravityMoment(CP_GM_angle_deg,Leg_Num=1)

            ## PHI IMPACT REWARD
            R_Phi = self.Reward_ImpactAngle(Phi_P_B_deg,Phi_P_B_Impact_min_deg,Phi_B_P_Impact_Condition)

            
        elif HindLeg_Contact:
            
            ## CALC IMPACT ANGLE CONDITIONS
            Phi_impact_deg = np.degrees(Phi_impact_rad)
            Phi_B_P_deg = (Phi_impact_deg - Plane_Angle_deg)
            Phi_P_B_deg = -Phi_B_P_deg # Phi_rel of plane w.r.t. body

            Beta2_deg = gamma_deg + Phi_impact_deg - Plane_Angle_deg + 90
            Beta2_rad = np.radians(Beta2_deg)

            ## CALC LEG DIRECTION VECTOR
            r_B_C2 = np.array([-L,0])                                           # {e_r1,e_beta1}
            r_B_C2 = self.R_PW(self.R_C2P(r_B_C2,Beta2_rad),Plane_Angle_rad)    # {X_W,Z_W}
            r_B_O = r_B_C2

            r_C2_B = -r_B_C2                       # {X_W,Z_W}
            e_r2 = r_C2_B/np.linalg.norm(r_C2_B)   # {X_W,Z_W}
            e_r_hat = np.array([e_r2[0],e_r2[1]])

            ## MOMENTUM TRANSFER REWARD
            CP_LT = -np.cross(V_hat_impact,e_r_hat) # Swap sign for consitent notation
            DP_LT = np.dot(V_hat_impact,e_r_hat)
            CP_LT_angle_deg = np.degrees(np.arctan2(CP_LT,DP_LT))
            R_LT = self.Reward_LT(CP_LT_angle_deg,Leg_Num=2)

            ## GRAVITY MOMENT REWARD
            g_hat = np.array([0,-1]) # {X_W,Z_W}
            CP_GM = -np.cross(g_hat,e_r_hat) # Swap sign for consitent notation
            DP_GM = np.dot(g_hat,e_r_hat)
            CP_GM_angle_deg = np.degrees(np.arctan2(CP_GM,DP_GM))
            R_GM = self.Reward_GravityMoment(CP_GM_angle_deg,Leg_Num=2)

            ## PHI IMPACT REWARD
            R_Phi = self.Reward_ImpactAngle(Phi_P_B_deg,Phi_P_B_Impact_min_deg,Phi_B_P_Impact_Condition)

        elif Body_Contact:

            ## CALC IMPACT ANGLE CONDITIONS
            Phi_impact_deg = np.degrees(Phi_impact_rad)
            Phi_B_P_deg = (Phi_impact_deg - Plane_Angle_deg)
            Phi_P_B_deg = -Phi_B_P_deg # Phi_rel of plane w.r.t. body

            ## CALC REWARD VALUES
            R_LT = 0
            R_GM = 0
            R_Phi = self.Reward_ImpactAngle(Phi_P_B_deg,Phi_P_B_Impact_min_deg,Phi_B_P_Impact_Condition)

        else:

            ## CALC REWARD VALUES
            R_LT = 0
            R_GM = 0
            R_Phi = 0

        
        ## REWARD: MINIMUM DISTANCE 
        if self.Tau_CR_trg < np.inf:

            R_dist = self.Reward_Exp_Decay(self.D_perp_min,L)
        else:
            R_dist = 0

        ## REWARD: TAU_CR TRIGGER
        # R_tau_cr = self.Reward_Exp_Decay(self.Tau_CR_trg,5)
        R_tau_cr = 0

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
        print(f"R_t_norm: {R_t/self.W_max:.3f} ")
        print(np.round(self.reward_vals,2))

        return R_t/self.W_max
    
    def Reward_Exp_Decay(self,x,threshold,k=5):
        if -0.1 < x < threshold:
            return 1
        elif threshold <= x:
            return np.exp(-k*(x-threshold))
        else:
            return 0
    
    def Reward_RotationDirection(self,x,rot_dir):

        if rot_dir == +1:
            return x if x < 0 else 1
        elif rot_dir == -1:
            return 1 if x < 0 else -x
        
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

    def Reward_LT(self,CP_angle_deg,Leg_Num):

        if Leg_Num == 2:
            CP_angle_deg = -CP_angle_deg  # Reflect across the y-axis

        if -180 <= CP_angle_deg <= 0:
            return -np.sin(np.radians(CP_angle_deg)) * 1/2 + 0.5
        elif 0 < CP_angle_deg <= 180:
            return -1.0/180 * CP_angle_deg * 1/2 + 0.5
        
    def Reward_GravityMoment(self,CP_angle_deg,Leg_Num):

        if Leg_Num == 2:
            CP_angle_deg = -CP_angle_deg  # Reflect across the y-axis

        return -np.sin(np.radians(CP_angle_deg)) * 1/2 + 0.5

    def _sample_flight_conditions(self):

        ## SAMPLE VEL FROM UNIFORM DISTRIBUTION IN VELOCITY RANGE
        Vel_Low = self.V_mag_range[0]
        Vel_High = self.V_mag_range[1]
        V_mag = np.random.uniform(low=Vel_Low,high=Vel_High)

        ## SAMPLE RELATIVE PHI FROM A WEIGHTED SET OF UNIFORM DISTRIBUTIONS
        Rel_Angle_Low = self.V_angle_range[0]
        Rel_Angle_High = self.V_angle_range[1]
        Flight_Angle_range = Rel_Angle_High-Rel_Angle_Low

        Dist_Num = np.random.choice([0,1,2],p=[0.1,0.8,0.1]) # Probability of sampling distribution

        if Dist_Num == 0: # Low Range
            Flight_Angle = np.random.default_rng().uniform(low=Rel_Angle_Low, high=Rel_Angle_Low + 0.1*Flight_Angle_range)
        elif Dist_Num == 1: # Medium Range
            Flight_Angle = np.random.default_rng().uniform(low=Rel_Angle_Low + 0.1*Flight_Angle_range, high=Rel_Angle_High - 0.1*Flight_Angle_range)
        elif Dist_Num == 2: # High Range
            Flight_Angle = np.random.default_rng().uniform(low=Rel_Angle_High - 0.1*Flight_Angle_range, high=Rel_Angle_High)
       
        return V_mag,Flight_Angle
    
    
    def _get_state(self):

        return self.state
      
    def _get_obs(self):
        
        x,z,phi,vx,vz,dphi = self._get_state()
        
        ## POSITION AND VELOCITY VECTORS
        r_B_O = np.array([x,z])
        V_B_O = np.array([vx,vz])

        ## PLANE POSITION AND UNIT VECTORS
        r_P_O = self.Plane_Pos
        Plane_Angle_rad = self.Plane_Angle_rad

        ## CALC DISPLACEMENT FROM PLANE CENTER
        r_P_B = r_P_O - r_B_O # {X_W,Z_W}

        ## CALC RELATIVE DISTANCE AND VEL
        _,D_perp = self.R_WP(r_P_B,Plane_Angle_rad)
        V_tx,V_perp = self.R_WP(V_B_O,Plane_Angle_rad)

        ## CALC OPTICAL FLOW VALUES
        Tau = np.clip(D_perp/(V_perp + EPS),0,5)
        Theta_x = np.clip(V_tx/(D_perp + EPS),-20,20)


        r_CR_B = np.array([0,self.Collision_Radius])                        # {t_x,n_p}
        r_P_CR = r_P_O - (r_B_O + self.R_PW((r_CR_B),self.Plane_Angle_rad)) # {X_W,Z_W}
        _,D_perp_CR = self.R_WP(r_P_CR,self.Plane_Angle_rad)                 # Convert to plane coords - {t_x,n_p}
        self.Tau_CR = np.clip(D_perp_CR/(V_perp + EPS),-5,5)


        ## OBSERVATION VECTOR
        obs = np.array([Tau,Theta_x,D_perp,Plane_Angle_rad],dtype=np.float32)

        return obs
    
    

    def close(self):

        return
    




if __name__ == '__main__':
    env = SAR_2D_Env(Ang_Acc_range=[-100,100],V_mag_range=[1.5,3.5],V_angle_range=[5,175],Plane_Angle_range=[0,180],Render=True)
    env.RENDER = True

    for ep in range(20):

        V_mag = None
        V_angle = None
        Plane_Angle = None

        obs,_ = env.reset(V_mag=V_mag,V_angle=V_angle,Plane_Angle=Plane_Angle)


        Done = False
        while not Done:

            action = env.action_space.sample() # obs gets passed in here
            action[0] = 0
            action[1] = -0.5
            obs,reward,terminated,truncated,_ = env.step(action)
            Done = terminated or truncated

        print(f"Episode: {ep} \t Reward: {reward:.3f} \t Reward_vec: ",end='')
        print(' '.join(f"{val:.2f}" for val in env.reward_vals))




                