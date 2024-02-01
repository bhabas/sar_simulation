import gymnasium as gym
import numpy as np
from gymnasium import spaces

import pygame as pg
import os

## DEFINE COLORS
WHITE = (255,255,255)
GREY = (200,200,200)
BLACK = (0,0,0)
RED = (204,0,0)
BLUE = (29,123,243)
SKY_BLUE = (0,153,153)
GREEN = (0,153,0)
PURPLE = (76,0,153)
ORANGE = (255,128,0)

EPS = 1e-6 # Epsilon (Prevent division by zero)
COORD_FLIP = -1  # Swap sign to match proper coordinate notation



class SAR_Env_2D(gym.Env):

    def __init__(self,GZ_Timeout=True,My_range=[-8.0e-3,0],V_mag_range=[1.5,3.5],V_angle_range=[-175,-5],Plane_Angle_range=[0,180],Render=True):
        super().__init__()
        
        ######################
        #    GENERAL CONFIGS
        ######################
        
        ## ENV CONFIG SETTINGS
        self.Env_Name = "SAR_Env_2D"

        ## DEFINE OBSERVATION SPACE
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.obs_trg = np.zeros(self.observation_space.shape,dtype=np.float32) # Obs values at triggering

        ## DEFINE ACTION SPACE
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.action_trg = np.zeros(self.action_space.shape,dtype=np.float32) # Action values at triggering

        ## TESTING CONDITIONS     
        self.V_mag_range = V_mag_range  
        self.V_angle_range = V_angle_range
        self.Plane_Angle_range = Plane_Angle_range
        self.My_range = My_range

        ## SAR DIMENSIONAL CONSTRAINTS
        self.gamma_deg = 17             # Leg Angle [m]
        self.L = 216.4e-3           # Leg Length [m]
        self.PD = 150.0e-3             # Prop Distance from COM [m]
        self.M = 781.1e-3            # Body Mass [kg]

        self.Ixx = 5.75e-3          # Body Moment of Inertia [kg*m^2]
        self.Iyy = 5.29e-3          # Body Moment of Inertia [kg*m^2]
        self.Izz = 5.66e-3          # Body Moment of Inertia [kg*m^2]

        I_c = self.Iyy + self.M*self.L**2
        self.params = (np.deg2rad(self.gamma_deg),self.L,self.PD,self.M,self.Iyy,I_c)
        self.Collision_Radius = max(self.L,self.PD)

        ## SAR CAPABILITY CONSTRAINTS
        My_max = abs(max(My_range,key=abs))
        self.t_rot_max = np.sqrt((2*self.Iyy*np.radians(360))/(0.5*My_max + EPS)) ## Allow enough time for a full rotation
        self.t_rot_max = 0.3


        ## PLANE PARAMETERS
        self.Plane_Pos = [1,0]
        self.Plane_Angle_deg = 0
        self.Plane_Angle_rad = np.radians(self.Plane_Angle_deg)

        ## LEARNING/REWARD CONFIGS
        self.Pol_Trg_threshold = 0.5

        ## TIME CONSTRAINTS
        self.t_trg_max = 1.0   # [s]
        self.t_impact_max = 1.0   # [s]

        ## INITIAL LEARNING/REWARD CONFIGS
        self.K_ep = 0
        self.Pol_Trg_Flag = False
        self.Done = False
        self.reward = 0
        self.reward_vals = np.array([0,0,0,0,0,0])
        self.reward_weights = {
            "W_Dist":0.2,
            "W_tau_cr":0.0,
            "W_LT":1.0,
            "W_GM":0.0,
            "W_Phi_rel":0.0,
            "W_Legs":2.0
        }
        self.W_max = sum(self.reward_weights.values())

        self.D_min = np.inf
        self.Tau_trg = np.inf
        self.Tau_CR_trg = np.inf



        ######################
        #   2D ENV CONFIGS
        ######################

        ## SPECIAL CONFIGS
        self.state = np.zeros(6)
        self.impact_state = np.full(6,np.nan)

        self.V_mag = np.nan
        self.V_angle = np.nan
        self.MomentCutoff = False
        self.BodyContact_flag = False
        self.Pad_Connections = 0


        ## PHYSICS PARAMETERS
        self.g = 9.81       # Gravity [m/s^2]
        self.dt = 1e-3      # [s]
        self.t = 0          # [s]

        ## RENDERING PARAMETERS
        self.world_width = 3.0      # [m]
        self.world_height = 2.0     # [m]
        self.x_offset = 1           # [m]
        self.y_offset = 1           # [m]
        self.screen_width = 1000    # [pixels]
        self.screen_height = self.screen_width*self.world_height/self.world_width # [pixels]
        
        self.RENDER = Render
        self.screen = None
        self.clock = None
        self.isopen = True

        ## START SIMULATION
        self.reset()

    def reset(self, seed=None, options=None, V_mag=None, V_angle=None, Plane_Angle=None):

        ######################
        #    GENERAL CONFIGS
        ######################

        ## RESET LEARNING/REWARD CONDITIONS
        self.K_ep += 1
        self.Done = False
        self.Pol_Trg_Flag = False
        self.reward = 0

        self.D_min = np.inf
        self.Tau_trg = np.inf
        self.Tau_CR_trg = np.inf
        self.action_trg = np.full(self.action_trg.shape[0],np.nan)
        self.obs_trg = np.full(self.obs_trg.shape[0],np.nan)
        self.state_trg = np.full(6,np.nan)
        self.impact_state = np.full(6,np.nan)

        self.Impact_Flag = False
        self.BodyContact_flag = False
        self.Pad_Connections = 0
        self.MomentCutoff = False

        ## RESET/UPDATE TIME CONDITIONS
        self.start_time_ep = self._getTime()
        self.start_time_trg = np.nan
        self.start_time_impact = np.nan

        ## RESET POSITION RELATIVE TO LANDING SURFACE (BASED ON STARTING TAU VALUE)
        # (Derivation: Research_Notes_Book_3.pdf (9/17/23))

        ## SET PLANE POSE
        if Plane_Angle == None:

            Plane_Angle_Low = self.Plane_Angle_range[0]
            Plane_Angle_High = self.Plane_Angle_range[1]
            self.Plane_Angle_deg = np.random.uniform(Plane_Angle_Low,Plane_Angle_High)
            self.Plane_Angle_rad = np.radians(self.Plane_Angle_deg)

        else:
            self.Plane_Angle_deg = Plane_Angle
            self.Plane_Angle_rad = np.radians(self.Plane_Angle_deg)

        ## SAMPLE VELOCITY AND FLIGHT ANGLE
        if V_mag == None or V_angle == None:
            V_mag,V_angle = self._sample_flight_conditions()

        else:
            V_mag = V_mag       # Flight velocity
            V_angle = V_angle   # Flight angle  

        self.V_mag = V_mag
        self.V_angle = V_angle


        ## RELATIVE VEL VECTORS
        V_tx = V_mag*np.cos(np.deg2rad(V_angle))
        V_perp = V_mag*np.sin(np.deg2rad(V_angle))
        V_B_P = np.array([V_tx,V_perp]) # {t_x,n_p}

        ## CONVERT RELATIVE VEL VECTORS TO WORLD COORDS
        V_B_O = self.R_PW(V_B_P,self.Plane_Angle_rad) # {X_W,Z_W}

        ## CALCULATE STARTING TAU VALUE
        self.Tau_CR_start = self.t_rot_max*2
        self.Tau_Body_start = (self.Tau_CR_start + self.Collision_Radius/V_perp) # Tau read by body


        ## CALC STARTING POSITION IN GLOBAL COORDS
        r_P_O = np.array(self.Plane_Pos)                                                    # Plane Position wrt to Origin - {X_W,Z_W}
        r_P_B = np.array([self.Tau_CR_start*V_tx, (self.Tau_Body_start)*V_perp])  # Body Position wrt to Plane - {t_x,n_p}
        r_B_O = r_P_O - self.R_PW(r_P_B,self.Plane_Angle_rad)                               # Body Position wrt to Origin - {X_W,Z_W}

        ## LAUNCH QUAD W/ DESIRED VELOCITY
        self._set_state(r_B_O[0],r_B_O[1],np.radians(-1),V_B_O[0],V_B_O[1],0)
        self.initial_state = (r_B_O,V_B_O)


        ## CALC TAU OF COLLISION RADIUS
        r_CR_B = np.array([0,self.Collision_Radius]) # {t_x,n_p}
        r_CR_O = r_B_O + self.R_PW(r_CR_B,self.Plane_Angle_rad)
        r_P_CR = r_P_O - r_CR_O # {X_W,Z_W}
        _,D_perp_CR = self.R_WP(r_P_CR,self.Plane_Angle_rad) # Convert to plane coords
        self.Tau_CR = D_perp_CR/V_perp

        self.t_flight_max = self.Tau_Body_start*2   # [s]

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

        ########## PRE-POLICY TRIGGER ##########
        if action[0] <= self.Pol_Trg_threshold:

            ## 2) UPDATE STATE
            self._iterStep(n_steps=10)

            # UPDATE RENDER
            if self.RENDER:
                self.render()
            

            # GRAB NEXT OBS
            next_obs = self._get_obs()

            # CHECK FOR IMPACT
            x,z,phi,vx,vz,dphi = self._get_state()
            impact_flag,impact_conditions = self._get_impact_conditions(x,z,phi)

            # UPDATE MINIMUM DISTANCE
            D_perp = next_obs[2]
            if D_perp <= self.D_min:
                self.D_min = D_perp 


            # 3) CALCULATE REWARD
            reward = 0.0


            # 4) CHECK TERMINATION
            self.Done = bool(
                self.Done
                or (impact_flag or self.BodyContact_flag)
            )
            terminated = self.Done
            truncated = bool(self._getTime() - self.start_time_ep >= self.t_flight_max) 

            if truncated:
                print("Truncated")
            
            # 5) RETURN VALUES
            return(
                next_obs,
                reward,
                terminated,
                truncated,
                {},
            )


        ########## POST-POLICY TRIGGER ##########
        elif action[0] >= self.Pol_Trg_threshold:

            # GRAB TERMINAL OBS/ACTION
            self.obs_trg = self._get_obs()
            self.action_trg = action

            self.Tau_trg = self.obs_trg[0]
            self.Tau_CR_trg = self.Tau_CR
            self.start_time_trg = self._getTime()

            ## TRIGGER STATE
            self.state_trg = self._get_state()


            # 2) UPDATE STATE
            self.Pol_Trg_Flag = True
            self.Finish_Sim(action)         


            # 3) CALC REWARD
            reward = self.Calc_Reward()  


            # 4) CHECK TERMINATION
            terminated = self.Done = True
            truncated = False


            # 5) RETURN VALUES
            return(
                self.obs_trg,
                reward,
                terminated,
                truncated,
                {},
            )


        return obs, reward, terminated, truncated, {}
    
    def Finish_Sim(self,action):

        ## SCALE ACTION
        scaled_action = 0.5 * (action[1] + 1) * (self.My_range[1] - self.My_range[0]) + self.My_range[0]

        while not self.Done:
            
            ## CHECK FOR IMPACT
            x,z,Phi,vx,vz,dphi = self._get_state()
            gamma_rad,L,PD,M,Iyy,I_c = self.params
            self.Impact_Flag,self.impact_conditions = self._get_impact_conditions(x,z,Phi)

            ## NO IMPACT
            if self.Impact_Flag == False:

                ## UPDATE ROTATION FLIGHT STEP
                self._iter_step_Rot(scaled_action)

                # UPDATE MINIMUM DISTANCE
                D_perp = self._get_obs()[2]
                if D_perp <= self.D_min:
                    self.D_min = D_perp 

                # UPDATE RENDER
                if self.RENDER:
                    self.render()

            if self.Impact_Flag == True:

                ## START IMPACT TIMER
                start_time_impact = self.t
                self.start_time_impact = self._getTime()
                (BodyContact,Leg1Contact,Leg2Contact) = self.impact_conditions

                ## GRAB IMPACT STATE
                self.impact_state = self._get_state()

                ## BODY CONTACT
                if BodyContact == True:
                    self.BodyContact_flag = True
                    self.Pad_Connections = 0
                    self.Done = True

                ## LEG 1 CONTACT
                elif Leg1Contact == True:

                    Beta_1,dBeta_1 = self._impact_conversion(self.state,self.params,self.impact_conditions)

                    ## FIND IMPACT COORDS (wrt World-Coords)
                    r_C1_O = self._get_pose()[1]

                    while not self.Done:

                        ## ITERATE THROUGH SWING
                        Beta_1,dBeta_1 = self._iter_step_Swing(Beta_1,dBeta_1,impact_leg=1)

                        ## CHECK FOR END CONDITIONS
                        if Beta_1 <= -self._beta_landing(impact_leg=1):
                            self.BodyContact_flag = False
                            self.Pad_Connections = 4
                            self.Done = True

                        elif Beta_1 >= -self._beta_prop(impact_leg=1):
                            self.BodyContact_flag = True
                            self.Pad_Connections = 2
                            self.Done = True

                        elif self.t - start_time_impact >= self.t_impact_max:
                            self.BodyContact_flag = False
                            self.Pad_Connections = 2
                            self.Done = True

                        ## CONVERT BODY BACK TO WORLD COORDS
                        r_B_C1 = np.array([-L,0])                                           # {e_r1,e_beta1}
                        r_B_C1 = self.R_PW(self.R_C1P(r_B_C1,Beta_1),self.Plane_Angle_rad)  # {X_W,Z_W}
                        r_B_O = r_C1_O + r_B_C1                                             # {X_W,Z_W}

                        v_B_C1 = np.array([0,L*dBeta_1])                                    # {e_r1,e_beta1}
                        v_B_C1 = self.R_PW(self.R_C1P(v_B_C1,Beta_1),self.Plane_Angle_rad)  # {X_W,Z_W}

                        Phi = np.arctan2(-np.cos(Beta_1 + gamma_rad + self.Plane_Angle_rad), \
                                          np.sin(Beta_1 + gamma_rad + self.Plane_Angle_rad))
                        self.state = (r_B_O[0],r_B_O[1],Phi,v_B_C1[0],v_B_C1[1],0)
                        

                        if self.RENDER:
                            self.render()


                ## LEG 2 CONTACT
                elif Leg2Contact == True:

                    Beta_2,dBeta_2 = self._impact_conversion(self.state,self.params,self.impact_conditions)

                    ## FIND IMPACT COORDS (wrt World-Coords)
                    r_C2_O = self._get_pose()[2]

                    while not self.Done:

                        ## ITERATE THROUGH SWING
                        Beta_2,dBeta_2 = self._iter_step_Swing(Beta_2,dBeta_2,impact_leg=2)

                        ## CHECK FOR END CONDITIONS
                        if Beta_2 >= -self._beta_landing(impact_leg=2):
                            self.BodyContact_flag = False
                            self.Pad_Connections = 4
                            self.Done = True

                        elif Beta_2 <= -self._beta_prop(impact_leg=2):
                            self.BodyContact_flag = True
                            self.Pad_Connections = 2
                            self.Done = True

                        elif self.t - start_time_impact >= self.t_impact_max:
                            self.BodyContact_flag = False
                            self.Pad_Connections = 2
                            self.Done = True

                        ## CONVERT BODY BACK TO WORLD COORDS
                        r_B_C2 = np.array([-L,0])                                           # {e_r1,e_beta1}
                        r_B_C2 = self.R_PW(self.R_C2P(r_B_C2,Beta_2),self.Plane_Angle_rad)  # {X_W,Z_W}
                        r_B_O = r_C2_O + r_B_C2                                             # {X_W,Z_W}

                        v_B_C2 = np.array([0,L*dBeta_2])                                    # {e_r1,e_beta1}
                        v_B_C2 = self.R_PW(self.R_C2P(v_B_C2,Beta_2),self.Plane_Angle_rad)  # {X_W,Z_W}

                        Phi = np.arctan2(-np.cos(Beta_2 - gamma_rad + self.Plane_Angle_rad), \
                                          np.sin(Beta_2 - gamma_rad + self.Plane_Angle_rad))
                        self.state = (r_B_O[0],r_B_O[1],Phi,v_B_C2[0],v_B_C2[1],0)

                        ## UPDATE MINIMUM DISTANCE
                        D_perp = self._get_obs()[2]
                        if not self.Done:
                            if D_perp <= self.D_min:
                                self.D_min = D_perp 

                        if self.RENDER:
                            self.render()



            # 4) CHECK TERMINATION
            self.Done = bool(
                self.Done
                or (z <= -0.7 and vz <= -1) # IF SAR IS FALLING
            )

            truncated = bool(self.t - self.start_time_trg > self.t_trg_max) 
            
        return self.Done,truncated

    def Calc_Reward(self):

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


        (Body_Contact,ForeLeg_Contact,HindLeg_Contact) = self.impact_conditions
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

            R_dist = self.Reward_Exp_Decay(self.D_min,L)
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

        if self.BodyContact_flag:
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
    
    def _iterStep(self,n_steps=10):

        for _ in range(n_steps):

            ## UPDATE STATE
            x,z,phi,vx,vz,dphi = self._get_state()

            self.t += self.dt

            z_acc = 0.0
            z = z + self.dt*vz
            vz = vz + self.dt*z_acc

            x_acc = 0.0
            x = x + self.dt*vx
            vx = vx + self.dt*x_acc

            phi_acc = 0.0
            phi = phi + self.dt*dphi
            dphi = dphi + self.dt*phi_acc

            self.state = np.array([x,z,phi,vx,vz,dphi])

    def _iter_step_Rot(self,Rot_action,n_steps=2):

        ## PARAMS
        gamma_rad,L,PD,M,Iyy,I_c = self.params

        ## CURRENT STATE
        x,z,phi,vx,vz,dphi = self._get_state()

        ## TURN OFF BODY MOMENT IF ROTATED PAST 90 DEG
        if np.abs(phi) < np.deg2rad(90) and self.MomentCutoff == False:
            My = Rot_action

        else: 
            self.MomentCutoff= True
            My = 0

        for _ in range(n_steps):

            ## STEP UPDATE
            self.t += self.dt

            z_acc = -self.g
            z = z + self.dt*vz
            vz = vz + self.dt*z_acc

            x_acc = 0
            x = x + self.dt*vx
            vx = vx + self.dt*x_acc

            phi_acc = My/Iyy
            phi = phi + self.dt*dphi
            dphi = dphi + self.dt*phi_acc

            self.state = np.array([x,z,phi,vx,vz,dphi])

    def _iter_step_Swing(self,Beta,dBeta,impact_leg,n_steps=2):

        gamma_rad,L,PD,M,Iyy,I_c = self.params

        for _ in range(n_steps):

            if impact_leg==1:

                ## GRAVITY MOMENT
                M_g = -M*self.g*L*np.cos(Beta)*np.cos(self.Plane_Angle_rad) \
                        + M*self.g*L*np.sin(Beta)*np.sin(self.Plane_Angle_rad)
                
            if impact_leg==2:

                ## GRAVITY MOMENT
                M_g = -M*self.g*L*np.cos(Beta)*np.cos(self.Plane_Angle_rad) \
                        + M*self.g*L*np.sin(Beta)*np.sin(self.Plane_Angle_rad)
                
                
            ## ITER STEP BETA
            self.t += self.dt
            Beta_acc = M_g/I_c
            Beta = Beta + self.dt*dBeta
            dBeta = dBeta + self.dt*Beta_acc

        return Beta,dBeta

    def _impact_conversion(self,impact_state,params,impact_conditions):

        x,z,phi,vx,vz,dphi = impact_state
        gamma_rad,L,PD,M,Iyy,I_c = self.params
        (BodyContact,Leg1Contact,Leg2Contact) = impact_conditions

        V_tx,V_perp = self.R_WP(np.array([vx,vz]),self.Plane_Angle_rad)

        if Leg1Contact:

            ## CALC BETA ANGLE
            Beta_1 = np.arctan2(np.cos(gamma_rad - phi + self.Plane_Angle_rad), \
                                np.sin(gamma_rad - phi + self.Plane_Angle_rad))
            Beta_1_deg = np.degrees(Beta_1)

            ## CALC DBETA FROM MOMENTUM CONVERSION
            H_V_perp = M*L*V_perp*np.cos(Beta_1)
            H_V_tx = M*L*V_tx*np.sin(Beta_1)
            H_dphi = Iyy*dphi
            dBeta_1 = 1/(I_c)*(H_V_perp + H_V_tx + H_dphi)

            return Beta_1,dBeta_1

        elif Leg2Contact:

            ## CALC BETA ANGLE
            Beta_2 = np.arctan2( np.cos(gamma_rad + phi - self.Plane_Angle_rad), \
                                -np.sin(gamma_rad + phi - self.Plane_Angle_rad))
            Beta2_deg = np.degrees(Beta_2)


            ## CALC DBETA FROM MOMENTUM CONVERSION
            H_V_perp = M*L*V_perp*np.cos(Beta_2)
            H_V_tx = M*L*V_tx*np.sin(Beta_2)
            H_dphi = Iyy*dphi
            
            dBeta_2 = 1/(I_c)*(H_V_perp + H_V_tx + H_dphi)

            return Beta_2,dBeta_2

    def _beta_landing(self,impact_leg):

        gamma_rad,L,PD,M,Iyy,I_c = self.params

        if impact_leg == 1:
            return np.pi/2 + gamma_rad

        elif impact_leg == 2:
            return np.pi/2 - gamma_rad
        
    def _beta_prop(self,impact_leg):

        gamma_rad,L,PD,M,Iyy,I_c = self.params

        a = np.sqrt(PD**2 + L**2 - 2*PD*L*np.cos(np.pi/2-gamma_rad))
        Beta = np.arccos((L**2 + a**2 - PD**2)/(2*a*L))

        if impact_leg == 1:

            return Beta
        
        elif impact_leg == 2:

            return np.pi-Beta

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
    
    def _getTime(self):

        return self.t
    
    def _get_pose(self):

        gamma_rad,L,PD,M,Iyy,I_c = self.params
        x,z,phi,_,_,_ = self._get_state()

        ## MODEL CoG
        CG = np.array([x,z])

        ## LEG COORDS
        L1 = np.array([ L*np.sin(gamma_rad),-L*np.cos(gamma_rad)])
        L2 = np.array([-L*np.sin(gamma_rad),-L*np.cos(gamma_rad)])

        ## PROP COORDS
        Prop1 = np.array([ PD,0])
        Prop2 = np.array([-PD,0])

        ## CONVERT BODY COORDS TO WORLD COORDS
        L1 = CG + self.R_BW(L1,phi)
        L2 = CG + self.R_BW(L2,phi)
        Prop1 = CG + self.R_BW(Prop1,phi)
        Prop2 = CG + self.R_BW(Prop2,phi)

        return np.array([CG,L1,L2,Prop1,Prop2])

    def _get_impact_conditions(self,x,z,phi):

        impact_flag  = False
        Body_contact = False
        Leg1_contact = False
        Leg2_contact = False

        CG_Pos,Leg1_Pos,Leg2_Pos,Prop1_Pos,Prop2_Pos = self._get_pose()

        ## CHECK FOR CG CONTACT 
        CG_wrt_Plane = self.R_WP((CG_Pos - self.Plane_Pos),self.Plane_Angle_rad)
        if CG_wrt_Plane[1] >= 0:
            impact_flag = True
            Body_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR PROP CONTACT
        for Prop_Pos in [Prop1_Pos,Prop2_Pos]:

            Prop_wrt_Plane = self.R_WP((Prop_Pos - self.Plane_Pos),self.Plane_Angle_rad)
            if Prop_wrt_Plane[1] >= 0:
                impact_flag = True
                Body_contact = True

                return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG1 CONTACT
        Leg1_wrt_Plane = self.R_WP((Leg1_Pos - self.Plane_Pos),self.Plane_Angle_rad)
        if Leg1_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg1_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG2 CONTACT
        Leg2_wrt_Plane = self.R_WP((Leg2_Pos - self.Plane_Pos),self.Plane_Angle_rad)
        if Leg2_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg2_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]


        return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

    def _set_state(self,x,z,phi,vx,vz,dphi):

        self.state = (x,z,phi,vx,vz,dphi)

    def render(self):

        ## SET DEFAULT WINDOW POSITION
        Win_Loc_z = 500
        Win_Loc_y = 700
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (Win_Loc_z,Win_Loc_y)

        ## CONVERT COORDINATES TO PIXEL LOCATION
        def c2p(Pos):

            if len(Pos) == 2:
                x = Pos[0]
                y = Pos[1]

            elif len(Pos) == 3:
                x = Pos[0]
                y = Pos[2]

            
            scale_x = self.screen_width/self.world_width
            scale_y = self.screen_height/self.world_height
            
            x_p = (self.x_offset+x)*scale_x # [pixels]
            y_p = (self.y_offset+y)*scale_y # [pixels]
            
            return (x_p,y_p)
        
        ## INITIATE SCREEN AND CLOCK ON FIRST LOADING
        if self.screen is None:
            pg.init()
            self.screen = pg.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pg.time.Clock()


        ## GET CURRENT STATE
        x,z,phi,vx,vz,dphi = self._get_state()

        ## GET CURRENT PARAMS
        gamma_rad,L,PD,M,Iyy,I_c = self.params
        
        ## CREATE BACKGROUND SURFACE
        self.surf = pg.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE)

        ## TRAJECTORY LINE
        r_B_O,V_B_O = self.initial_state
        self.draw_line_dashed(self.surf,GREY,c2p(r_B_O - 5*V_B_O),c2p(r_B_O + 5*V_B_O),width=3)

        ## ORIGIN AXES
        pg.draw.line(self.surf,GREEN,c2p((0,0,0)),c2p((0.1,0)),width=5) # X_w   
        pg.draw.line(self.surf,BLUE, c2p((0,0,0)),c2p((0,0.1)),width=5) # Z_w   
        pg.draw.circle(self.surf,RED,c2p((0,0,0)),radius=4,width=0)


        ## LANDING SURFACE
        pg.draw.line(self.surf,GREY,
                         c2p(self.Plane_Pos + self.R_PW(np.array([-2,0]),self.Plane_Angle_rad)),
                         c2p(self.Plane_Pos + self.R_PW(np.array([+2,0]),self.Plane_Angle_rad)),width=2)
        
        pg.draw.line(self.surf,BLACK,
                         c2p(self.Plane_Pos + self.R_PW(np.array([-0.5,0]),self.Plane_Angle_rad)),
                         c2p(self.Plane_Pos + self.R_PW(np.array([+0.5,0]),self.Plane_Angle_rad)),width=5)
    
        ## LANDING SURFACE AXES
        pg.draw.line(self.surf,GREEN,c2p(self.Plane_Pos),c2p(self.Plane_Pos + self.R_PW(np.array([0.1,0]),self.Plane_Angle_rad)),width=7)  # t_x   
        pg.draw.line(self.surf,BLUE, c2p(self.Plane_Pos),c2p(self.Plane_Pos + self.R_PW(np.array([0,0.1]),self.Plane_Angle_rad)),width=7)  # n_p 
        pg.draw.circle(self.surf,RED,c2p(self.Plane_Pos),radius=4,width=0)


        ## DRAW QUADROTOR
        Pose = self._get_pose()
        pg.draw.line(self.surf,RED,c2p(Pose[0]),c2p(Pose[1]),width=3) # Leg 1
        pg.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[2]),width=3) # Leg 2
        pg.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[3]),width=3) # Prop 1
        pg.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[4]),width=3) # Prop 2
        pg.draw.circle(self.surf,GREY,c2p(Pose[0]),radius=self.Collision_Radius*self.screen_width/self.world_width,width=2)

        ## BODY AXES
        pg.draw.line(self.surf,GREEN,c2p(Pose[0]),c2p(Pose[0] + self.R_BW(np.array([0.05,0]),phi)),width=5)  # B_x   
        pg.draw.line(self.surf,BLUE,c2p(Pose[0]),c2p(Pose[0] + self.R_BW(np.array([0,0.05]),phi)),width=5)  # B_z  

        ## GRAVITY UNIT VECTOR
        g_hat = np.array([0,-1])
        pg.draw.line(self.surf,PURPLE,c2p(Pose[0]),c2p(Pose[0]) + g_hat*25,width=3)

        ## VELOCITY UNIT VECTOR
        v = np.array([vx,vz])
        v_hat = v/np.linalg.norm(v)
        pg.draw.line(self.surf,ORANGE,c2p(Pose[0]),c2p(Pose[0]) + v_hat*25,width=3)




        ## TRIGGER INDICATOR
        if self.Pol_Trg_Flag == True:
            pg.draw.circle(self.surf,RED,  c2p(Pose[0]),radius=4,width=0)
            pg.draw.circle(self.surf,BLACK,c2p(Pose[0]),radius=5,width=3)
        else:
            pg.draw.circle(self.surf,BLUE, c2p(Pose[0]),radius=4,width=0)
            pg.draw.circle(self.surf,BLACK,c2p(Pose[0]),radius=5,width=3)



        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pg.transform.flip(self.surf, False, True)

        ## WINDOW TEXT
        my_font = pg.font.SysFont(None, 30)

        ## STATES TEXT
        text_States = my_font.render(f'States:', True, GREY)
        text_t_step = my_font.render(f'Time Step: {self.t:7.03f} [s]', True, BLACK)
        text_V_mag = my_font.render(f'V_mag: {self.V_mag:.2f} [m/s]', True, BLACK)
        text_Rel_Angle = my_font.render(f'Flight_Angle: {self.V_angle:.2f} [deg]', True, BLACK)

        ## OBSERVATIONS TEXT
        text_Obs = my_font.render(f'Observations:', True, GREY)
        text_Tau = my_font.render(f'Tau: {self._get_obs()[0]:2.2f} [s]', True, BLACK)
        text_theta_x = my_font.render(f'Theta_x: {self._get_obs()[1]:2.2f} [rad/s]', True, BLACK)
        text_D_perp = my_font.render(f'D_perp: {self._get_obs()[2]:2.2f} [m]', True, BLACK)
        text_Plane_Angle = my_font.render(f'Plane Angle: {self.Plane_Angle_deg:3.1f} [deg]', True, BLACK)

        ## ACTIONS TEXT
        text_Actions = my_font.render(f'Actions:', True, GREY)
        text_Trg_Action = my_font.render(f'Trg_Action: {self.action_trg[0]:3.1f}', True, BLACK)
        text_Rot_Action = my_font.render(f'Rot_Action: {self.action_trg[1]:3.1f}', True, BLACK)

        ## OTHER TEXT
        text_Other = my_font.render(f'Other:', True, GREY)
        text_reward = my_font.render(f'Prev Reward: {self.reward:.3f}',True, BLACK)
        text_Tau_trg = my_font.render(f'Tau_trg: {self.Tau_trg:.3f} [s]',True, BLACK)
        text_Tau_CR_trg = my_font.render(f'Tau_CR_trg: {self.Tau_CR_trg:.3f} [s]',True, BLACK)
        text_Tau_CR = my_font.render(f'Tau CR: {self.Tau_CR:.3f} [s]',True, BLACK)
        text_Phi = my_font.render(f'Phi: {np.degrees(phi):.0f} deg',True, BLACK)





        ## DRAW OBJECTS TO SCREEN
        self.screen.blit(self.surf,         (0,0))
        self.screen.blit(text_States,       (5,5))
        self.screen.blit(text_t_step,       (5,5 + 25*1))
        self.screen.blit(text_Rel_Angle,    (5,5 + 25*2))
        self.screen.blit(text_V_mag,        (5,5 + 25*3))

        self.screen.blit(text_Obs,          (5,5 + 25*5))
        self.screen.blit(text_Tau,          (5,5 + 25*6))
        self.screen.blit(text_theta_x,      (5,5 + 25*7))
        self.screen.blit(text_D_perp,       (5,5 + 25*8))
        self.screen.blit(text_Plane_Angle,  (5,5 + 25*9))

        self.screen.blit(text_Actions,      (5,5 + 25*11))
        self.screen.blit(text_Trg_Action,   (5,5 + 25*12))
        self.screen.blit(text_Rot_Action,   (5,5 + 25*13))

        self.screen.blit(text_Other,        (5,5 + 25*15))
        self.screen.blit(text_reward,       (5,5 + 25*16))
        self.screen.blit(text_Tau_trg,      (5,5 + 25*17))
        self.screen.blit(text_Tau_CR_trg,   (5,5 + 25*18))
        self.screen.blit(text_Tau_CR,       (5,5 + 25*19))
        self.screen.blit(text_Phi,          (5,5 + 25*20))






        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(30) # [Hz]
        pg.display.flip()

    def close(self):

        return
    
    def R_BW(self,vec,phi):

        R_BW = np.array([
            [ np.cos(phi), np.sin(phi)],
            [-np.sin(phi), np.cos(phi)],
        ])

        return R_BW.dot(vec)
    
    def R_WP(self,vec,theta):

        R_WP = np.array([
            [ np.cos(theta),-np.sin(theta)],
            [ np.sin(theta), np.cos(theta)]
        ])

        return R_WP.dot(vec)
    
    def R_PW(self,vec,theta):

        R_PW = np.array([
            [ np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])

        return R_PW.dot(vec)
    
    def R_PC1(self,vec,Beta1):

        R_PC1 = np.array([
            [ np.cos(Beta1),-np.sin(Beta1)],
            [ np.sin(Beta1), np.cos(Beta1)]
        ])

        return R_PC1.dot(vec)
    
    def R_C1P(self,vec,Beta1):

        R_C1P = np.array([
            [ np.cos(Beta1), np.sin(Beta1)],
            [-np.sin(Beta1), np.cos(Beta1)]
        ])

        return R_C1P.dot(vec)
    
    def R_C1B(self,vec,gamma_rad):

        R_C1B = np.array([
            [ np.sin(gamma_rad), np.cos(gamma_rad)],
            [-np.cos(gamma_rad), np.sin(gamma_rad)],
        ])

        return R_C1B.dot(vec)

    def R_PC2(self,vec,Beta2):

        R_PC2 = np.array([
            [ np.cos(Beta2), np.sin(Beta2)],
            [-np.sin(Beta2), np.cos(Beta2)]
        ])

        return R_PC2.dot(vec)
    
    def R_C2P(self,vec,Beta2):

        R_C2P = np.array([
            [ np.cos(Beta2), np.sin(Beta2)],
            [-np.sin(Beta2), np.cos(Beta2)],
        ])

        return R_C2P.dot(vec)

    def R_C2B(self,vec,gamma_rad):

        R_C2B = np.array([
            [-np.sin(gamma_rad), np.cos(gamma_rad)],
            [-np.cos(gamma_rad),-np.sin(gamma_rad)],
        ])

        return R_C2B.dot(vec)


    def draw_line_dashed(self,surface, color, start_pos, end_pos, width = 1, dash_length = 10, exclude_corners = True):

        # convert tuples to numpy arrays
        start_pos = np.array(start_pos)
        end_pos   = np.array(end_pos)

        # get euclidian distance between start_pos and end_pos
        length = np.linalg.norm(end_pos - start_pos)

        # get amount of pieces that line will be split up in (half of it are amount of dashes)
        dash_amount = int(length / dash_length)

        # x-y-value-pairs of where dashes start (and on next, will end)
        dash_knots = np.array([np.linspace(start_pos[i], end_pos[i], dash_amount) for i in range(2)]).transpose()

        return [pg.draw.line(surface, color, tuple(dash_knots[n]), tuple(dash_knots[n+1]), width)
                for n in range(int(exclude_corners), dash_amount - int(exclude_corners), 2)]





if __name__ == '__main__':
    env = SAR_Env_2D(My_range=[-8e-3,+8e-3],V_mag_range=[2,2],V_angle_range=[-100,-100],Plane_Angle_range=[0,0])
    env.RENDER = True

    for ep in range(50):

        obs,_ = env.reset(V_mag=3.0,V_angle=20,Plane_Angle=0)

        Done = False
        truncated = False
        while not (Done or truncated):

            action = env.action_space.sample()
            action = np.array([0.0,0])
            obs,reward,Done,truncated,_ = env.step(action)

        # print(f"Episode: {ep} \t Obs: {obs[2]:.3f} \t Reward: {reward:.3f}")

    env.close()



                