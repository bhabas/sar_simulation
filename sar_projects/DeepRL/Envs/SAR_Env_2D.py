import gymnasium as gym
import numpy as np
from gymnasium import spaces

import pygame
import os

## DEFINE COLORS
WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (204,0,0)
BLUE = (29,123,243)
GREEN = (0,153,0)
PURPLE = (76,0,153)
ORANGE = (255,128,0)

EPS = 1e-6 # Epsilon (Prevent division by zero)

class SAR_Env_2D(gym.Env):
    """Custom Environment that follows gym interface."""

    def __init__(self,GZ_Timeout=True,My_range=[-8.0,8.0],Vel_range=[1.5,3.5],Phi_rel_range=[0,90],Plane_Angle_range=[90,180],Tau_0=0.4):

        gym.Env.__init__(self)

        ## ENV CONFIG SETTINGS
        self.Env_Name = "SAR_Env_2D"

        ## DOMAIN RANDOMIZATION SETTINGS
        self.Mass_std = 0.5e-3  # [kg]
        self.Iyy_std = 1.5e-6   # [kg*m^2]

        ## TESTING CONDITIONS
        self.Tau_0 = Tau_0          
        self.Vel_range = Vel_range  
        self.Phi_rel_range = Phi_rel_range
        self.Plane_Angle_range = Plane_Angle_range
        self.My_range = My_range

        ## RESET INITIAL VALUES
        self.K_ep = 0
        self.Trg_threshold = 0.5
        self.D_min = np.inf
        self.Tau_trg = np.inf
        self.Done = False
        self.Pol_Trg_Flag = False

        ## DEFINE OBSERVATION SPACE
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.obs_trg = np.zeros(self.observation_space.shape,dtype=np.float32) # Obs values at triggering

        ## DEFINE ACTION SPACE
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.action_trg = np.zeros(self.action_space.shape,dtype=np.float32) # Action values at triggering

        ## PLANE PARAMETERS
        self.Plane_Pos = [1,0]
        self.Plane_Angle = 180

        ## SAR DIMENSIONS CONSTRAINTS 
        gamma = np.deg2rad(45)  # Leg Angle [m]
        L = 150.0e-3             # Leg Length [m]
        M_B = 35.0e-3           # Body Mass [kg]
        I_B = 17.0e-6           # Body Moment of Inertia [kg*m^2]
        PD = 32.5e-3            # Prop Distance from COM [m]
        self.params = (L,gamma,M_B,I_B,PD)

        ## PHYSICS PARAMETERS
        self.g = 9.81       # Gravity [m/s^2]
        self.dt = 0.005     # [s]
        self.t = 0          # [s]
        self.t_max = 0.8    # [s]
        self.state = (0,0.1,0,0,0,0) # Initial State (X_pos,Z_pos,phi,Vx,Vz,dphi)



        ## RENDERING PARAMETERS
        self.screen_width = 1000
        self.RENDER = False
        self.world_width = 4.0  # [m]
        self.world_height = 4.0 # [m]

        self.screen_height = self.screen_width*self.world_height/self.world_width
        self.screen = None
        self.clock = None
        self.isopen = True

        ## RESET LOGGING CONDITIONS 
        self.K_ep += 1

    def reset(self, seed=None, options=None, V_mag=None, Phi_rel=None):

        ## RESET RECORDED VALUES
        self.Done = False
        self.D_min = np.inf       # Reset max distance from landing surface [m]
        self.Tau_trg = np.inf     # Reset Tau triggering value [s]
        self.obs_trg = np.zeros_like(self.observation_space.high)
        self.action_trg = np.zeros_like(self.action_space.high)

        ## SET PLANE POSE
        Plane_Angle_Low = self.Plane_Angle_range[0]
        Plane_Angle_High = self.Plane_Angle_range[1]
        self.Plane_Angle = np.random.uniform(Plane_Angle_Low,Plane_Angle_High)
        self.Plane_Angle_rad = np.radians(self.Plane_Angle)

        ## RESET POSITION RELATIVE TO LANDING SURFACE (BASED ON STARTING TAU VALUE)
        # (Derivation: Research_Notes_Book_3.pdf (6/22/23))
        r_PO = np.array(self.Plane_Pos)                             # Plane Position w/r to origin
        n_hat,t_x = self._calc_PlaneNormal(self.Plane_Angle)    # Plane direction vectors

        ## SAMPLE VELOCITY AND FLIGHT ANGLE
        if V_mag == None or Phi_rel == None:
            V_mag,Phi_rel = self._sample_flight_conditions()

        else:
            V_mag = V_mag           # Flight velocity
            Phi_rel = Phi_rel   # Flight angle  

        ## RELATIVE VEL VECTORS
        V_perp = V_mag*np.sin(np.deg2rad(Phi_rel))
        V_tx = V_mag*np.cos(np.deg2rad(Phi_rel))
        V_BP = np.array([V_tx,V_perp])

        ## CONVERT RELATIVE VEL VECTORS TO WORLD COORDS
        V_BO = self._P_to_W(V_BP,self.Plane_Angle,deg=True)
        V_hat = V_BO/np.linalg.norm(V_BO)   # Flight Velocity unit vector


        ## INITIAL DISTANCE
        D_perp = (self.Tau_0*V_perp)

        ## INITIAL POSITION RELATIVE TO PLANE
        r_BP = -(D_perp/(V_hat.dot(n_hat)+EPS))*V_hat

        ## INITIAL POSITION IN GLOBAL COORDS
        r_BO = r_PO + r_BP 

        ## LAUNCH QUAD W/ DESIRED VELOCITY
        self._set_state(r_BO[0],r_BO[1],np.deg2rad(0),V_BO[0],V_BO[1],0)

        ## UPDATE RENDER
        if self.RENDER:
            self.render()

        ## RESET/UPDATE RUN CONDITIONS
        self.start_time_rollout = self.t
        self.start_time_pitch = np.nan
        self.start_time_impact = np.nan

        ## RESET 2D SIM FLAGS
        self.impact_flag = False
        self.BodyContact_flag = False
        self.MomentCutoff = False
        

        return self._get_obs(),{}

    def step(self, action):
        
        ########## PRE-POLICY TRIGGER ##########
        if action[0] <= self.Trg_threshold:

            ## GRAB CURRENT OBSERVATION
            obs = self._get_obs()

            ## CHECK FOR IMPACT
            x,z,phi,vx,vz,dphi = self._get_state()
            self.impact_flag,self.impact_conditions = self._get_impact_conditions(x,z,phi)

            ## CHECK FOR DONE
            self.Done = bool(
                self.Done
                or (self.impact_flag or self.BodyContact_flag)  # BODY CONTACT W/O POLICY TRIGGER
            )

            ## UPDATE MINIMUM DISTANCE
            D_perp = obs[2]
            if not self.Done:
                if D_perp <= self.D_min:
                    self.D_min = D_perp 

            ## CHECK FOR DONE
            terminated = self.Done
            truncated = bool(self.t - self.start_time_rollout > self.t_max) # EPISODE TIME-OUT 

            ## CALCULATE REWARD
            reward = 0

            ## UPDATE SIM AND OBSERVATION
            self._iter_step()
            next_obs = self._get_obs()

            ## UPDATE RENDER
            if self.RENDER:
                self.render()

            return (
                next_obs,
                reward,
                terminated,
                truncated,
                {},
            )


        ########## POST-POLICY TRIGGER ##########
        elif action[0] >= self.Trg_threshold:

            ## GRAB CURRENT OBSERVATION
            terminal_obs = self._get_obs()   # Return this observation because reward and future 
                                    # are defined by action taken here. Not obs at end of episode.

            ## SAVE TRIGGERING OBSERVATION AND ACTIONS
            self.obs_trg = terminal_obs
            self.Tau_trg = terminal_obs[0]
            self.action_trg = action       

            ## COMPLETE REST OF SIMULATION
            terminated,truncated = self._finish_sim(action)       

            ## CALCULATE REWARD
            reward = self._CalcReward()    

            return(
                terminal_obs,
                reward,
                terminated,
                truncated,
                {},
            )

    def _finish_sim(self,action):

        ## SCALE ACTION
        scaled_action = 0.5 * (action[1] + 1) * (self.My_range[1] - self.My_range[0]) + self.My_range[0]

        ## START PROJECTILE FLIGHT
        while not self.Done:

            ## CHECK FOR IMPACT
            x,z,phi,vx,vz,dphi = self._get_state()
            L,gamma,M_B,I_B,PD = self._get_params()
            self.impact_flag,self.impact_conditions = self._get_impact_conditions(x,z,phi)
            

            if self.impact_flag == False:

                ## UPDATE STEP
                self._iter_step_Rot(scaled_action)

                

            elif self.impact_flag == True:

                self.phi_impact = np.rad2deg(phi)
                (BodyContact,Leg1Contact,Leg2Contact) = self.impact_conditions

                ## BODY CONTACT
                if BodyContact == True:
                    self.BodyContact_flag = True
                    self.pad_connections = 0
                    self.Done = True

                ## LEG 1 CONTACT
                elif Leg1Contact == True:

                    beta,dbeta = self._impact_conversion(self.state,self.params,self.impact_conditions)

                    ## CALC INERTIA ABOUT CONTACT POINT
                    I_C = M_B*L**2 + I_B
                    
                    ## FIND IMPACT COORDS (wrt World-Coords)
                    r_C1_W = self._get_pose(x,z,phi)[1]


                    ## ITERATE THROUGH SWING
                    while not self.Done:

                        ## GRAVITY MOMENT
                        M_g = -M_B*self.g*L*np.sin(self.Plane_Angle_rad)*np.sin(beta) \
                                + M_B*self.g*L*np.cos(self.Plane_Angle_rad)*np.cos(beta)
                        
                        ## ITER STEP BETA
                        beta_acc = -M_g/I_C
                        beta = beta + self.dt*dbeta
                        dbeta = dbeta + self.dt*beta_acc

                        ## CONVERT BODY BACK TO WORLD COORDS
                        phi = np.deg2rad(90) - beta - self.Plane_Angle_rad + gamma

                        temp = self._Beta_to_P(np.array([L,0]),beta)
                        r_B_C1 = self._P_to_W(temp,self.Plane_Angle_rad)
                        r_B_W = r_C1_W + r_B_C1

                        self.state = (r_B_W[0],r_B_W[1],phi,0,0,0)

                        if self.RENDER:
                            self.render()

                        


                    self.Done = True

                    

                ## LEG 2 CONTACT
                elif Leg2Contact == True:

                    self.Done = True


            ## UPDATE MINIMUM DISTANCE
            D_perp = self._get_obs()[2]
            if not self.Done:
                if D_perp <= self.D_min:
                    self.D_min = D_perp 


            truncated = bool(self.t - self.start_time_rollout > self.t_max) # EPISODE TIME-OUT 

            ## CHECK FOR DONE
            self.Done = bool(
                self.Done
                or truncated
                # or (vz <= -5.0 and self._get_obs()[2] >= 0.5) # IF SAR IS FALLING
            )

            ## UPDATE RENDER
            if self.RENDER:
                self.render()

            return self.Done,truncated
        
    def _impact_conversion(self,impact_state,params,impact_conditions):

        x,z,phi,vx,vz,dphi = impact_state
        L,gamma,M_B,I_B,PD = params
        (BodyContact,Leg1Contact,Leg2Contact) = impact_conditions

        V_tx,V_perp = self._W_to_P(np.array([vx,vz]),self.Plane_Angle,deg=True)

        I_C = M_B*L**2 + I_B

        if Leg1Contact:
            
            ## CALC BETA ANGLE
            Beta = np.deg2rad(90) - phi - self.Plane_Angle_rad + gamma

            ## CALC DBETA FROM MOMENTUM CONVERSION
            H_V_perp = M_B*L*V_perp*np.cos(Beta)
            H_V_tx = -M_B*L*V_tx*np.sin(Beta)
            H_dphi = -I_B*dphi
            
            dBeta = (H_V_perp + H_V_tx + H_dphi)/(-I_C)

            return Beta,dBeta

        elif Leg2Contact:
            pass



        return None,None



    def render(self):

        ## SET DEFAULT WINDOW POSITION
        Win_Loc_z = 500
        Win_Loc_y = 700
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (Win_Loc_z,Win_Loc_y)

        ## CONVERT COORDINATES TO PIXEL LOCATION
        def c2p(Pos):

            x_offset = 1  # [m]
            y_offset = 2  # [m]

            scale_x = self.screen_width/self.world_width
            scale_y = self.screen_height/self.world_height
            
            x_p = (x_offset+Pos[0])*scale_x # [pixels]
            y_p = (y_offset+Pos[1])*scale_y # [pixels]
            
            return (x_p,y_p)
        
        ## INITIATE SCREEN AND CLOCK ON FIRST LOADING
        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pygame.time.Clock()


        ## GET CURRENT STATE
        x,z,phi,vx,vz,dphi = self._get_state()
        
        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE)

        ## ORIGIN
        pygame.draw.line(self.surf,GREEN,c2p((0,0)),c2p((0.1,0)),width=5) # X_w   
        pygame.draw.line(self.surf,BLUE, c2p((0,0)),c2p((0,0.1)),width=5) # Z_w   
        pygame.draw.circle(self.surf,RED,c2p((0,0)),radius=4,width=0)


        ## LANDING SURFACE
        pygame.draw.line(self.surf,BLACK,
                         c2p(self.Plane_Pos + self._P_to_W(np.array([-0.5,0]),self.Plane_Angle,deg=True)),
                         c2p(self.Plane_Pos + self._P_to_W(np.array([+0.5,0]),self.Plane_Angle,deg=True)),width=5)
        
        pygame.draw.line(self.surf,GREEN,c2p(self.Plane_Pos),c2p(self.Plane_Pos + self._P_to_W(np.array([0.1,0]),self.Plane_Angle,deg=True)),width=7)  # t_x   
        pygame.draw.line(self.surf,BLUE, c2p(self.Plane_Pos),c2p(self.Plane_Pos + self._P_to_W(np.array([0,0.1]),self.Plane_Angle,deg=True)),width=7)  # n_p 
        pygame.draw.circle(self.surf,RED,c2p(self.Plane_Pos),radius=4,width=0)


        ## DRAW QUADROTOR
        Pose = self._get_pose(x,z,phi)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[1]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[2]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[3]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[4]),width=3)

        ## TRIGGER INDICATOR
        if self.Pol_Trg_Flag == True:
            pygame.draw.circle(self.surf,RED,  c2p(Pose[0]),radius=4,width=0)
            pygame.draw.circle(self.surf,BLACK,c2p(Pose[0]),radius=5,width=3)
        else:
            pygame.draw.circle(self.surf,BLUE, c2p(Pose[0]),radius=4,width=0)
            pygame.draw.circle(self.surf,BLACK,c2p(Pose[0]),radius=5,width=3)


        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)

        ## WINDOW TEXT
        my_font = pygame.font.SysFont(None, 30)
        text_t_step = my_font.render(f'Time Step: {self.t:.03f}', True, BLACK)
        # text_Vz = my_font.render(f'Vz: {vz:.2f}', True, BLACK)
        # text_Vel = my_font.render(f'Vel: {Vel:.2f}  Phi: {phi:.2f}', True, BLACK)
        
        text_Tau = my_font.render(f'Tau: {self._get_obs()[0]:2.2f}', True, BLACK)
        text_Theta_x = my_font.render(f'Theta_x: {self._get_obs()[1]:2.2f}', True, BLACK)
        text_D_perp = my_font.render(f'D_perp: {self._get_obs()[2]:2.2f}', True, BLACK)
        text_Plane_Angle = my_font.render(f'Plane Angle: {self.Plane_Angle:3.1f}', True, BLACK)

        # text_dTau = my_font.render(f'dTau: {0.0:.3f}', True, BLACK)
        # text_z_acc = my_font.render(f'z_acc: {0.0:.3f}', True, BLACK)
        # text_reward = my_font.render(f'Prev Reward: {self.reward:.3f}',True, BLACK)

        ## DRAW OBJECTS TO SCREEN
        self.screen.blit(self.surf,(0,0))
        self.screen.blit(text_t_step,   (5,5))
        # self.screen.blit(text_Vel,      (5,30))
        # self.screen.blit(text_Vz,       (5,55))
        self.screen.blit(text_Tau,          (5,80))
        self.screen.blit(text_Theta_x,      (5,105))
        self.screen.blit(text_D_perp,       (5,130))
        self.screen.blit(text_Plane_Angle,  (5,155))

        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(30) # [Hz]
        pygame.display.flip()

    def close(self):

        if self.screen is not None:
            pygame.display.quit()
            pygame.quit()
            self.isopen = False

    def _iter_step(self):

        ## CURRENT STATE
        x,z,phi,vx,vz,dphi = self._get_state()

        ## STEP UPDATE
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

        self.state = (x,z,phi,vx,vz,dphi)

    def _iter_step_Rot(self,Rot_action):

        ## PARAMS
        L,gamma,M_B,I_B,PD = self._get_params()

        ## CURRENT STATE
        x,z,phi,vx,vz,dphi = self._get_state()

        ## TURN OFF BODY MOMENT IF ROTATED PAST 90 DEG
        if np.abs(phi) < np.deg2rad(90) and self.MomentCutoff == False:

            My = Rot_action*1e-3

        else: 

            self.MomentCutoff= True
            My = 0

        Thrust = np.array([0,np.sign(My)*My/PD]) # Body Coords
        Thrust_x,Thrust_z = self._B_to_W(Thrust,phi)

        ## STEP UPDATE
        self.t += self.dt

        z_acc = Thrust_z/M_B - self.g
        z = z + self.dt*vz
        vz = vz + self.dt*z_acc

        x_acc = Thrust_x/M_B
        x = x + self.dt*vx
        vx = vx + self.dt*x_acc

        phi_acc = My/I_B
        phi = phi + self.dt*dphi
        dphi = dphi + self.dt*phi_acc

        self.state = (x,z,phi,vx,vz,dphi)

    def _get_state(self):

        return self.state
    
    def _get_params(self):

        return self.params

    def _get_impact_conditions(self,x,z,phi):

        impact_flag  = False
        Body_contact = False
        Leg1_contact = False
        Leg2_contact = False

        _,Leg1_Pos,Leg2_Pos,Prop1_Pos,Prop2_Pos = self._get_pose(x,z,phi)

        ## CHECK FOR PROP CONTACT
        for Prop_Pos in [Prop1_Pos,Prop2_Pos]:

            Prop_wrt_Plane = self._W_to_P(-(self.Plane_Pos - Prop_Pos),self.Plane_Angle,deg=True)
            if Prop_wrt_Plane[1] >= 0:
                impact_flag = True
                Body_contact = True

                return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG1 CONTACT
        Leg1_wrt_Plane = self._W_to_P(-(self.Plane_Pos - Leg1_Pos),self.Plane_Angle,deg=True)
        if Leg1_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg1_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG2 CONTACT
        Leg2_wrt_Plane = self._W_to_P(-(self.Plane_Pos - Leg2_Pos),self.Plane_Angle,deg=True)
        if Leg2_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg2_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]


        return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

    def _set_state(self,x,z,phi,vx,vz,dphi):

        self.state = (x,z,phi,vx,vz,dphi)

    def _get_obs(self):

        ## UPDATE SAR POS AND VEL
        x,z,phi,vx,vz,dphi = self._get_state()
        r_BO = np.array([x,z])
        V_BO = np.array([vx,vz])

        ## PLANE POSITION AND UNIT VECTORS
        r_PO = self.Plane_Pos
        Plane_Angle = self.Plane_Angle
        n_hat,t_x = self._calc_PlaneNormal(Plane_Angle)

        ## CALC DISPLACEMENT FROM PLANE CENTER
        r_PB = r_PO - r_BO

        ## CALC RELATIVE DISTANCE AND VEL
        D_perp = r_PB.dot(n_hat)
        V_perp = V_BO.dot(n_hat)
        V_tx = V_BO.dot(t_x)

        ## CALC OPTICAL FLOW VALUES
        Theta_x = np.clip(V_tx/(D_perp + EPS),-20,20)
        Tau = np.clip(D_perp/(V_perp + EPS),0,5)

        return np.array([Tau,Theta_x,D_perp,Plane_Angle],dtype=np.float32)
    
    def _sample_flight_conditions(self):
        """This function samples the flight velocity and angle from the supplied range.
        Velocity is sampled from a uniform distribution. Phi is sampled from a set of 
        uniform distributions which are weighted such that edge cases are only sampled 10% of the time.
        Poor performance on edge cases can cause poor learning convergence.

        Returns:
            V_mag,Phi_rel: Sampled flight velocity and flight angle
        """        

        ## SAMPLE VEL FROM UNIFORM DISTRIBUTION IN VELOCITY RANGE
        Vel_Low = self.Vel_range[0]
        Vel_High = self.Vel_range[1]
        V_mag = np.random.uniform(low=Vel_Low,high=Vel_High)

        ## SAMPLE RELATIVE PHI FROM A WEIGHTED SET OF UNIFORM DISTRIBUTIONS
        Phi_rel_Low = self.Phi_rel_range[0]
        Phi_rel_High = self.Phi_rel_range[1]
        Phi_rel_Range = Phi_rel_High-Phi_rel_Low

        Dist_Num = np.random.choice([0,1,2],p=[0.1,0.8,0.1]) # Probability of sampling distribution

        if Dist_Num == 0: # Low Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_Low, high=Phi_rel_Low + 0.1*Phi_rel_Range)
        elif Dist_Num == 1: # Medium Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_Low + 0.1*Phi_rel_Range, high=Phi_rel_High - 0.1*Phi_rel_Range)
        elif Dist_Num == 2: # High Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_High - 0.1*Phi_rel_Range, high=Phi_rel_High)
       
        return V_mag,Phi_rel

    def _CalcReward(self):

        return 0
    
    def _get_pose(self,x_pos,z_pos,phi):

        (L,gamma,M_B,I_B,PD) = self.params

        ## MODEL COG
        CG = np.array([x_pos,z_pos])

        ## LEG COORDS
        L1 = np.array([ L*np.sin(gamma),-L*np.cos(gamma)])
        L2 = np.array([-L*np.sin(gamma),-L*np.cos(gamma)])

        ## PROP COORDS
        Prop1 = np.array([ PD,0])
        Prop2 = np.array([-PD,0])

        ## CONVERT BODY COORDS TO WORLD COORDS
        L1 = CG + self._B_to_W(L1,phi)
        L2 = CG + self._B_to_W(L2,phi)
        Prop1 = CG + self._B_to_W(Prop1,phi)
        Prop2 = CG + self._B_to_W(Prop2,phi)

        return np.array([CG,L1,L2,Prop1,Prop2])
    
    def _calc_PlaneNormal(self,Plane_Angle):

        Plane_Angle_rad = np.deg2rad(Plane_Angle)

        ## PRE-INIT ARRAYS
        t_x =   np.array([1,0],dtype=np.float64)
        n_hat = np.array([0,1],dtype=np.float64)

        ## DEFINE PLANE TANGENT UNIT-VECTOR
        t_x[0] = -np.cos(Plane_Angle_rad)
        t_x[1] = -np.sin(Plane_Angle_rad)

        ## DEFINE PLANE NORMAL UNIT-VECTOR
        n_hat[0] = np.sin(Plane_Angle_rad)
        n_hat[1] = -np.cos(Plane_Angle_rad)

        return n_hat,t_x
    
    def _B_to_W(self,vec,phi,deg=False):

        if deg == True:
            phi = np.deg2rad(phi)

        R_BW = np.array([
            [ np.cos(phi), np.sin(phi)],
            [-np.sin(phi), np.cos(phi)]
        ])

        return R_BW.dot(vec)

    def _P_to_W(self,vec,theta,deg=False):

        if deg == True:
            theta = np.deg2rad(theta)

        R_PW = np.array([
            [-np.cos(theta), np.sin(theta)],
            [-np.sin(theta),-np.cos(theta)]
        ])

        return R_PW.dot(vec)
    
    def _W_to_P(self,vec,theta,deg=False):

        if deg == True:
            theta = np.deg2rad(theta)

        R_WP = np.array([
            [-np.cos(theta),-np.sin(theta)],
            [ np.sin(theta),-np.cos(theta)]
        ])

        return R_WP.dot(vec)

    def _Beta_to_P(self,vec,beta,deg=False):

        if deg == True:
            beta = np.deg2rad(beta)

        R_WP = np.array([
            [-np.cos(beta), np.sin(beta)],
            [-np.sin(beta),-np.cos(beta)]
        ])

        return R_WP.dot(vec)

if __name__ == '__main__':
    env = SAR_Env_2D(Plane_Angle_range=[45,45],Tau_0=0.2)
    env.RENDER = True

    for ep in range(25):
        
        V_mag = 1
        Phi_rel = 45
        obs = env.reset(V_mag=V_mag,Phi_rel=Phi_rel)
        Done = False

        while not Done:

            # action = f(obs)
            action = env.action_space.sample()
            action[0] = 0.6
            action[1] = 0.0


            next_obs,reward,Done,truncated,_ = env.step(action)
            obs = next_obs