import gymnasium as gym
import numpy as np
from gymnasium import spaces

import pygame
import os

## DEFINE COLORS
WHITE = (255,255,255)
GREY = (200,200,200)
BLACK = (0,0,0)
RED = (204,0,0)
BLUE = (29,123,243)
GREEN = (0,153,0)
PURPLE = (76,0,153)
ORANGE = (255,128,0)

EPS = 1e-6 # Epsilon (Prevent division by zero)

class SAR_Env_2D(gym.Env):
    """Custom Environment that follows gym interface."""

    def __init__(self,GZ_Timeout=True,My_range=[-8.0e-3,8.0e-3],Vel_range=[1.5,3.5],Flight_Angle_range=[0,90],Plane_Angle_range=[90,180]):

        gym.Env.__init__(self)

        ## ENV CONFIG SETTINGS
        self.Env_Name = "SAR_Env_2D"

        ## DOMAIN RANDOMIZATION SETTINGS
        self.Mass_std = 0.5e-3  # [kg]
        self.Iyy_std = 1.5e-6   # [kg*m^2]

        ## TESTING CONDITIONS     
        self.Vel_range = Vel_range  
        self.Flight_Angle_range = Flight_Angle_range
        self.Plane_Angle_range = Plane_Angle_range
        self.My_range = My_range

        ## RESET INITIAL VALUES
        self.K_ep = 0
        self.Trg_threshold = 0.75
        self.D_min = np.inf
        self.Tau_trg = np.inf
        self.Done = False
        self.Pol_Trg_Flag = False
        self.reward = 0

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
        gamma = np.deg2rad(30)  # Leg Angle [m]
        L = 150.0e-3            # Leg Length [m]
        PD = 32.5e-3           # Prop Distance from COM [m]
        M_B = 35.0e-3           # Body Mass [kg]
        I_B = 17.0e-6           # Body Moment of Inertia [kg*m^2]
        self.params = (L,gamma,M_B,I_B,PD)
        self.collision_radius = max(L,PD)

        ## INITIAL POSITION CONSTRAINTS
        My_max = abs(max(My_range,key=abs))
        self.t_rot = np.sqrt((2*I_B*np.radians(360))/(0.5*My_max + EPS)) ## Allow enough time for a full rotation
        self.t_rot = 0.3

        ## TIME CONSTRAINTS
        self.t_episode_max = 2.0   # [s]
        self.t_impact_max = 1.0   # [s]

        ## PHYSICS PARAMETERS
        self.g = 9.81       # Gravity [m/s^2]
        self.dt = 0.005     # [s]
        self.t = 0          # [s]

        ## INITIAL STATE
        self.state = (0,0.1,0,0,0,0) # Initial State (X_pos,Z_pos,phi,Vx,Vz,dphi)

        ## RENDERING PARAMETERS
        self.RENDER = False
        self.world_width = 3.0      # [m]
        self.world_height = 2.0     # [m]
        self.screen_width = 1000    # [pixels]
        self.screen_height = self.screen_width*self.world_height/self.world_width
        
        self.screen = None
        self.clock = None
        self.isopen = True

    def reset(self, seed=None, options=None, V_mag=None, Phi_rel=None):

        ## RESET RECORDED VALUES
        self.Done = False
        self.D_min = np.inf       # Reset max distance from landing surface [m]
        self.Tau_trg = np.inf     # Reset Tau triggering value [s]
        self.obs_trg = np.zeros_like(self.observation_space.high)
        self.action_trg = np.zeros_like(self.action_space.high)
        self.phi_impact = np.nan
        self.pad_connections = 0
        self.K_ep += 1
        self.reward = 0

        ## SET PLANE POSE
        Plane_Angle_Low = self.Plane_Angle_range[0]
        Plane_Angle_High = self.Plane_Angle_range[1]
        self.Plane_Angle = np.random.uniform(Plane_Angle_Low,Plane_Angle_High)
        self.Plane_Angle_rad = np.radians(self.Plane_Angle)

        ## SAMPLE VELOCITY AND FLIGHT ANGLE
        if V_mag == None or Phi_rel == None:
            V_mag,Phi_rel = self._sample_flight_conditions()

        else:
            V_mag = V_mag           # Flight velocity
            Phi_rel = Phi_rel   # Flight angle  

        self.Phi_rel = Phi_rel
        self.V_mag = V_mag


        ## RESET POSITION RELATIVE TO LANDING SURFACE (BASED ON STARTING TAU VALUE)
        # (Derivation: Research_Notes_Book_3.pdf (9/17/23))

        ## RELATIVE VEL VECTORS
        V_perp = V_mag*np.sin(np.deg2rad(Phi_rel))
        V_tx = V_mag*np.cos(np.deg2rad(Phi_rel))
        V_BP = np.array([V_tx,V_perp])

        ## CONVERT RELATIVE VEL VECTORS TO WORLD COORDS
        V_BO = self._P_to_W(V_BP,self.Plane_Angle,deg=True)
        
        ## CALCULATE STARTING TAU VALUE
        Tau_min = self.t_rot    # TTC allowing for full body rotation before any body part impacts
        Tau_B = (self.collision_radius + Tau_min*V_perp)/V_perp
        

        ## CALC STARTING POSITION IN GLOBAL COORDS
        r_PO = np.array(self.Plane_Pos)                             # Plane Position wrt to origin
        r_PB = np.array([(0.5*Tau_B + Tau_min)*V_tx, (0.5*Tau_B + Tau_B)*V_perp])               # Plane Position wrt to Body
        r_BO = r_PO - self._P_to_W(r_PB,self.Plane_Angle,deg=True)  # Body Position wrt to origin

        ## LAUNCH QUAD W/ DESIRED VELOCITY
        self._set_state(r_BO[0],r_BO[1],np.radians(0.1),V_BO[0],V_BO[1],0)

        ## RESET/UPDATE RUN CONDITIONS
        self.t = 0
        self.start_time_episode = self.t
        self.start_time_impact = np.nan

        ## RESET 2D SIM FLAGS
        self.impact_flag = False
        self.BodyContact_flag = False
        self.MomentCutoff = False
        self.Pol_Trg_Flag = False

        ## UPDATE RENDER
        if self.RENDER:
            self.render()
        

        return self._get_obs(),{}

    def step(self, action):

        # if self._get_obs()[0] <= 0.16:
        #     action[0] = 1

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
            truncated = bool(self.t - self.start_time_episode > self.t_episode_max) # EPISODE TIME-OUT 

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

            self.Pol_Trg_Flag = True

            ## GRAB CURRENT OBSERVATION
            terminal_obs = self._get_obs()   # Return this observation because reward and future 
                                    # are defined by action taken here. Not obs at end of episode.

            ## SAVE TRIGGERING OBSERVATION AND ACTIONS
            self.obs_trg = terminal_obs
            self.Tau_trg = terminal_obs[0]
            self.action_trg = action       

            ## COMPLETE REST OF SIMULATION
            # terminated,truncated = self._finish_sim(action)       
            self.Done = True
            terminated = self.Done
            truncated = False

            ## CALCULATE REWARD
            reward = self.CalcReward()    

            return(
                terminal_obs,
                reward,
                terminated,
                truncated,
                {},
            )
        
    def render(self):

        ## SET DEFAULT WINDOW POSITION
        Win_Loc_z = 500
        Win_Loc_y = 700
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (Win_Loc_z,Win_Loc_y)

        ## CONVERT COORDINATES TO PIXEL LOCATION
        def c2p(Pos):

            x_offset = 1  # [m]
            y_offset = 1  # [m]

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

        ## GET CURRENT PARAMS
        (L,gamma,M_B,I_B,PD) = self._get_params()
        
        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE)

        ## ORIGIN AXES
        pygame.draw.line(self.surf,GREEN,c2p((0,0)),c2p((0.1,0)),width=5) # X_w   
        pygame.draw.line(self.surf,BLUE, c2p((0,0)),c2p((0,0.1)),width=5) # Z_w   
        pygame.draw.circle(self.surf,RED,c2p((0,0)),radius=4,width=0)


        ## LANDING SURFACE
        pygame.draw.line(self.surf,GREY,
                         c2p(self.Plane_Pos + self._P_to_W(np.array([-2,0]),self.Plane_Angle,deg=True)),
                         c2p(self.Plane_Pos + self._P_to_W(np.array([+2,0]),self.Plane_Angle,deg=True)),width=2)
        
        pygame.draw.line(self.surf,BLACK,
                         c2p(self.Plane_Pos + self._P_to_W(np.array([-0.5,0]),self.Plane_Angle,deg=True)),
                         c2p(self.Plane_Pos + self._P_to_W(np.array([+0.5,0]),self.Plane_Angle,deg=True)),width=5)
    
        ## LANDING SURFACE AXES
        pygame.draw.line(self.surf,GREEN,c2p(self.Plane_Pos),c2p(self.Plane_Pos + self._P_to_W(np.array([0.1,0]),self.Plane_Angle,deg=True)),width=7)  # t_x   
        pygame.draw.line(self.surf,BLUE, c2p(self.Plane_Pos),c2p(self.Plane_Pos + self._P_to_W(np.array([0,0.1]),self.Plane_Angle,deg=True)),width=7)  # n_p 
        pygame.draw.circle(self.surf,RED,c2p(self.Plane_Pos),radius=4,width=0)


        ## DRAW QUADROTOR
        Pose = self._get_pose(x,z,phi)
        pygame.draw.line(self.surf,RED,c2p(Pose[0]),c2p(Pose[1]),width=3) # Leg 1
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[2]),width=3) # Leg 2
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[3]),width=3) # Prop 1
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[4]),width=3) # Prop 2
        pygame.draw.circle(self.surf,GREY,c2p(Pose[0]),radius=self.collision_radius*self.screen_width/self.world_width,width=2)


        ## BODY AXES
        pygame.draw.line(self.surf,GREEN,c2p(Pose[0]),c2p(Pose[0] + self._B_to_W(np.array([0.05,0]),phi)),width=5)  # B_x   
        pygame.draw.line(self.surf,BLUE,c2p(Pose[0]),c2p(Pose[0] + self._B_to_W(np.array([0,0.05]),phi)),width=5)  # B_z  



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

        ## STATES TEXT
        text_States = my_font.render(f'States:', True, GREY)
        text_t_step = my_font.render(f'Time Step: {self.t:7.03f} [s]', True, BLACK)
        text_V_mag = my_font.render(f'V_mag: {self.V_mag:.2f} [m/s]', True, BLACK)
        text_Phi_rel = my_font.render(f'Alpha_rel: {self.Phi_rel:.2f} [deg]', True, BLACK)

        ## OBSERVATIONS TEXT
        text_Obs = my_font.render(f'Observations:', True, GREY)
        text_Tau = my_font.render(f'Tau: {self._get_obs()[0]:2.2f} [s]', True, BLACK)
        text_Theta_x = my_font.render(f'Theta_x: {self._get_obs()[1]:2.2f} [rad/s]', True, BLACK)
        text_D_perp = my_font.render(f'D_perp: {self._get_obs()[2]:2.2f} [m]', True, BLACK)
        text_Plane_Angle = my_font.render(f'Plane Angle: {self.Plane_Angle:3.1f} [deg]', True, BLACK)

        ## ACTIONS TEXT
        text_Actions = my_font.render(f'Actions:', True, GREY)
        text_Trg_Action = my_font.render(f'Trg_Action: {5.0:3.1f}', True, BLACK)
        text_Rot_Action = my_font.render(f'Rot_Action: {5.0:3.1f}', True, BLACK)

        ## REWARD TEXT
        text_Other = my_font.render(f'Other:', True, GREY)
        text_reward = my_font.render(f'Prev Reward: {self.reward:.3f}',True, BLACK)
        text_Tau_trg = my_font.render(f'Tau trg: {self.Tau_trg:.3f} [s]',True, BLACK)


        ## DRAW OBJECTS TO SCREEN
        self.screen.blit(self.surf,(0,0))
        self.screen.blit(text_States,       (5,5))
        self.screen.blit(text_t_step,       (5,5 + 25*1))
        self.screen.blit(text_Phi_rel,      (5,5 + 25*2))
        self.screen.blit(text_V_mag,        (5,5 + 25*3))


        # self.screen.blit(text_Vel,      (5,30))
        # self.screen.blit(text_Vz,       (5,55))

        self.screen.blit(text_Obs,          (5,5 + 25*5))
        self.screen.blit(text_Tau,          (5,5 + 25*6))
        self.screen.blit(text_Theta_x,      (5,5 + 25*7))
        self.screen.blit(text_D_perp,       (5,5 + 25*8))
        self.screen.blit(text_Plane_Angle,  (5,5 + 25*9))

        self.screen.blit(text_Actions,      (5,5 + 25*11))
        self.screen.blit(text_Rot_Action,   (5,5 + 25*12))
        self.screen.blit(text_Trg_Action,   (5,5 + 25*13))

        self.screen.blit(text_Other,        (5,5 + 25*15))
        self.screen.blit(text_reward,       (5,5 + 25*16))
        self.screen.blit(text_Tau_trg,      (5,5 + 25*17))



        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(60) # [Hz]
        pygame.display.flip()

    def close(self):

        if self.screen is not None:
            pygame.display.quit()
            pygame.quit()
            self.isopen = False

    def _finish_sim(self,action):

        ## SCALE ACTION
        scaled_action = 0.5 * (action[1] + 1) * (self.My_range[1] - self.My_range[0]) + self.My_range[0]

        ## START PROJECTILE FLIGHT
        while not self.Done:
            
            ## CHECK FOR IMPACT
            x,z,phi,vx,vz,dphi = self._get_state()
            L,gamma,M_B,I_B,PD = self._get_params()
            self.impact_flag,self.impact_conditions = self._get_impact_conditions(x,z,phi)

            ## IMPACT FALSE
            if self.impact_flag == False:

                ## UPDATE STEP
                self._iter_step_Rot(scaled_action)

                ## UPDATE MINIMUM DISTANCE
                D_perp = self._get_obs()[2]
                if not self.Done:
                    if D_perp <= self.D_min:
                        self.D_min = D_perp 

                ## UPDATE RENDER
                if self.RENDER:
                    self.render()

            ## IMPACT TRUE
            elif self.impact_flag == True:

                ## START IMPACT TIMER
                start_time_impact = self.t
                self.phi_impact = phi
                (BodyContact,Leg1Contact,Leg2Contact) = self.impact_conditions

                ## BODY CONTACT
                if BodyContact == True:
                    self.BodyContact_flag = True
                    self.pad_connections = 0
                    self.Done = True

                ## LEG 1 CONTACT
                elif Leg1Contact == True:

                    beta,dbeta = self._impact_conversion(self.state,self.params,self.impact_conditions)

                    ## FIND IMPACT COORDS (wrt World-Coords)
                    r_C1_W = self._get_pose(x,z,phi)[1]

                    while not self.Done:

                        ## ITERATE THROUGH SWING
                        beta,dbeta = self._iter_step_Swing(beta,dbeta,impact_leg=1)

                        ## CHECK FOR END CONDITIONS
                        if beta >= self._beta_landing(impact_leg=1):
                            self.BodyContact_flag = False
                            self.pad_connections = 4
                            self.Done = True

                        elif beta <= self._beta_prop(impact_leg=1):
                            self.BodyContact_flag = True
                            self.pad_connections = 2
                            self.Done = True

                        elif self.t - start_time_impact >= self.t_impact_max:
                            self.BodyContact_flag = False
                            self.pad_connections = 2
                            self.Done = True


                        ## CONVERT BODY BACK TO WORLD COORDS
                        temp = self._Beta1_to_P(np.array([L,0]),beta)
                        r_B_C1 = self._P_to_W(temp,self.Plane_Angle_rad)
                        r_B_W = r_C1_W + r_B_C1

                        phi = np.deg2rad(90) - beta - self.Plane_Angle_rad + gamma
                        self.state = (r_B_W[0],r_B_W[1],phi,0,0,0)

                        ## UPDATE MINIMUM DISTANCE
                        D_perp = self._get_obs()[2]
                        if not self.Done:
                            if D_perp <= self.D_min:
                                self.D_min = D_perp 

                        if self.RENDER:
                            self.render()

                ## LEG 2 CONTACT
                elif Leg2Contact == True:

                    beta_2,dbeta_2 = self._impact_conversion(self.state,self.params,self.impact_conditions)

                    ## FIND IMPACT COORDS (wrt World-Coords)
                    r_C2_W = self._get_pose(x,z,phi)[2]
                    
                    while not self.Done:

                        ## ITERATE THROUGH SWING
                        beta_2,dbeta_2 = self._iter_step_Swing(beta_2,dbeta_2,impact_leg=2)

                        ## CHECK FOR END CONDITIONS
                        if beta_2 >= self._beta_landing(impact_leg=2):
                            self.BodyContact_flag = False
                            self.pad_connections = 4
                            self.Done = True

                        elif beta_2 <= self._beta_prop(impact_leg=2):
                            self.BodyContact_flag = True
                            self.pad_connections = 2
                            self.Done = True

                        elif self.t - start_time_impact >= self.t_impact_max:
                            self.BodyContact_flag = False
                            self.pad_connections = 2
                            self.Done = True

                        ## CONVERT BODY BACK TO WORLD COORDS
                        temp = self._Beta2_to_P(np.array([L,0]),beta_2)
                        r_B_C2 = self._P_to_W(temp,self.Plane_Angle_rad)
                        r_B_W = r_C2_W + r_B_C2

                        phi = (-np.deg2rad(90) + beta_2 - self.Plane_Angle_rad - gamma)
                        self.state = (r_B_W[0],r_B_W[1],phi,0,0,0)

                        ## UPDATE MINIMUM DISTANCE
                        D_perp = self._get_obs()[2]
                        if not self.Done:
                            if D_perp <= self.D_min:
                                self.D_min = D_perp 

                        if self.RENDER:
                            self.render()

            truncated = bool(self.t - self.start_time_episode > self.t_episode_max) # EPISODE TIME-OUT 

            ## CHECK FOR DONE
            self.Done = bool(
                self.Done
                or truncated
                or (z <= -0.7 and vz <= -1) # IF SAR IS FALLING
            )

            
        return self.Done,truncated

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

    def _iter_step_Rot(self,Rot_action,step=1):

        ## PARAMS
        L,gamma,M_B,I_B,PD = self._get_params()

        ## CURRENT STATE
        x,z,phi,vx,vz,dphi = self._get_state()

        ## TURN OFF BODY MOMENT IF ROTATED PAST 90 DEG
        if np.abs(phi) < np.deg2rad(90) and self.MomentCutoff == False:

            My = Rot_action

        else: 

            self.MomentCutoff= True
            My = 0

        for _ in range(step):
            ## STEP UPDATE
            self.t += self.dt

            z_acc = -self.g
            z = z + self.dt*vz
            vz = vz + self.dt*z_acc

            x_acc = 0
            x = x + self.dt*vx
            vx = vx + self.dt*x_acc

            phi_acc = My/I_B
            phi = phi + self.dt*dphi
            dphi = dphi + self.dt*phi_acc

            self.state = (x,z,phi,vx,vz,dphi)

    def _iter_step_Swing(self,beta,dbeta,impact_leg):

        L,gamma,M_B,I_B,PD = self._get_params()

        ## INERTIA ABOUT CONTACT POINT
        I_C = M_B*L**2 + I_B

        if impact_leg==1:

            ## GRAVITY MOMENT
            M_g = -M_B*self.g*L*np.cos(beta)*np.cos(self.Plane_Angle_rad) \
                    + M_B*self.g*L*np.sin(beta)*np.sin(self.Plane_Angle_rad)
            
        if impact_leg==2:

            ## GRAVITY MOMENT
            M_g = -M_B*self.g*L*np.cos(beta)*np.cos(self.Plane_Angle_rad) \
                    - M_B*self.g*L*np.sin(beta)*np.sin(self.Plane_Angle_rad)
            
        ## ITER STEP BETA
        self.t += self.dt
        beta_acc = M_g/I_C
        beta = beta + self.dt*dbeta
        dbeta = dbeta + self.dt*beta_acc

        return beta,dbeta

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
        Phi_rel_Low = self.Flight_Angle_range[0]
        Phi_rel_High = self.Flight_Angle_range[1]
        Flight_Angle_range = Phi_rel_High-Phi_rel_Low

        Dist_Num = np.random.choice([0,1,2],p=[0.1,0.8,0.1]) # Probability of sampling distribution

        if Dist_Num == 0: # Low Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_Low, high=Phi_rel_Low + 0.1*Flight_Angle_range)
        elif Dist_Num == 1: # Medium Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_Low + 0.1*Flight_Angle_range, high=Phi_rel_High - 0.1*Flight_Angle_range)
        elif Dist_Num == 2: # High Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_High - 0.1*Flight_Angle_range, high=Phi_rel_High)
       
        return V_mag,Phi_rel

    # def _CalcReward(self):

    #     x,z,phi,vx,vz,dphi = self._get_state()
    #     L,gamma,M_B,I_B,PD = self._get_params()

    #     ## DISTANCE REWARD 
    #     Distance_min = L*np.cos(gamma)*1.1
    #     if self.D_min < Distance_min:
    #         R_dist = 1
    #     elif Distance_min < self.D_min:
    #         R_dist = np.exp(-(self.D_min - Distance_min)/0.5)
        
    #     ## POLICY TRIGGER REWARD
    #     # if self.Pol_Trg_Flag == True:
    #     #     R_trg = 1
    #     # else:
    #     #     R_trg = 0

    #     ## TAU TRIGGER REWARD
    #     R_trg = np.clip(1/np.abs(self.Tau_trg - 0.2),0,15)/15


    #     ## SOLVE FOR MINIMUM PHI IMPACT ANGLE VIA GEOMETRIC CONSTRAINTS
    #     a = np.sqrt(PD**2 + L**2 - 2*PD*L*np.cos(np.pi/2-gamma))
    #     beta_min = np.arccos((L**2 + a**2 - PD**2)/(2*a*L))
    #     self.phi_rel_min = np.abs(np.arcsin(L/PD*np.sin(beta_min))-np.pi)
    #     self.phi_rel_min = np.rad2deg(self.phi_rel_min)

    #     self.phi_rel_impact = self.phi_impact + (self.Plane_Angle_rad - np.pi)
    #     self.phi_rel_impact = np.rad2deg(self.phi_rel_impact)


    #     ## IMPACT ANGLE REWARD # (Derivation: Research_Notes_Book_3.pdf (6/21/23))
    #     OverRotate_flag = False

    #     if self.phi_rel_impact < -200:
    #         R_angle = 0
    #         OverRotate_flag = True
    #     elif -200 <= self.phi_rel_impact < -self.phi_rel_min:
    #         R_angle = 1
    #     elif -self.phi_rel_min <= self.phi_rel_impact < 0:
    #         R_angle = self.phi_rel_impact/-self.phi_rel_min
    #     elif 0 <= self.phi_rel_impact < self.phi_rel_min:
    #         R_angle = self.phi_rel_impact/self.phi_rel_min
    #     elif self.phi_rel_min <= self.phi_rel_impact < 200:
    #         R_angle = 1
    #     elif self.phi_rel_impact >= 200:
    #         R_angle = 0
    #         OverRotate_flag = True
    #     else:
    #         R_angle = 0


    #     ## PAD CONTACT REWARD
    #     if self.pad_connections >= 3: 
    #         if self.BodyContact_flag == False:
    #             R_legs = 1.0
    #         else:
    #             R_legs = 0.3

    #     elif self.pad_connections == 2: 
    #         if self.BodyContact_flag == False:
    #             R_legs = 0.6
    #         else:
    #             R_legs = 0.1

    #     else:
    #         R_legs = 0.0

    #     self.reward_vals = [R_dist,R_trg,R_angle,R_legs,0]
    #     self.reward = 0.7*R_dist + 0.3*R_trg + 0.0*R_angle + 0.0*R_legs*(not OverRotate_flag)
    #     # print(self.reward_vals)
        

    #     return self.reward
    
    def CalcReward(self):

        x,z,phi,vx,vz,dphi = self._get_state()
        L,gamma,M_B,I_B,PD = self._get_params()

        ## DISTANCE REWARD 
        R_dist = np.clip(1/np.abs(self.D_min + 1e-3),0,15)/15
        
        ## TAU TRIGGER REWARD
        R_tau = np.clip(1/np.abs(self.Tau_trg - 0.2),0,15)/15

        self.reward_vals = [R_dist,R_tau,0,0,0]
        self.reward = 0.0*R_dist + 1.0*R_tau

        return self.reward
        
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
    
    def _beta_landing(self,impact_leg):

        L,gamma,M_B,I_B,PD = self._get_params()

        if impact_leg == 1:
            beta_max = np.pi/2 + gamma
            return beta_max

        elif impact_leg == 2:
            beta_min = np.pi/2 + gamma
            return beta_min
        
    def _beta_prop(self,impact_leg):

        L,gamma,M_B,I_B,PD = self._get_params()

        if impact_leg == 1:

            a = np.sqrt(PD**2 + L**2 - 2*PD*L*np.cos(np.pi/2-gamma))
            beta_min = np.arccos((L**2 + a**2 - PD**2)/(2*a*L))

            return beta_min

        elif impact_leg == 2:
            
            a = np.sqrt(PD**2 + L**2 - 2*PD*L*np.cos(np.pi/2-gamma))
            beta_max = np.arccos((L**2 + a**2 - PD**2)/(2*a*L))

            return beta_max

    def _impact_conversion(self,impact_state,params,impact_conditions):

        x,z,phi,vx,vz,dphi = impact_state
        L,gamma,M_B,I_B,PD = params
        (BodyContact,Leg1Contact,Leg2Contact) = impact_conditions

        V_tx,V_perp = self._W_to_P(np.array([vx,vz]),self.Plane_Angle,deg=True)

        I_C = M_B*L**2 + I_B

        if Leg1Contact:

            ## CALC BETA ANGLE
            Beta_1 = np.pi/2 - phi - self.Plane_Angle_rad + gamma
            Beta_1 = Beta_1 % np.pi # Counter effects from multiple rotations
            Beta_1_deg = np.degrees(Beta_1)

            ## CALC DBETA FROM MOMENTUM CONVERSION
            H_V_perp = M_B*L*V_perp*np.cos(Beta_1)
            H_V_tx = -M_B*L*V_tx*np.sin(Beta_1)
            H_dphi = I_B*dphi
            dBeta_1 = (H_V_perp + H_V_tx + H_dphi)*1/(-I_C)

            return Beta_1,dBeta_1

        elif Leg2Contact:

            ## CALC BETA ANGLE
            Beta_2 = np.pi/2 + phi + self.Plane_Angle_rad + gamma
            Beta_2 = Beta_2 % np.pi
            Beta2_deg = np.degrees(Beta_2)


            ## CALC DBETA FROM MOMENTUM CONVERSION
            H_V_perp = -M_B*L*V_perp*np.cos(Beta_2)
            H_V_tx = -M_B*L*V_tx*np.sin(Beta_2)
            H_dphi = I_B*dphi
            
            dBeta_2 = (H_V_perp + H_V_tx + H_dphi)*1/(I_C)

            return Beta_2,dBeta_2



        return None,None


## COORDINATE TRANSFORMS ##
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

    def _Beta1_to_P(self,vec,beta1,deg=False):

        if deg == True:
            beta1 = np.deg2rad(beta1)

        R_Beta1P = np.array([
            [-np.cos(beta1), np.sin(beta1)],
            [-np.sin(beta1),-np.cos(beta1)]
        ])

        return R_Beta1P.dot(vec)
    
    def _Beta2_to_P(self,vec,beta2,deg=False):

        if deg == True:
            beta2 = np.deg2rad(beta2)

        R_Beta1P = np.array([
            [ np.cos(beta2),-np.sin(beta2)],
            [-np.sin(beta2),-np.cos(beta2)]
        ])

        return R_Beta1P.dot(vec)

if __name__ == '__main__':

    env = SAR_Env_2D(Vel_range=[1.0,1.0],Flight_Angle_range=[45,45],Plane_Angle_range=[180,180],My_range=[0,0])
    env.RENDER = True
    

    for ep in range(500):

        obs = env.reset(V_mag=None,Phi_rel=None)
        Done = False

        while not Done:

            # action = f(obs)
            action = env.action_space.sample()
            print(action)
            action[0] = 0
            action[1] = 0


            next_obs,reward,Done,truncated,_ = env.step(action)
            obs = next_obs

        print(f"Reward: {reward:.2f}")

