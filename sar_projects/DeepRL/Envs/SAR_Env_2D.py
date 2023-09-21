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

    def __init__(self):
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

        ## TESTING CONFIGS  
        # self.Vel_range = Vel_range  
        # self.Flight_Angle_range = Flight_Angle_range
        # self.Plane_Angle_range = Plane_Angle_range
        # self.a_Rot_range = a_Rot_range

        ## SAR DIMENSIONAL CONSTRAINTS
        gamma = np.deg2rad(30)  # Leg Angle [m]
        L = 150.0e-3            # Leg Length [m]
        PD = 75e-3              # Prop Distance from COM [m]
        M_B = 35.0e-3           # Body Mass [kg]
        I_B = 17.0e-6           # Body Moment of Inertia [kg*m^2]
        I_C = I_B + M_B*L**2
        self.params = (gamma,L,PD,M_B,I_B,I_C)
        self.collision_radius = max(L,PD)

        ## PLANE PARAMETERS
        self.Plane_Pos = [0.5,0]
        self.Plane_Angle = 90
        self.Plane_Angle_rad = np.radians(self.Plane_Angle)

        ## LEARNING/REWARD CONFIGS
        self.Trg_threshold = 0.5

        ## TIME CONSTRAINTS
        self.t_episode_max = 1.5   # [s]
        self.t_impact_max = 1.0   # [s]

        ## INITIAL LEARNING/REWARD CONFIGS
        self.K_ep = 0
        self.Pol_Trg_Flag = False
        self.Done = False
        self.reward = 0

        self.D_min = np.inf
        self.Tau_trg = np.inf

        

        ######################
        #   2D ENV CONFIGS
        ######################

        ## SPECIAL CONFIGS
        self.state = (0,0,0,0,0,0)

        ## PHYSICS PARAMETERS
        self.g = 9.81       # Gravity [m/s^2]
        self.dt = 0.001      # [s]
        self.t = 0          # [s]

        ## RENDERING PARAMETERS
        self.world_width = 3.0      # [m]
        self.world_height = 2.0     # [m]
        self.x_offset = 1           # [m]
        self.y_offset = 1           # [m]


        self.screen_width = 1000    # [pixels]
        self.screen_height = self.screen_width*self.world_height/self.world_width # [pixels]
        
        self.RENDER = False
        self.screen = None
        self.clock = None
        self.isopen = True

    def reset(self, seed=None, options=None, V_mag=None, Rel_Angle=None):

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

        self.impact_flag = False
        self.BodyContact_flag = False

        ## RESET/UPDATE TIME CONDITIONS
        self.start_time_episode = self._get_time()
        self.start_time_trg = np.nan
        self.start_time_impact = np.nan

        ######################
        #   2D ENV CONFIGS
        ######################

        ## RESET STATE
        self.state = (0,0,0,0.5,0,0)


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
        if action[0] <= self.Trg_threshold:

            ## 2) UPDATE STATE
            self._iter_step(n_step=10)

            # UPDATE RENDER
            if self.RENDER:
                self.render()
            

            # GRAB NEXT OBS
            next_obs = self._get_obs()

            # CHECK FOR IMPACT
            x,z,phi,vx,vz,dphi = self._get_state()
            self.impact_flag,self.impact_conditions = self._get_impact_conditions(x,z,phi)

            # UPDATE MINIMUM DISTANCE
            D_perp = next_obs[2]
            if D_perp <= self.D_min:
                self.D_min = D_perp 


            # 3) CALCULATE REWARD
            reward = 0.0


            # 4) CHECK TERMINATION
            self.Done = bool(
                self.Done
                or (self.impact_flag or self.BodyContact_flag)
            )
            terminated = self.Done
            truncated = bool(self._get_time() - self.start_time_episode >= self.t_episode_max) 

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
        elif action[0] >= self.Trg_threshold:

            # GRAB TERMINAL OBS
            terminal_obs = self._get_obs() # Attribute final reward to triggering state
            self.Tau_trg = terminal_obs[0]

            # 2) UPDATE STATE
            self.Pol_Trg_Flag = True
            # self.Finish_Sim()         


            # 3) CALC REWARD
            reward = self.Calc_Reward_PostTrg()  


            # 4) CHECK TERMINATION
            terminated = self.Done = True
            truncated = False


            # 5) RETURN VALUES
            return(
                terminal_obs,
                reward,
                terminated,
                truncated,
                {},
            )


        return obs, reward, terminated, truncated, {}

    
    
    def Calc_Reward_PreTrg(self):

        D = self._get_obs()[2]
        R_dist = np.clip(1/np.abs(D + EPS),0,15)/15

        R = R_dist*0.5
        print(f"Pre_Trg: Reward: {R:.3f} \t D: {D:.3f}")

        return R
    
    def Calc_Reward_PostTrg(self):

        D = self._get_obs()[2]
        R_dist = np.clip(1/np.abs(D + EPS),0,15)/15

        ## TAU TRIGGER REWARD
        R_tau = np.clip(1/np.abs(self.Tau_trg - 0.2),0,15)/15

        R = R_dist*0.25 + R_tau*0.75
        print(f"Post_Trg: Reward: {R:.3f} \t D: {D:.3f}")

        return R


    
    def _iter_step(self,n_step=10):

        for _ in range(n_step):

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

            self.state = (x,z,phi,vx,vz,dphi)

    def _get_state(self):

        return self.state
      
    def _get_obs(self):
        
        x,z,phi,vx,vz,dphi = self._get_state()
        
        ## POSITION AND VELOCITY VECTORS
        r_BO = np.array([x,z])
        V_BO = np.array([vx,vz])

        ## PLANE POSITION AND UNIT VECTORS
        r_PO = self.Plane_Pos
        Plane_Angle_rad = self.Plane_Angle_rad

        ## CALC DISPLACEMENT FROM PLANE CENTER
        r_PB = r_PO - r_BO

        ## CALC RELATIVE DISTANCE AND VEL
        D_tx,D_perp = self.R_WP(r_PB,Plane_Angle_rad)
        V_tx,V_perp = self.R_WP(V_BO,Plane_Angle_rad)

        ## CALC OPTICAL FLOW VALUES
        Theta_x = np.clip(V_tx/(D_perp + EPS),-20,20)
        Tau = np.clip(D_perp/(V_perp + EPS),0,5)

        ## OBSERVATION VECTOR
        obs = np.array([Tau,Theta_x,D_perp,Plane_Angle_rad],dtype=np.float32)

        return obs
    
    def _get_time(self):

        return self.t
    
    def _get_pose(self):

        gamma,L,PD,M_B,I_B,I_C = self.params
        x,z,phi,_,_,_ = self._get_state()

        ## MODEL CoG
        CG = np.array([x,z])

        ## LEG COORDS
        L1 = np.array([ L*np.sin(gamma),-L*np.cos(gamma)])
        L2 = np.array([-L*np.sin(gamma),-L*np.cos(gamma)])

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

        _,Leg1_Pos,Leg2_Pos,Prop1_Pos,Prop2_Pos = self._get_pose()

        ## CHECK FOR PROP CONTACT
        for Prop_Pos in [Prop1_Pos,Prop2_Pos]:

            Prop_wrt_Plane = self.R_WP(-(self.Plane_Pos - Prop_Pos),self.Plane_Angle_rad)
            if Prop_wrt_Plane[1] >= 0:
                impact_flag = True
                Body_contact = True

                return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG1 CONTACT
        Leg1_wrt_Plane = self.R_WP(-(self.Plane_Pos - Leg1_Pos),self.Plane_Angle_rad)
        if Leg1_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg1_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG2 CONTACT
        Leg2_wrt_Plane = self.R_WP(-(self.Plane_Pos - Leg2_Pos),self.Plane_Angle_rad)
        if Leg2_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg2_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]


        return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

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
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pygame.time.Clock()


        ## GET CURRENT STATE
        x,z,phi,vx,vz,dphi = self._get_state()

        ## GET CURRENT PARAMS
        gamma,L,PD,M_B,I_B,I_C = self.params
        
        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE)

        ## ORIGIN AXES
        pygame.draw.line(self.surf,GREEN,c2p((0,0,0)),c2p((0.1,0)),width=5) # X_w   
        pygame.draw.line(self.surf,BLUE, c2p((0,0,0)),c2p((0,0.1)),width=5) # Z_w   
        pygame.draw.circle(self.surf,RED,c2p((0,0,0)),radius=4,width=0)


        ## LANDING SURFACE
        pygame.draw.line(self.surf,GREY,
                         c2p(self.Plane_Pos + self.R_PW(np.array([-2,0]),self.Plane_Angle_rad)),
                         c2p(self.Plane_Pos + self.R_PW(np.array([+2,0]),self.Plane_Angle_rad)),width=2)
        
        pygame.draw.line(self.surf,BLACK,
                         c2p(self.Plane_Pos + self.R_PW(np.array([-0.5,0]),self.Plane_Angle_rad)),
                         c2p(self.Plane_Pos + self.R_PW(np.array([+0.5,0]),self.Plane_Angle_rad)),width=5)
    
        # ## LANDING SURFACE AXES
        pygame.draw.line(self.surf,GREEN,c2p(self.Plane_Pos),c2p(self.Plane_Pos + self.R_BW(np.array([0.1,0]),self.Plane_Angle_rad)),width=7)  # t_x   
        pygame.draw.line(self.surf,BLUE, c2p(self.Plane_Pos),c2p(self.Plane_Pos + self.R_BW(np.array([0,0.1]),self.Plane_Angle_rad)),width=7)  # n_p 
        pygame.draw.circle(self.surf,RED,c2p(self.Plane_Pos),radius=4,width=0)


        ## DRAW QUADROTOR
        Pose = self._get_pose()
        pygame.draw.line(self.surf,RED,c2p(Pose[0]),c2p(Pose[1]),width=3) # Leg 1
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[2]),width=3) # Leg 2
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[3]),width=3) # Prop 1
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[4]),width=3) # Prop 2
        pygame.draw.circle(self.surf,GREY,c2p(Pose[0]),radius=self.collision_radius*self.screen_width/self.world_width,width=2)

        ## TRIGGER INDICATOR
        if self.Pol_Trg_Flag == True:
            pygame.draw.circle(self.surf,RED,  c2p(Pose[0]),radius=4,width=0)
            pygame.draw.circle(self.surf,BLACK,c2p(Pose[0]),radius=5,width=3)
        else:
            pygame.draw.circle(self.surf,BLUE, c2p(Pose[0]),radius=4,width=0)
            pygame.draw.circle(self.surf,BLACK,c2p(Pose[0]),radius=5,width=3)

        ## BODY AXES
        pygame.draw.line(self.surf,GREEN,c2p(Pose[0]),c2p(Pose[0] + self.R_BW(np.array([0.05,0]),phi)),width=5)  # B_x   
        pygame.draw.line(self.surf,BLUE,c2p(Pose[0]),c2p(Pose[0] + self.R_BW(np.array([0,0.05]),phi)),width=5)  # B_z  


        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)

        ## WINDOW TEXT
        my_font = pygame.font.SysFont(None, 30)

        ## STATES TEXT
        text_States = my_font.render(f'States:', True, GREY)
        text_t_step = my_font.render(f'Time Step: {self.t:7.03f} [s]', True, BLACK)
        # text_V_mag = my_font.render(f'V_mag: {self.V_mag:.2f} [m/s]', True, BLACK)
        # text_Phi_rel = my_font.render(f'Alpha_rel: {self.Phi_rel:.2f} [deg]', True, BLACK)

        ## OBSERVATIONS TEXT
        text_Obs = my_font.render(f'Observations:', True, GREY)
        text_Tau = my_font.render(f'Tau: {self._get_obs()[0]:2.2f} [s]', True, BLACK)
        text_theta_x = my_font.render(f'Theta_x: {self._get_obs()[1]:2.2f} [rad/s]', True, BLACK)
        text_D_perp = my_font.render(f'D_perp: {self._get_obs()[2]:2.2f} [m]', True, BLACK)
        text_Plane_Angle = my_font.render(f'Plane Angle: {self.Plane_Angle:3.1f} [deg]', True, BLACK)

        ## ACTIONS TEXT
        text_Actions = my_font.render(f'Actions:', True, GREY)
        # text_Trg_Action = my_font.render(f'Trg_Action: {5.0:3.1f}', True, BLACK)
        # text_Rot_Action = my_font.render(f'Rot_Action: {5.0:3.1f}', True, BLACK)

        ## REWARD TEXT
        text_Other = my_font.render(f'Other:', True, GREY)
        # text_reward = my_font.render(f'Prev Reward: {self.reward:.3f}',True, BLACK)
        text_Tau_trg = my_font.render(f'Tau trg: {self.Tau_trg:.3f} [s]',True, BLACK)


        ## DRAW OBJECTS TO SCREEN
        self.screen.blit(self.surf,(0,0))
        self.screen.blit(text_States,       (5,5))
        self.screen.blit(text_t_step,       (5,5 + 25*1))
        # self.screen.blit(text_Phi_rel,      (5,5 + 25*2))
        # self.screen.blit(text_V_mag,        (5,5 + 25*3))

        self.screen.blit(text_Obs,          (5,5 + 25*5))
        self.screen.blit(text_Tau,          (5,5 + 25*6))
        self.screen.blit(text_theta_x,      (5,5 + 25*7))
        self.screen.blit(text_D_perp,       (5,5 + 25*8))
        self.screen.blit(text_Plane_Angle,  (5,5 + 25*9))

        self.screen.blit(text_Actions,      (5,5 + 25*11))
        # self.screen.blit(text_Rot_Action,   (5,5 + 25*12))
        # self.screen.blit(text_Trg_Action,   (5,5 + 25*13))

        self.screen.blit(text_Other,        (5,5 + 25*15))
        # self.screen.blit(text_reward,       (5,5 + 25*16))
        self.screen.blit(text_Tau_trg,      (5,5 + 25*17))



        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(60) # [Hz]
        pygame.display.flip()

    def close(self):

        return
    
    def R_BW(self,vec,phi):

        R_BW = np.array([
            [ np.cos(phi), np.sin(phi)],
            [-np.sin(phi), np.cos(phi)],
        ])

        return R_BW.dot(vec)
    
    def R_PW(self,vec,theta):

        R_PW = np.array([
            [-np.cos(theta), np.sin(theta)],
            [-np.sin(theta),-np.cos(theta)]
        ])

        return R_PW.dot(vec)
    
    def R_WP(self,vec,theta):

        R_WP = np.array([
            [-np.cos(theta),-np.sin(theta)],
            [ np.sin(theta),-np.cos(theta)]
        ])

        return R_WP.dot(vec)







if __name__ == '__main__':
    env = SAR_Env_2D()
    env.RENDER = True

    for ep in range(50):

        obs,_ = env.reset(V_mag=None,Rel_Angle=None)

        Done = False
        truncated = False
        while not (Done or truncated):

            action = env.action_space.sample()
            action = np.zeros_like(action)
            obs,reward,Done,truncated,_ = env.step(action)

        print(f"Episode: {ep} \t Obs: {obs[2]:.3f} \t Reward: {reward:.3f}")

    env.close()



                