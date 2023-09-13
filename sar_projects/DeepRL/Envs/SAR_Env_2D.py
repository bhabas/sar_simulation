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

class SAR_Env_2D(gym.Env):
    """Custom Environment that follows gym interface."""

    def __init__(self,GZ_Timeout=True,My_range=[-8.0,8.0],Vel_range=[1.5,3.5],Phi_rel_range=[0,90],Plane_Angle_range=[180,180],Tau_0=0.4):

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
        self.Plane_Pos = [1,1]
        self.Plane_Angle = 135

        ## SAR DIMENSIONS CONSTRAINTS 
        gamma = np.deg2rad(30)  # Leg Angle [m]
        L = 75.0e-3             # Leg Length [m]
        M_B = 0.035             # Body Mass [kg]
        I_B = 30.46e-6          # Body Moment of Inertia [kg*m^2]
        PD = 60.0e-3            # Prop Distance from COM [m]
        self.params = (L,gamma,M_B,I_B,PD)

        ## PHYSICS PARAMETERS
        self.dt = 0.005     # [s]
        self.g = 9.81       # Gravity [m/s^2]
        self.t_max = 600    # [s]


        ## RENDERING PARAMETERS
        self.screen_width = 1000
        self.RENDER = False
        self.world_width = 4.0  # [m]
        self.world_height = 4.0 # [m]

        self.screen_height = self.screen_width*self.world_height/self.world_width
        self.screen = None
        self.clock = None
        self.isopen = True



    def reset(self, seed=None, options=None, V_mag=None, Phi_rel=None):

        pass

    def step(self, action):
        
        pass
        # return observation, reward, terminated, truncated, info


    def render(self):

        ## SET DEFAULT WINDOW POSITION
        z = 500
        y = 700
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (z,y)

        ## CONVERT COORDINATES TO PIXEL LOCATION
        def c2p(Pos):

            x_offset = 2  # [m]
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

        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE)

        ## ORIGIN
        pygame.draw.line(self.surf,GREEN,c2p((0,0)),c2p((0.1,0)),width=5) # X_w   
        pygame.draw.line(self.surf,BLUE,c2p((0,0)),c2p((0,0.1)),width=5) # Z_w   
        pygame.draw.circle(self.surf,RED,c2p((0,0)),radius=4,width=0)

        

        ## LANDING SURFACE
        pygame.draw.line(self.surf,BLACK,
                         c2p(self.Plane_Pos + self._P_to_W(np.array([-1,0]),self.Plane_Angle)),
                         c2p(self.Plane_Pos + self._P_to_W(np.array([+1,0]),self.Plane_Angle)),width=5)
        
        pygame.draw.line(self.surf,GREEN,c2p(self.Plane_Pos),c2p(self.Plane_Pos + self._P_to_W(np.array([0.1,0]),self.Plane_Angle)),width=7)  # t_x   
        pygame.draw.line(self.surf,BLUE, c2p(self.Plane_Pos),c2p(self.Plane_Pos + self._P_to_W(np.array([0,0.1]),self.Plane_Angle)),width=7)  # n_p 
        pygame.draw.circle(self.surf,RED,c2p(self.Plane_Pos),radius=4,width=0)


        ## DRAW QUADROTOR
        Pose = self._get_pose(0,1,0)
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

        ## DRAW OBJECTS TO SCREEN
        self.screen.blit(self.surf,(0,0))

        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(60) # [Hz]
        pygame.display.flip()

    def close(self):

        if self.screen is not None:
            pygame.display.quit()
            pygame.quit()
            self.isopen = False

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
    
    def _B_to_W(self,vec,phi):

        phi = np.deg2rad(phi)
        R_BW = np.array([
            [ np.cos(phi), np.sin(phi)],
            [-np.sin(phi), np.cos(phi)]
        ])

        return R_BW.dot(vec)

    def _P_to_W(self,vec,theta):

        theta = np.deg2rad(theta)
        R_PW = np.array([
            [-np.cos(theta), np.sin(theta)],
            [-np.sin(theta),-np.cos(theta)]
        ])

        return R_PW.dot(vec)

if __name__ == '__main__':
    env = SAR_Env_2D()

    while True:
        env.render()