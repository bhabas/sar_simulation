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

        ## DEFINE OBSERVATION SPACE
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.obs_trg = np.zeros(self.observation_space.shape,dtype=np.float32) # Obs values at triggering

        ## DEFINE ACTION SPACE
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.action_trg = np.zeros(self.action_space.shape,dtype=np.float32) # Action values at triggering

        ## PLANE PARAMETERS
        self.Plane_Pos = [2.0,-2.0]
        self.Plane_Angle = 45

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
        self.world_width = 8.0  # [m]
        self.world_height = 8.0 # [m]

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

            x_offset = 4  # [m]
            y_offset = 4  # [m]

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
        pygame.draw.circle(self.surf,BLACK,c2p((0,0)),radius=7,width=3)
        pygame.draw.line(self.surf,GREEN,c2p((0,0)),c2p((0.2,0)),width=3) # X_w   
        pygame.draw.line(self.surf,BLUE,c2p((0,0)),c2p((0,0.2)),width=3) # Z_w   

        

        ## LANDING SURFACE
        pygame.draw.line(self.surf,BLACK,
                         c2p(self.Plane_Pos + self.P_to_W(np.array([-1,0]))),
                         c2p(self.Plane_Pos + self.P_to_W(np.array([+1,0]))),width=5)
        
        pygame.draw.line(self.surf,GREEN,c2p(self.Plane_Pos),c2p(self.Plane_Pos + self.P_to_W(np.array([0.2,0]))),width=3)  # t_x   
        pygame.draw.line(self.surf,BLUE, c2p(self.Plane_Pos),c2p(self.Plane_Pos + self.P_to_W(np.array([0,0.2]))),width=3)  # n_p 
        pygame.draw.circle(self.surf,RED,c2p(self.Plane_Pos),radius=4,width=0)





        # pygame.draw.line(self.surf,BLACK,c2p((-5,self.Plane_Pos[2])),c2p((5,self.Plane_Pos[2])),width=2) # Landing Surface Plane
        # pygame.draw.line(self.surf,BLACK,c2p((0,-5)),c2p((0,5)),width=1) # Z-axis       
        # pygame.draw.line(self.surf,BLACK,c2p((-5,0)),c2p((5,0)),width=2) #  Ground Line


        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)

        ## DRAW OBJECTS TO SCREEN
        self.screen.blit(self.surf,     (0,0))

        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(60) # [Hz]
        pygame.display.flip()

    def close(self):

        if self.screen is not None:
            pygame.display.quit()
            pygame.quit()
            self.isopen = False

    def P_to_W(self,vec):

        theta = np.deg2rad(self.Plane_Angle)
        R_PW = np.array([
            [-np.cos(theta), np.sin(theta)],
            [-np.sin(theta),-np.cos(theta)]
        ])

        return R_PW.dot(vec)

if __name__ == '__main__':
    env = SAR_Env_2D()

    while True:
        env.render()