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
        self.g = 9.81       # Gravity [m/s^2]
        self.dt = 0.005     # [s]
        self.t = 0          # [s]
        self.t_max = 5      # [s]
        self.state = (0,0.1,0,0.1,0,0) # Initial State (X_pos,V_x,Z_pos,V_z,phi,dphi)


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

        ## SET PLANE POSE
        Plane_Angle_Low = self.Plane_Angle_range[0]
        Plane_Angle_High = self.Plane_Angle_range[1]
        self.Plane_Angle = np.random.uniform(Plane_Angle_Low,Plane_Angle_High)

        return self._get_obs(),{}

    def step(self, action):
        
        if self.RENDER:
            self.render()

        ## UPDATE SIM 
        self._iter_step()

        reward = 0
        terminated = False
        truncated = False

        return (
            self._get_obs(),
            reward,
            terminated,
            truncated,
            {},

        )


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


        ## GET CURRENT STATE
        x,vx,z,vz,phi,dphi = self._get_state()
        
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
        self.clock.tick(60) # [Hz]
        pygame.display.flip()

    def close(self):

        if self.screen is not None:
            pygame.display.quit()
            pygame.quit()
            self.isopen = False

    def _iter_step(self):

        ## CURRENT STATE
        x,vx,z,vz,phi,dphi = self._get_state()

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

        self.state = (x,vx,z,vz,phi,dphi)

    def _get_state(self):

        return self.state

    def _get_obs(self):

        ## UPDATE SAR POS AND VEL
        x,vx,z,vz,theta,dtheta = self._get_state()
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

if __name__ == '__main__':
    env = SAR_Env_2D(Plane_Angle_range=[135,135])
    env.RENDER = True

    for ep in range(25):
        
        env.reset()
        Done = False

        while not Done:
            action = env.action_space.sample()
            obs,reward,Done,truncated,_ = env.step(action)