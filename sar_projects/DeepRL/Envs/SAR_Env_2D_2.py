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

        ## LEARNING/REWARD CONFIGS
        self.Trg_threshold = 0.5


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

        ## PHYSICS PARAMETERS
        self.g = 9.81       # Gravity [m/s^2]
        self.dt = 0.005     # [s]
        self.t = 0          # [s]

        ## RENDERING PARAMETERS
        self.RENDER = False
        self.world_width = 3.0      # [m]
        self.world_height = 2.0     # [m]
        self.screen_width = 1000    # [pixels]
        self.screen_height = self.screen_width*self.world_height/self.world_width
        
        self.screen = None
        self.clock = None
        self.isopen = True

    def reset(self, seed=None, options=None, V_mag=None, Rel_Angle=None):

        ## RESET LEARNING/REWARD CONDITIONS
        self.K_ep += 1
        self.Done = False
        self.Pol_Trg_Flag = False
        self.reward = 0

        self.D_min = np.inf
        self.Tau_trg = np.inf

        
        return self._get_obs(), {}




    def step(self, action):

        ########## PRE-POLICY TRIGGER ##########
        if action[0] <= self.Trg_threshold:

            ## GRAB NEXT OBS
            next_obs = self._get_obs()


            ## MARK IF SIMULATION IS DONE
            terminated = self.Done = False
            truncated = False
            

            ## CACULATE REWARD
            reward = 0
            
            return(
                next_obs,
                reward,
                terminated,
                truncated,
                {},
            )


        ########## POST-POLICY TRIGGER ##########
        elif action[0] >= self.Trg_threshold:

            self.Pol_Trg_Flag = True

            ## GRAB TERMINAL OBS
            terminal_obs = self._get_obs()

            ## EXECUTE REMAINDER OF SIMULATION
            # self.Finish_Sim()

            ## MARK IF SIMULATION IS DONE
            terminated = self.Done = True
            truncated = False

            ## CALCULATE REWARD
            reward = self.Calc_Reward()  

            return(
                terminal_obs,
                reward,
                terminated,
                truncated,
                {},
            )


        return obs, reward, terminated, truncated, {}

    
    
    def Calc_Reward(self):

        return 0
    

    
    def _get_obs(self):
    
        return np.array([0,0,0,0],dtype=np.float32)
    

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


        # ## GET CURRENT STATE
        # x,z,phi,vx,vz,dphi = self._get_state()

        # ## GET CURRENT PARAMS
        # (L,gamma,M_B,I_B,PD) = self._get_params()
        
        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE)

        ## ORIGIN AXES
        pygame.draw.line(self.surf,GREEN,c2p((0,0)),c2p((0.1,0)),width=5) # X_w   
        pygame.draw.line(self.surf,BLUE, c2p((0,0)),c2p((0,0.1)),width=5) # Z_w   
        pygame.draw.circle(self.surf,RED,c2p((0,0)),radius=4,width=0)


        ## LANDING SURFACE
        # pygame.draw.line(self.surf,GREY,
        #                  c2p(self.Plane_Pos + self._P_to_W(np.array([-2,0]),self.Plane_Angle,deg=True)),
        #                  c2p(self.Plane_Pos + self._P_to_W(np.array([+2,0]),self.Plane_Angle,deg=True)),width=2)
        
        # pygame.draw.line(self.surf,BLACK,
        #                  c2p(self.Plane_Pos + self._P_to_W(np.array([-0.5,0]),self.Plane_Angle,deg=True)),
        #                  c2p(self.Plane_Pos + self._P_to_W(np.array([+0.5,0]),self.Plane_Angle,deg=True)),width=5)
    
        # ## LANDING SURFACE AXES
        # pygame.draw.line(self.surf,GREEN,c2p(self.Plane_Pos),c2p(self.Plane_Pos + self._P_to_W(np.array([0.1,0]),self.Plane_Angle,deg=True)),width=7)  # t_x   
        # pygame.draw.line(self.surf,BLUE, c2p(self.Plane_Pos),c2p(self.Plane_Pos + self._P_to_W(np.array([0,0.1]),self.Plane_Angle,deg=True)),width=7)  # n_p 
        # pygame.draw.circle(self.surf,RED,c2p(self.Plane_Pos),radius=4,width=0)


        ## DRAW QUADROTOR
        # Pose = self._get_pose(x,z,phi)
        # pygame.draw.line(self.surf,RED,c2p(Pose[0]),c2p(Pose[1]),width=3) # Leg 1
        # pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[2]),width=3) # Leg 2
        # pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[3]),width=3) # Prop 1
        # pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[4]),width=3) # Prop 2
        # pygame.draw.circle(self.surf,GREY,c2p(Pose[0]),radius=self.collision_radius*self.screen_width/self.world_width,width=2)


        ## BODY AXES
        # pygame.draw.line(self.surf,GREEN,c2p(Pose[0]),c2p(Pose[0] + self._B_to_W(np.array([0.05,0]),phi)),width=5)  # B_x   
        # pygame.draw.line(self.surf,BLUE,c2p(Pose[0]),c2p(Pose[0] + self._B_to_W(np.array([0,0.05]),phi)),width=5)  # B_z  



        ## TRIGGER INDICATOR
        # if self.Pol_Trg_Flag == True:
        #     pygame.draw.circle(self.surf,RED,  c2p(Pose[0]),radius=4,width=0)
        #     pygame.draw.circle(self.surf,BLACK,c2p(Pose[0]),radius=5,width=3)
        # else:
        #     pygame.draw.circle(self.surf,BLUE, c2p(Pose[0]),radius=4,width=0)
        #     pygame.draw.circle(self.surf,BLACK,c2p(Pose[0]),radius=5,width=3)


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
        # text_Tau = my_font.render(f'Tau: {self._get_obs()[0]:2.2f} [s]', True, BLACK)
        # text_Theta_x = my_font.render(f'Theta_x: {self._get_obs()[1]:2.2f} [rad/s]', True, BLACK)
        # text_D_perp = my_font.render(f'D_perp: {self._get_obs()[2]:2.2f} [m]', True, BLACK)
        # text_Plane_Angle = my_font.render(f'Plane Angle: {self.Plane_Angle:3.1f} [deg]', True, BLACK)

        ## ACTIONS TEXT
        text_Actions = my_font.render(f'Actions:', True, GREY)
        # text_Trg_Action = my_font.render(f'Trg_Action: {5.0:3.1f}', True, BLACK)
        # text_Rot_Action = my_font.render(f'Rot_Action: {5.0:3.1f}', True, BLACK)

        ## REWARD TEXT
        text_Other = my_font.render(f'Other:', True, GREY)
        # text_reward = my_font.render(f'Prev Reward: {self.reward:.3f}',True, BLACK)
        # text_Tau_trg = my_font.render(f'Tau trg: {self.Tau_trg:.3f} [s]',True, BLACK)


        ## DRAW OBJECTS TO SCREEN
        self.screen.blit(self.surf,(0,0))
        self.screen.blit(text_States,       (5,5))
        self.screen.blit(text_t_step,       (5,5 + 25*1))
        # self.screen.blit(text_Phi_rel,      (5,5 + 25*2))
        # self.screen.blit(text_V_mag,        (5,5 + 25*3))

        self.screen.blit(text_Obs,          (5,5 + 25*5))
        # self.screen.blit(text_Tau,          (5,5 + 25*6))
        # self.screen.blit(text_Theta_x,      (5,5 + 25*7))
        # self.screen.blit(text_D_perp,       (5,5 + 25*8))
        # self.screen.blit(text_Plane_Angle,  (5,5 + 25*9))

        self.screen.blit(text_Actions,      (5,5 + 25*11))
        # self.screen.blit(text_Rot_Action,   (5,5 + 25*12))
        # self.screen.blit(text_Trg_Action,   (5,5 + 25*13))

        self.screen.blit(text_Other,        (5,5 + 25*15))
        # self.screen.blit(text_reward,       (5,5 + 25*16))
        # self.screen.blit(text_Tau_trg,      (5,5 + 25*17))



        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(120) # [Hz]
        pygame.display.flip()

    def close(self):

        return
    

if __name__ == '__main__':
    env = SAR_Env_2D()
    env.RENDER = True

    for ep in range(25):

        obs,_ = env.reset(V_mag=None,Rel_Angle=None)

        Done = False
        while not Done:

            action = env.action_space.sample()
            # action = np.zeros_like(action)
            obs,reward,Done,truncated,_ = env.step(action)

        print(f"Episode: {ep} \t Reward: {reward:.3f}")

    env.close()



                