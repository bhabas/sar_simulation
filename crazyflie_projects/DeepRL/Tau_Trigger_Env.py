import gym
from gym import logger,spaces
from stable_baselines3.common.env_checker import check_env
import math

import numpy as np


class Tau_Trigger_Env():
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super(Tau_Trigger_Env, self).__init__()
        self.env_name = "Tau_Trigger_Discrete"

        ## PHYSICS PARAMETERS
        self.dt = 0.02  # seconds between state updates
        self.masscart = 1.0
        self.t_step = 0

        ## POLICY PARAMETERS
        self.x_d = 1.0
        self.Once_flag = False
        self.state = None
        self.tau_thr = 0

        ## ENV LIMITS
        self.x_threshold = 2.4
        self.t_threshold = 250

        high = np.array(
            [
                self.x_threshold * 2,
                np.finfo(np.float32).max,
            ],
            dtype=np.float32,
        )

        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        ## RENDERING PARAMETERS
        self.screen_width = 600
        self.screen_height = 400
        self.screen = None
        self.clock = None
        self.isopen = True


    def step(self, action):
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg
        assert self.state is not None, "Call reset before using step method."
        self.t_step += 1
        x,x_dot = self.state


        ## IF ACTION TRIGGERED THEN KEEP THESE DYNAMICS
        if action == 1 or self.Once_flag == True:
            self.Once_flag = True
            C_drag = 2.0
            x_acc = (-C_drag*x_dot)/self.masscart
            self.tau_thr = self.obs[0]
        else:
            x_acc = 0
        
        ## UPDATE STATE
        x = x + self.dt*x_dot
        x_dot = x_dot + self.dt*x_acc
        self.state = (x, x_dot)

        ## UPDATE OBSERVATION
        d_ceil = self.x_d - x
        tau = d_ceil/x_dot
        self.obs = (tau,d_ceil)

        ## CHECK FOR DONE
        done = bool(
            x > self.x_threshold
            or self.t_step >= self.t_threshold
            or x_dot <= 0.05
        )

        ## CALCULATE REWARD
        if not done:
            reward = 0
        else:
            reward = np.clip(2/np.abs(self.x_d-x+1e-3),0,40)


        return np.array(self.obs,dtype=np.float32), reward, done, {}

    def reset(self):
        ## RESET PHYSICS PARAMS
        self.t_step = 0
        self.Once_flag = False
        
        ## RESET STATE
        tau_0 = 1.5
        vel_0 = np.random.uniform(low=0.5,high=2.5)
        d_ceil_0 = tau_0*vel_0
        pos_0 = self.x_d - d_ceil_0
        self.state = (pos_0,vel_0)

        ## RESET OBSERVATION
        self.obs = (tau_0,d_ceil_0)

        return np.array(self.obs,dtype=np.float32)

    def render(self):
        import pygame
        from pygame import gfxdraw

        ## DEFINE COLORS
        white = (255,255,255)
        black = (0,0,0)
        blue = (29,123,243)
        red = (255,0,0)

        ## INITIATE SCREEN AND CLOCK ON FIRST LOADING
        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pygame.time.Clock()

        if self.state is None:
            return None
        x = self.state

        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(white)

        ## CREATE TIMESTEP LABEL
        my_font = pygame.font.SysFont(None, 30)
        text_surface = my_font.render(f'Time Step: {self.t_step:03d}', True, black)

        world_width = self.x_threshold * 2
        scale = self.screen_width / world_width

        

        ## CREATE CART
        cartwidth = 50.0
        cartheight = 30.0

        l, r, t, b = -cartwidth / 2, cartwidth / 2, cartheight / 2, -cartheight / 2
        cartx = x[0] * scale + self.screen_width / 2.0  # MIDDLE OF CART
        carty = 100  # TOP OF CART

        gfxdraw.hline(self.surf, 0, self.screen_width, carty,black)
        gfxdraw.vline(self.surf, int(self.x_d*scale + self.screen_width / 2.0), 0, self.screen_height,black)

        ## CREATE CART COORDS AND TRANSLATE TO PROPER LOCATION
        cart_coords = [(l, b), (l, t), (r, t), (r, b)]
        cart_coords = [(c[0] + cartx, c[1] + carty) for c in cart_coords]
        gfxdraw.filled_polygon(self.surf, cart_coords, black)

        if self.Once_flag == True:
            gfxdraw.aacircle(self.surf,int(cartx),int(carty),5,red)
            gfxdraw.filled_circle(self.surf,int(cartx),int(carty),5,red)
        else:
            gfxdraw.aacircle(self.surf,int(cartx),int(carty),5,blue)
            gfxdraw.filled_circle(self.surf,int(cartx),int(carty),5,blue)



        
        

        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)
        self.screen.blit(self.surf, (0, 0))
        self.screen.blit(text_surface, (5,5))



        self.clock.tick(45)
        pygame.display.flip()

        
    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False


if __name__ == '__main__':

    env = Tau_Trigger_Env()
    for _ in range(5):
        env.reset()
        done = False
        while not done:
            env.render()
            obs,reward,done,info = env.step(env.action_space.sample())


    env.close()
