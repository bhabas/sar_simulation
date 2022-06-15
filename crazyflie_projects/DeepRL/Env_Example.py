import gym
from gym import logger,spaces
from stable_baselines3.common.env_checker import check_env
import math

import numpy as np


class CustomEnv():
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__()

        self.gravity = 9.8
        self.masscart = 1.0
        self.force_mag = 10.0
        self.tau = 0.02  # seconds between state updates
        self.t_step = 0

        # Angle at which to fail the episode
        self.x_threshold = 2.4

        # Angle limit set to 2 * theta_threshold_radians so failing observation
        # is still within bounds.
        high = np.array(
            [
                self.x_threshold * 2,
                np.finfo(np.float32).max,
            ],
            dtype=np.float32,
        )

        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.screen_width = 600
        self.screen_height = 400
        self.screen = None
        self.clock = None
        self.isopen = True
        self.state = None

        self.steps_beyond_done = None

    def step(self, action):
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg
        assert self.state is not None, "Call reset before using step method."
        self.t_step += 1
        x,x_dot = self.state

        if action == 1:
            force = self.force_mag
        else:
            force = -self.force_mag

        x_acc = force/self.masscart
        x = x + self.tau*x_dot
        x_dot = x_dot + self.tau*x_acc

        self.state = (x, x_dot)


        done = bool(
            x < -self.x_threshold
            or x > self.x_threshold
            or self.t_step >= 250
        )

        if not done:
            reward = np.clip(1/np.abs(x-1),0,100)
        elif self.steps_beyond_done is None:
            self.steps_beyond_done = 0
            reward = np.clip(1/np.abs(x-1),0,100)
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "environment has already returned done = True. You "
                    "should always call 'reset()' once you receive 'done = "
                    "True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_done += 1
            reward = 0.0


        return np.array(self.state,dtype=np.float32), reward, done, {}

    def reset(self):
        self.t_step = 0
        self.state = np.random.uniform(low=-0.1,high=0.1,size=(2,))
        self.state = [-1,0]

        return np.array(self.state,dtype=np.float32)

    def render(self):
        import pygame
        from pygame import gfxdraw

        white = (255,255,255)
        black = (0,0,0)

        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pygame.time.Clock()

        
        my_font = pygame.font.SysFont(None, 30)
        text_surface = my_font.render(f'Time Step: {self.t_step:03d}', True, black)

        world_width = self.x_threshold * 2
        scale = self.screen_width / world_width
        cartwidth = 50.0
        cartheight = 30.0

        if self.state is None:
            return None

        x = self.state
        
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(white)

        l, r, t, b = -cartwidth / 2, cartwidth / 2, cartheight / 2, -cartheight / 2
        axleoffset = cartheight / 4.0
        cartx = x[0] * scale + self.screen_width / 2.0  # MIDDLE OF CART
        carty = 100  # TOP OF CART
        cart_coords = [(l, b), (l, t), (r, t), (r, b)]
        cart_coords = [(c[0] + cartx, c[1] + carty) for c in cart_coords]
        gfxdraw.aapolygon(self.surf, cart_coords, black)
        gfxdraw.filled_polygon(self.surf, cart_coords, black)

       
        gfxdraw.hline(self.surf, 0, self.screen_width, carty,black)

        self.surf = pygame.transform.flip(self.surf, False, True)
        self.screen.blit(self.surf, (0, 0))
        self.screen.blit(text_surface, (5,5))


        pygame.event.pump()
        self.clock.tick(50)
        pygame.display.flip()

        
    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False


if __name__ == '__main__':

    env = CustomEnv()
    env.reset()

    done = False
    while not done:
        env.render()
        obs,reward,done,info = env.step(env.action_space.sample())


    env.close()
