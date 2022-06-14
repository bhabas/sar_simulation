import gym
from gym import spaces

import numpy as np


class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__()

        self.gravity = 9.8
        self.masscart = 1.0
        self.force_mag = 10.0
        self.tau = 0.02  # seconds between state updates

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

    # def step(self, action):
    #     ...
    #     return observation, reward, done, info
    def reset(self):
        self.state = self.np_random.uniform(low=-0.05,high=0.05,size=(2,))
        return np.array(self.state,dtype=np.float32)

    def render(self):
        import pygame
        from pygame import gfxdraw

        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pygame.time.Clock()

        world_width = self.x_threshold * 2
        scale = self.screen_width / world_width
        cart_width = 50.0
        cart_height = 30.0

        # if self.state is None:
        #     return None

        x = self.state
        
        self.surf = pygame.Surface((self.screen_width,self.screen_height))
        self.surf.fill((255,255,255))

        self.surf = pygame.transform.flip(self.surf,False,True)
        self.screen.blit(self.surf,(0,0))

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
    env.action_space.sample()
    env.render()
    # env.reset()

    # episodes = 10
    # for ep in range(episodes):
    #     obs = env.reset()
    #     done = False
    #     while not done:
    #         env.render()
    #         obs, reward,done,info, = env.step(env.action_space.sample())

    env.close()
