import gym
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os

import torch
import random
from collections import deque

from DQN_model import Linear_QNet, QTrainer

MAX_MEMORY = 100_000 
BATCH_SIZE = 1_000
LR = 0.1
DISCOUNT_RATE = 0.99
EPISODES = 10_000


class Agent:

    def __init__(self):
        self.n_games = 0
        self.epsilon = 0.5
        self.gamma = 0.9
        self.memory = deque(maxlen=MAX_MEMORY)
        self.model = Linear_QNet(2,50,3)
        self.trainer = QTrainer(self.model,lr=LR,gamma=self.gamma)

env = gym.make("MountainCar-v0")
env.reset()

if __name__ == '__main__':
    
    plot_scores = []
    plot_mean_scores = []
    total_score = 0
    record = 0
    agent = Agent()
    env = gym.make("MountainCar-v0")
    env.reset()
