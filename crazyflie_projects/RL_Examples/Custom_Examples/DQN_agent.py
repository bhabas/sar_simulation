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
        
        self.Q_Net = Linear_QNet(2,50,3)
        self.trainer = QTrainer(self.Q_Net,lr=LR,gamma=self.gamma)

    def get_action(self,state):

        if np.random.random() >= self.epsilon:
            state = torch.tensor(state,dtype=torch.float)
            Q_prediction = self.Q_Net(state)
            action = torch.argmax(Q_prediction).item()

        else:
            action = np.random.randint(0,env.action_space.n)     


        return action

    def train_short_memory(self,state,action,reward,next_state,done):

        self.trainer.train_step(state,action,reward,next_state,done)


env = gym.make("MountainCar-v0")
env.reset()

if __name__ == '__main__':
    
    plot_scores = []
    plot_mean_scores = []
    total_score = 0
    record = 0
    agent = Agent()
    env = gym.make("MountainCar-v0")


    ## INITIALIZE EPISODE
    episode_reward = 0
    state = env.reset()
    done = False

    while not done:

        ## SELECT ACTION FROM CURRENT STATE
        action = agent.get_action(state)

        ## PERFORM ACTION AND GET NEW STATE
        next_state,reward,done,_ = env.step(action)
        
        ## TRAIN SHORT TERM MEMORY
        agent.train_short_memory(state,action,reward,next_state,done)

        ## STORE SAMPLE OF TRAINING DATA
        agent.memory.append((state,action,reward,next_state,done))


        episode_reward += reward


        # STORE IN MEMORY

        state = next_state