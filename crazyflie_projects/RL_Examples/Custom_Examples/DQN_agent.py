import gym
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os
import torch
import random
from collections import deque
import pandas as pd

from DQN_model import Linear_QNet, QTrainer
from helper import plot

MAX_MEMORY = 100_000 
BATCH_SIZE = 1_000
LR = 0.1
DISCOUNT_RATE = 0.99
EPISODES = 10_000
T = 20


class Agent:

    def __init__(self):
        self.n_games = 0
        self.epsilon = 0.5
        self.gamma = 0.9
        self.memory = deque(maxlen=MAX_MEMORY)
        
        self.Q_Net = Linear_QNet(4,256,2)
        self.trainer = QTrainer(self.Q_Net,lr=LR,gamma=self.gamma)

    def get_action(self,state):

        if np.random.random() >= self.epsilon:
            state = torch.tensor(state,dtype=torch.float)
            Q_prediction = self.Q_Net.forward(state)
            action = torch.argmax(Q_prediction).item()

        else:
            action = np.random.randint(0,env.action_space.n)     


        return action

    def train_short_memory(self):
        
        df_train = pd.DataFrame(random.sample(self.memory,50),columns=['state','action','reward','next_state','done'])
        self.trainer.train_step(df_train)




if __name__ == '__main__':
    
    plot_scores = []
    plot_mean_scores = []
    episode_reward = 0
    record = 0
    agent = Agent()
    env = gym.make("CartPole-v1")

    

    for episode in range(EPISODES):

        ## INITIALIZE EPISODE
        episode_reward = 0
        state = env.reset()
        done = False
        t_step = 0

        while not done:

            ## SELECT ACTION FROM CURRENT STATE
            action = agent.get_action(state)

            ## PERFORM ACTION AND GET NEW STATE
            next_state,reward,done,_ = env.step(action)

            if episode%100==0:
                env.render()
            
            if episode >= 1:
                
                ## UPDATE Q-NETWORK TO TARGET NETWORK
                if t_step%T == 0:
                    agent.trainer.copy_TargetNN()

                ## TRAIN Q-NETWORK
                agent.train_short_memory()

            ## STORE SAMPLE OF TRAINING DATA
            agent.memory.append((state,action,reward,next_state,done))

            episode_reward += reward

            state = next_state
            t_step += 1

        if episode_reward > record:
            record = episode_reward

        mean_score = np.average(episode_reward)

        plot_scores.append(episode_reward)
        plot_mean_scores.append(mean_score)
        plot(plot_scores,plot_mean_scores)

        print(f"Game: {episode} \t Score: {episode_reward} \t Record: {record}")
        
        

        