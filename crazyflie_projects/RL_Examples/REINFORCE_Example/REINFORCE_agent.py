import numpy as np
import matplotlib.pyplot as plt
import gym
import random
import copy 

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical
torch.manual_seed(0)

from collections import deque

env = gym.make("CartPole-v1")
observation_space = env.observation_space.shape[0]
action_space = env.action_space.n

EPISODES = 300
LR = 0.0001
MEM_SIZE = 10_000
BATCH_SIZE = 64
GAMMA = 0.95


DEVICE = torch.device("cuda")

best_reward = 0
average_reward = 0
episode_number = []
average_reward_number = []


class Policy_NN(nn.Module):
    def __init__(self,input_size=4,output_size=2):
        super().__init__()

        self.fc1 = nn.Linear(input_size, 32)
        self.fc2 = nn.Linear(32, output_size)

        self.optimizer = optim.Adam(self.parameters(), lr=LR)
        self.loss = nn.MSELoss()
        self.to(DEVICE)
    
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.softmax(self.fc2(x),dim=1)

        return x


class Agent():
    def __init__(self):

        self.Policy_NN = Policy_NN(input_size=4,output_size=2)
        self.memory = deque(maxlen=MEM_SIZE)

    def choose_action(self,state):

        ## FIND ACTION PROBABILITIES FOR CURRENT STATE 
        state = torch.tensor(state,dtype=torch.float).to(DEVICE) # Convert state to tensor
        probs = self.Policy_NN.forward(state) # Probability for each action

        ## SAMPLE FROM PROBABILITY DISTRIBUTION
        dist = Categorical(probs)
        action = dist.sample()

        return action.item(),dist.log_prob(action)

    def remember(self,state,action,reward,next_state,done,log_prob):
        self.memory.append((state,action,reward,next_state,done,log_prob))

    def train(self,):

        minibatch = self.memory

        state = np.zeros((len(minibatch),4))
        next_state = np.zeros((len(minibatch),4))
        action,reward,done,log_prob = [],[],[],[]


        for ii in range(len(minibatch)):
            state[ii] = minibatch[ii][0]
            action.append(minibatch[ii][1])
            reward.append(minibatch[ii][2])
            next_state[ii] = minibatch[ii][3]
            done.append(minibatch[ii][4])
            log_prob.append(minibatch[ii][5])

        def discount_rewards(rewards, gamma):
            t_steps = np.arange(len(rewards))
            r = rewards * gamma**t_steps
            r = r[::-1].cumsum()[::-1] / gamma**t_steps
            return r

        R = discount_rewards(reward,GAMMA) # Discounted return
        

        


        states = torch.tensor(state,dtype=torch.float).to(DEVICE)
        actions = torch.tensor(action,dtype=torch.long).to(DEVICE)
        next_states = torch.tensor(next_state,dtype=torch.float).to(DEVICE)
        rewards = torch.tensor(reward,dtype=torch.float).to(DEVICE)
        dones = torch.tensor(done, dtype=torch.bool).to(DEVICE)
        log_probs = torch.tensor(log_prob, dtype=torch.float).to(DEVICE)
        Rs = torch.tensor(R, dtype=torch.float).to(DEVICE)
        batch_indices = np.arange(len(minibatch), dtype=np.int64)


        policy_loss = torch.tensor(-log_probs*Rs,dtype=torch.float,requires_grad=True).sum().to(DEVICE)

        self.Policy_NN.optimizer.zero_grad()
        policy_loss.backward()
        self.Policy_NN.optimizer.step()


if __name__ == '__main__':

    agent = Agent()

    for i in range(1,EPISODES):
        state = env.reset()
        state = state.reshape(1,-1)
        score = 0

        while True:
            # env.render()
            action,log_prob = agent.choose_action(state)
            next_state,reward,done,_ = env.step(action)
            next_state = next_state.reshape(1,-1)
            agent.remember(state,action,reward,next_state,done,log_prob)

            
            score += reward

            if done:

                agent.train()

                if score > best_reward:
                    best_reward = score

                average_reward += score
                print(f"Episode: {i} | Average Reward: {average_reward/i:.3f} | Best Reward: {best_reward} | Last Reward: {score}")

                episode_number.append(i)
                average_reward_number.append(score)

                break

            state = next_state

                


    plt.plot(episode_number, average_reward_number)
    plt.show()