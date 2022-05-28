import numpy as np
import matplotlib.pyplot as plt
import gym
import random
import copy 

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

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


class Network(nn.Module):
    def __init__(self,input_size,output_size):
        super().__init__()
        self.input_shape = env.observation_space.shape
        self.action_space = action_space

        self.fc1 = nn.Linear(input_size, 1024)
        self.fc2 = nn.Linear(1024, 512)
        self.fc3 = nn.Linear(512, output_size)

        self.optimizer = optim.Adam(self.parameters(), lr=LR)
        self.loss = nn.MSELoss()
        self.to(DEVICE)
    
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)

        return x

class DQN_Agent():
    def __init__(self):

        self.epsilon_max = 1.0
        self.epsilon_decay = 0.999
        self.epsilon_min = 0.001
        self.epsilon = self.epsilon_max

        self.Q_NN = Network(input_size=4,output_size=2)
        self.Q_target_NN = copy.deepcopy(self.Q_NN)
        self.memory = deque(maxlen=MEM_SIZE)

    def choose_action(self,state):

        if random.random() < self.epsilon:
            return env.action_space.sample()

        state = torch.tensor(state,dtype=torch.float)
        state = state.to(DEVICE)

        q_vals = self.Q_NN.forward(state)

        return torch.argmax(q_vals).item()

    def remember(self,state,action,reward,next_state,done):
        self.memory.append((state,action,reward,next_state,done))

    def update_Q_target(self):
        self.Q_target_NN = copy.deepcopy(self.Q_NN)

    def train(self):

        if len(self.memory) < BATCH_SIZE:
            return

        minibatch = random.sample(self.memory, min(len(self.memory),BATCH_SIZE))

        state = np.zeros((BATCH_SIZE,4))
        next_state = np.zeros((BATCH_SIZE,4))
        action,reward,done = [],[],[]

        # Do this before prediction
        # for speedup, this could be done on the tensor level
        # but easier to understand using a loop
        for ii in range(BATCH_SIZE):
            state[ii] = minibatch[ii][0]
            action.append(minibatch[ii][1])
            reward.append(minibatch[ii][2])
            next_state[ii] = minibatch[ii][3]
            done.append(minibatch[ii][4])

        states = torch.tensor(state,dtype=torch.float).to(DEVICE)
        actions = torch.tensor(action,dtype=torch.long).to(DEVICE)
        next_states = torch.tensor(next_state,dtype=torch.float).to(DEVICE)
        rewards = torch.tensor(reward,dtype=torch.float).to(DEVICE)
        dones = torch.tensor(done, dtype=torch.bool).to(DEVICE)
        batch_indices = np.arange(BATCH_SIZE, dtype=np.int64)


        Q_vals = self.Q_NN(states)
        Q_vals_pred = Q_vals[batch_indices, actions]


        Target_vals = self.Q_target_NN(next_states)
        Target_vals_pred = torch.max(Target_vals, dim=1)[0]
        
        Q_target = rewards + GAMMA * Target_vals_pred * torch.logical_not(dones)

        loss = self.Q_NN.loss(Q_target, Q_vals_pred)
        self.Q_NN.optimizer.zero_grad()
        loss.backward()
        self.Q_NN.optimizer.step()


        ## EPSILON DECAY
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay




if __name__ == '__main__':

    

    agent = DQN_Agent()

    for i in range(1,EPISODES):
        state = env.reset()
        state = state.reshape(1,-1)
        score = 0

        while True:
            # env.render()
            action = agent.choose_action(state)
            next_state,reward,done,_ = env.step(action)
            next_state = next_state.reshape(1,-1)
            agent.remember(state,action,reward,next_state,done)

            agent.train()
            state = next_state
            score += reward

            if done:

                if score > best_reward:
                    best_reward = score

                average_reward += score
                print(f"Episode: {i} | Average Reward: {average_reward/i:.3f} | Best Reward: {best_reward} | Last Reward: {score} | Epsilon: {agent.epsilon:.3f}")
                agent.update_Q_target()

                episode_number.append(i)
                average_reward_number.append(score)

                break

                


    plt.plot(episode_number, average_reward_number)
    plt.show()