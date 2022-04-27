import numpy as np
import matplotlib.pyplot as plt
import gym
import random

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim


env = gym.make("CartPole-v1")
observation_space = env.observation_space.shape[0]
action_space = env.action_space.n

EPISODES = 100
LR = 0.0001
MEM_SIZE = 10_000
BATCH_SIZE = 64
GAMMA = 0.95

epsilon_max = 1.0
epsilon_decay = 0.999
epsilon_min = 0.001

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
        self.epsilon = epsilon_max
        self.network = Network(input_size=4,output_size=2)

    def choose_action(self,state):

        if random.random() < self.epsilon:
            return env.action_space.sample()

        state = torch.tensor(state,dtype=torch.float)
        state = state.to(DEVICE)

        q_vals = self.network.forward(state)

        return torch.argmax(q_vals).item()

    def learn(self):
        pass


if __name__ == '__main__':

    

    agent = DQN_Agent()

    for i in range(1,EPISODES):
        state = env.reset()
        state = state.reshape(1,-1)
        score = 0

        while True:
            env.render()
            action = agent.choose_action(state)
            next_state,reward,done,_ = env.step(action)
            next_state = next_state.reshape(1,-1)

            # agent.memory.add(state,action,reward,next_state,done)
            agent.learn()
            state = next_state
            score += reward

            if done:

                if score > best_reward:
                    best_reward = score

                average_reward += score
                print(f"Episode: {i} | Average Reward: {average_reward/i:.3f} | Best Reward: {best_reward} | Last Reward: {score} | Epsilon: {agent.epsilon:.3f}")
                break

            episode_number.append(i)
            average_reward_number.append(average_reward/i)


    plt.plot(episode_number, average_reward_number)
    plt.show()