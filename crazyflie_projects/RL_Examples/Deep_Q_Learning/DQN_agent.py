import gym
import random
import numpy as np
import matplotlib.pyplot as plt
from collections import deque


import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F


from DQN_model import Linear_QNet


class DQN_Agent:
    def __init__(self):
        self.env = gym.make("CartPole-v1")

        self.state_size = self.env.observation_space.shape[0]
        self.action_size = self.env.action_space.n
        self.EPISODES = 1000
        self.memory = deque(maxlen=2000)

        self.gamma = 0.95
        self.lr = 0.00025
        self.epsilon = 1.0
        self.epsilon_min = 0.001
        self.epsilon_decay = 0.999
        self.batch_size = 64
        self.train_start = 64

        self.model = Linear_QNet(self.state_size,self.action_size)

        self.optimizer = optim.Adam(self.model.parameters(), lr=self.lr)
        self.criterion = nn.MSELoss()

    def remember(self,state,action,reward,next_state,done):
        self.memory.append((state,action,reward,next_state,done))

        ## ONLY TRAIN WITH ENOUGH MEMORY
        if len(self.memory) > self.train_start:

            ## EPSILON DECAY
            if self.epsilon >self.epsilon_min:
                self.epsilon *= self.epsilon_decay
    
    def act(self, state):
        if np.random.random() <= self.epsilon:
            action = np.random.randint(0,self.env.action_space.n)  
             
        else:
            state = torch.tensor(state,dtype=torch.float)
            Q_prediction = self.model.forward(state)
            action = torch.argmax(Q_prediction).item()

        return action



    def replay(self):
        if len(self.memory) < self.train_start:
            return 
        
        ## RANDOMLY SAMPLE MINIBATCH FROM MEMORY
        minibatch = random.sample(self.memory, min(len(self.memory),self.batch_size))

        state = np.zeros((self.batch_size,self.state_size))
        next_state = np.zeros((self.batch_size,self.state_size))
        action,reward,done = [],[],[]

        # Do this before prediction
        # for speedup, this could be done on the tensor level
        # but easier to understand using a loop
        for ii in range(self.batch_size):
            state[ii] = minibatch[ii][0]
            action.append(minibatch[ii][1])
            reward.append(minibatch[ii][2])
            next_state[ii] = minibatch[ii][3]
            done.append(minibatch[ii][4])

        state = torch.tensor(state,dtype=torch.float)
        action = torch.tensor(action,dtype=torch.long)
        next_state = torch.tensor(next_state,dtype=torch.float)
        reward = torch.tensor(reward,dtype=torch.float)

        ## DO BATCH PREDICTION TO SAVE SPEED
        target = self.model.forward(state)
        target_next = self.model.forward(next_state)

        for ii in range(self.batch_size):

            if done[ii]:
                target[ii][action[ii]] = reward[ii] 
            
            else:
                ## STANDARD - DQN
                # DQN chooses the max Q value among next actions
                # selection and evaluation of action is on the target Q Network
                # Q_max = max_a' Q_target(s', a')
                target[ii][action[ii]] = reward[ii] + self.gamma*(torch.max(target_next[ii]).item())

        ## TRAIN NN
        self.optimizer.zero_grad()
        loss = self.criterion(target_next, target)
        loss.backward()

        self.optimizer.step()


    def run(self):
        for episode in range(self.EPISODES):
            state = self.env.reset()
            state = state.reshape(1,-1)
            done = False
            i = 0

            while not done:
                self.env.render()
                action = self.act(state)
                next_state, reward,done,_ = self.env.step(action)
                next_state = next_state.reshape(1,-1)

                if not done or i == self.env._max_episode_steps-1:
                    reward = reward
                else:
                    reward = -100
                
                self.remember(state,action,reward,next_state,done)
                state = next_state
                i += 1

                # if done:
                    # print(f"Episode: {episode} \t Score: {i} \t epsilon: {self.epsilon:.3f}")

                self.replay()



if __name__ == '__main__':
    agent = DQN_Agent()
    agent.run()
