import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import os
import copy

BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/RL_Examples/Custom_Examples"
class Linear_QNet(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super().__init__()
        self.linear1 = nn.Linear(input_size, hidden_size)
        self.linear2 = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        x = F.relu(self.linear1(x))
        x = self.linear2(x)
        return x

    # def save(self, file_name='model.pth'):
    #     model_folder_path = f'{BASEPATH}/model'
    #     if not os.path.exists(model_folder_path):
    #         os.makedirs(model_folder_path)

    #     file_name = os.path.join(model_folder_path, file_name)
    #     torch.save(self.state_dict(), file_name)


class QTrainer:
    def __init__(self,Q_NN,lr,gamma):
        self.lr = lr
        self.gamma = gamma

        self.Q_NN = Q_NN
        self.Q_Target_NN = copy.deepcopy(self.Q_NN) # Create target network as copy of Q_NN

        self.optimizer = optim.Adam(Q_NN.parameters(), lr=self.lr)
        self.criterion = nn.MSELoss()

    def train_step(self,df_train):

        state = torch.tensor(df_train['state'], dtype=torch.float)
        next_state = torch.tensor(df_train['next_state'], dtype=torch.float)
        action = torch.tensor(df_train['action'], dtype=torch.long)
        reward = torch.tensor(df_train['reward'], dtype=torch.float)
        done = df_train['done']


         

        ## PREDICT Q-VALUES FROM CURRENT DATA BATCH
        Q_prediction = self.Q_NN.forward(state)             
        Q_target = self.Q_Target_NN.forward(next_state)     
        for idx in range(len(done)):

            Q_prediction[idx][0] = Q_prediction[idx][action[idx]] # Predict Q-value for current (states,action)

            if not done[idx]:
                Q_target[idx][0] = reward[idx] + self.gamma*torch.max(Q_target[idx]).item() # Predict Q-value for next state
            else:
                Q_target[idx][0] = reward[idx]

        Q_prediction = Q_prediction[:,0].reshape(1,-1)
        Q_target = Q_target[:,0].reshape(1,-1)

        self.optimizer.zero_grad()
        loss = self.criterion(Q_prediction, Q_target)
        loss.backward()

        self.optimizer.step()



        

