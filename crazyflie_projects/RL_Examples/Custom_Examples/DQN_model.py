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

    def train_step(self,state,action,reward,next_state,done):

        state = torch.tensor(state, dtype=torch.float)
        next_state = torch.tensor(next_state, dtype=torch.float)
        action = torch.tensor(action, dtype=torch.long)
        reward = torch.tensor(reward, dtype=torch.float)

        ## IF LENGTH OF TRAINING DATA IS 1 THEN TURN INTO 2-D TENSOR
        if len(state.shape) == 1:
            
            state = torch.unsqueeze(state, 0)
            next_state = torch.unsqueeze(next_state, 0)
            action = torch.unsqueeze(action, 0)
            reward = torch.unsqueeze(reward, 0)
            done = (done, )
            

        ## PREDICT Q-VALUES FROM CURRENT DATA BATCH
        Q_prediction = self.Q_NN.forward(state)         # Predict Q value for current state
        Q_target = self.Q_Target_NN.forward(next_state) # Predict Q-value for next state

        for idx in range(len(done)):
            Q_new = reward[idx]

        

