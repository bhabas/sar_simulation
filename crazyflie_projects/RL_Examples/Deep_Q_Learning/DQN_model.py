import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import os
import copy

class Linear_QNet(nn.Module):
    def __init__(self, input_size, output_size):
        super().__init__()
        self.linear1 = nn.Linear(input_size, 512)
        self.linear2 = nn.Linear(512,256)
        self.linear3 = nn.Linear(256,64)
        self.linear4 = nn.Linear(64, output_size)

    def forward(self, x):
        x = F.relu(self.linear1(x))
        x = F.relu(self.linear2(x))
        x = F.relu(self.linear3(x))
        x = self.linear4(x)
        return x