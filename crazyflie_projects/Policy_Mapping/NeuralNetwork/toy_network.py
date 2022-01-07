import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split

BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"


## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=3,h=4,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h) # Fully connected layer
        # self.fc2 = nn.Linear(h,h)
        self.out = nn.Linear(h,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.sigmoid(self.fc1(x))
        # x = F.sigmoid(self.fc2(x))
        x = F.sigmoid(self.out(x))


        return x




if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    torch.manual_seed(22)
    np.random.seed(0)

    
    ## CONVERT DATA INTO TENSORS
    X = torch.FloatTensor([1,2,3]).reshape((1,-1))
    

    model = Model()

    with torch.no_grad():
        y = model.forward(X)
        ii = 0
        f = open('NN_Layers.data','ab')
        f.truncate(0) ## Clears contents of file

        for name, layer in model.named_modules():
            if ii > 0:
                # print(layer.weight.numpy())]

                W = layer.weight.numpy()
                np.savetxt(f,W,
                    fmt='%.6f',
                    delimiter='\t',
                    comments='',
                    header=f"{W.shape[0]} {W.shape[1]}")

                b = layer.bias.numpy().reshape(-1,1)
                np.savetxt(f,b,
                    fmt='%.6f',
                    delimiter='\t',
                    comments='',
                    header=f"{b.shape[0]} {b.shape[1]}")

            ii+=1

        f.close()

        print(y)

