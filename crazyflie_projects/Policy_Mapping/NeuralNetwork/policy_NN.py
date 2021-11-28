from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
import os
import matplotlib as mpl

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

## DEFINE NEURAL NETWORK MODEL
class Model(nn.Module):
    def __init__(self,in_features=3,h1=20,h2=20,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h1)
        self.fc2 = nn.Linear(h1,h2)
        self.out = nn.Linear(h2,out_features)

    def forward(self,x):

        ## PASS DATA THROUGH NETWORK
        x = F.tanh(self.fc1(x))
        x = F.tanh(self.fc2(x))
        x = self.out(x)

        return x


## GENERATE TRAINING DATA
df_raw = pd.read_csv('crazyflie_projects/Policy_Mapping/Data_Analysis/Policy_Mapping_Compiled.csv')

x1 = df_raw.iloc[:]['OFy_trig']
x2 = df_raw.iloc[:]['RREV_trig']
x3 = df_raw.iloc[:]['d_ceiling']

X = np.stack((x1,x2,x3),axis=1)
y_My = df_raw.iloc[:]['My']
y_LR = df_raw.iloc[:]['leg_contacts']


data_array = np.stack((x1,x2,x3,y_LR)).T
df = pd.DataFrame(data_array,columns=['X1','X2','X3','y_LR'])

## SPLIT DATA FEATURES INTO TRAINING DATA & TESTING DATA
train, test = train_test_split(df, test_size=0.2)

## CONVERT DATA INTO TENSORS
X_train = torch.FloatTensor(train[['X1','X2','X3']].to_numpy())
X_test = torch.FloatTensor(test[['X1','X2','X3']].to_numpy())

y_train = torch.FloatTensor(train[['y_LR']].to_numpy())
y_test = torch.FloatTensor(test[['y_LR']].to_numpy())


## INITIALIZE NEURAL NETWORK MODEL
torch.manual_seed(32)
model = Model()

## DEFINE OPTIMIZER AND LOSS FUNCTION (CRITERION)
criterion = nn.MSELoss(reduction='mean')
optimizer = torch.optim.Adam(model.parameters(),lr=0.01) # Model parameters are the layers of model

epochs = 10_000
losses = []

for ii in range(epochs):
    
    ## MODEL PREDICTION
    y_pred = model.forward(X_train)

    ## CALCULATE LOSS/ERROR
    loss = criterion(y_pred,y_train)
    losses.append(loss)

    if ii%10 == 1:
        print(f"epoch {ii} and loss is: {loss}")


    ## BACKPROPAGATION
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

# # ## PLOT LOSS VS EPOCHS
# plt.plot(range(epochs),losses)
# plt.ylabel('Loss')
# plt.xlabel('Epoch')

# plt.show()

## PLOT NETWORK OUTPUTS
# fig = plt.figure(1,figsize=(12,6))


# cmap = mpl.cm.jet
# norm = mpl.colors.Normalize(vmin=0,vmax=4.0)

# ax1 = fig.add_subplot(1,2,1,projection='3d')
# ax1.scatter(x1,x2,x3,c=y_LR,cmap=cmap,norm=norm)
# ax1.set_xlabel('OFy_tr')
# ax1.set_ylabel('RREV_tr')
# ax1.set_zlabel('d_ceiling_tr')

# # CREATE PLOTS AND COLOR BAR
# # ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
# # ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.25,depthshade=False)

# plt.show()