import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split

import os


import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

os.system('clear')


## DESIGNATE FUNCTION USED
def f(x1,x2,x3):
    a = 0
    b = 0
    c = 0

    r = np.sqrt((x1-a)**2 + (x2-b)**2 + (x3-c)**2)

    LR = r <= 9.0

    return r.flatten(),LR.flatten()

## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=3,h1=8,h2=8,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h1) # Fully connected layer
        self.fc2 = nn.Linear(h1,h2)
        self.out_r = nn.Linear(h2,1)
        self.out_LR = nn.Linear(h2,1)


    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = torch.tanh(self.fc1(x))
        x = torch.tanh(self.fc2(x))
        x1 = self.out_r(x)
        x2 = self.out_LR(x)

        return x1,x2


## GENERATE TRAINING DATA
batch_size = 15
function_select = 1

x1 = np.linspace(-3.0, 3.0, num=batch_size)
x2 = np.linspace(-3.0, 3.0, num=batch_size)
x3 = np.linspace(-3.0, 3.0, num=batch_size)

X1,X2,X3 = np.meshgrid(x1, x2, x3)
X = np.stack((X1.flatten(),X2.flatten(),X3.flatten()),axis=1)

y_r,y_LR = f(X1,X2,X3)

data_array = np.stack((X1.flatten(),X2.flatten(),X3.flatten(),y_r,y_LR)).T
df = pd.DataFrame(data_array,columns=['X1','X2','X3','r','LR'])

## SPLIT DATA FEATURES INTO TRAINING DATA & TESTING DATA

train, test = train_test_split(df, test_size=0.2)


## CONVERT DATA INTO TENSORS
X_train = torch.FloatTensor(train[['X1','X2','X3']].to_numpy())
X_test = torch.FloatTensor(test[['X1','X2','X3']].to_numpy())

y_train = torch.FloatTensor(train[['r','LR']].to_numpy())
y_test = torch.FloatTensor(test[['r','LR']].to_numpy())



## INITIALIZE NEURAL NETWORK MODEL
torch.manual_seed(32)
model = Model()

## DEFINE OPTIMIZER AND LOSS FUNCTION (CRITERION)
criterion1 = nn.MSELoss(reduction='mean')
criterion2 = nn.MSELoss(reduction='mean')

optimizer = torch.optim.Adam(model.parameters(),lr=0.01) # Model parameters are the layers of model


epochs = 10000
losses = []

for i in range(epochs):

    ## MODEL PREDICTION
    y_pred_r,y_pred_LR = model.forward(X_train)

    ## CALCULATE LOSS/ERROR
    loss1 = criterion1(y_pred_r,y_train[:,0].reshape(-1,1))
    loss2 = criterion2(y_pred_LR,y_train[:,1].reshape(-1,1))
    loss = loss1 + loss2
    losses.append(loss)

    if i%10 == 1:
        print(f"epoch {i} and loss is: {loss}")

    

    ## BACKPROPAGATION
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()



with torch.no_grad():
    y_eval_r,y_eval_LR = model.forward(X_test)



## PLOT NETWORK OUTPUTS
fig = plt.figure(1, figsize=(12,6))

ax1 = fig.add_subplot(2,3,1,projection='3d')
ax1.scatter(X1.flatten()[::5],X2.flatten()[::5],X3.flatten()[::5],c=y_r[::5])
ax1.set_xlabel('x1')
ax1.set_ylabel('x2')
ax1.set_title('Function')

ax2 = fig.add_subplot(2,3,2,projection='3d')
ax2.scatter(X_test[:,0],X_test[:,1],X_test[:,2],c=y_eval_r[:,0])
ax2.set_xlabel('x1')
ax2.set_ylabel('x2')
ax2.set_title('Function')



plt.show()
print()

