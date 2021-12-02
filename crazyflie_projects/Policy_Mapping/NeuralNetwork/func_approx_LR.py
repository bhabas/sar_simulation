import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl

import os
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split                                                               

BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"
DATAPATH = "crazyflie_projects/ICRA_DataAnalysis/Wide-Long_2-Policy"

os.system("clear")


## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=3,h1=10,h2=10,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h1) # Fully connected layer
        self.fc2 = nn.Linear(h1,h2)
        self.out = nn.Linear(h2,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = torch.tanh(self.fc1(x))
        x = torch.tanh(self.fc2(x))
        x = self.out(x)

        return x

def train_model(epochs,X_train,y_train):

    ## INITIALIZE NEURAL NETWORK MODEL
    torch.manual_seed(22)
    model = Model()

    criterion = nn.MSELoss(reduction='mean')

    optimizer = torch.optim.Adam(model.parameters(),lr=0.01) #  Model parameters are the layers of model
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

    torch.save(model,f'{BASEPATH}/Func_approx_LR.pt')

    plt.plot(range(epochs),losses)
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.show()

    return model


if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    batch_size = 50
    epochs = 10_000

    df = pd.read_csv(f"{DATAPATH}/Wide-Long_2-Policy_Summary.csv")
    df['flip_height_mean'] = 2.1 - df['flip_height_mean']
    # df = df.query('landing_rate_4_leg >= 0.6')

    
    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train, test = train_test_split(df,test_size=0.2,random_state=33)

    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train[['RREV_flip_mean','OF_y_flip_mean','flip_height_mean']].to_numpy())
    X_test = torch.FloatTensor(test[['RREV_flip_mean','OF_y_flip_mean','flip_height_mean']].to_numpy())

    y_train = torch.FloatTensor(train[['landing_rate_4_leg']].to_numpy())
    y_test = torch.FloatTensor(test[['landing_rate_4_leg']].to_numpy())








    # train_model(epochs,X_train,y_train)
    model = torch.load(f'{BASEPATH}/Func_approx_LR.pt')

    ## MODEL EVALUATION

    with torch.no_grad():
        y_eval = model.forward(X_test)

    for ii in range(len(y_eval)):
        # print(f"LR_test: {y_test[ii,1].numpy():.1f} \t LR_eval: {y_eval[ii,1].numpy():.1f}")
        print(f"LR_test: {y_test[ii,0].numpy():.1f} \t LR_eval: {y_eval[ii,0].numpy():.1f} \t%Error: {((y_test[ii,0]-y_eval[ii,0])/y_test[ii,0]).numpy():.2f}")

    ## PLOT REGIONS
    fig = plt.figure(1,figsize=(10,5))
    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=0,vmax=1.0)


    ## SUBPLOT 1 (FUNCTION)
    ax1 = fig.add_subplot(1,2,1,projection='3d')
    ax1.scatter(
        df['OF_y_flip_mean'],
        df['RREV_flip_mean'],
        df['flip_height_mean'],
        c = df['landing_rate_4_leg'],
        cmap=cmap,norm=norm,alpha=0.8,depthshade=False
    )

    ax1.set_xlabel('OF_y')
    ax1.set_xlim(-20,0)
    ax1.set_xticks([-20,-15,-10,-5,0])


    ax1.set_ylabel('RREV')
    ax1.set_ylim(0,8)
    ax1.set_yticks([0,2,4,6,8])

    ax1.set_zlabel('d_ceiling')
    ax1.set_zlim(0,1.0)
    ax1.set_zticks([0,0.2,0.4,0.6,0.8,1.0])

    ## SUBPLOT 2 (PREDICTION)
    RREV_eval = np.linspace(2,6,10).reshape(-1,1)
    OF_y_eval = np.linspace(0,-15,10).reshape(-1,1)
    d_ceil_eval = np.linspace(0.1,1,10).reshape(-1,1)
    RREV_eval,OF_y_eval,d_ceil_eval = np.meshgrid(RREV_eval,OF_y_eval,d_ceil_eval)
    data_array = np.stack((RREV_eval.flatten(),OF_y_eval.flatten(),d_ceil_eval.flatten())).T
    df = pd.DataFrame(data_array,columns=['RREV','OF_y','d_ceil',])

    X_plot = torch.FloatTensor(df[['RREV','OF_y','d_ceil']].to_numpy())
    with torch.no_grad():
        y_plot = model.forward(X_plot)

    df['LR'] = y_plot.numpy()
    # df = df.query('LR >= 0.6')

    
    ax2 = fig.add_subplot(1,2,2,projection='3d')
    ax2.scatter(
        df['OF_y'],
        df['RREV'],
        df['d_ceil'],
        c = df['LR'],
        cmap=cmap,norm=norm,alpha=0.8,depthshade=False
    )

    ax2.set_xlabel('OF_y')
    ax2.set_xlim(-20,0)
    ax2.set_xticks([-20,-15,-10,-5,0])


    ax2.set_ylabel('RREV')
    ax2.set_ylim(0,8)
    ax2.set_yticks([0,2,4,6,8])

    ax2.set_zlabel('d_ceiling')
    ax2.set_zlim(0,1.0)
    ax2.set_zticks([0,0.2,0.4,0.6,0.8,1.0])
        
    


    # fig.tight_layout()
    plt.show()