import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl

import os
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split\

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

    torch.save(model,f'{BASEPATH}/Func_approx_My.pt')

    plt.plot(range(epochs),losses)
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.show()

    return model


if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    batch_size = 50
    epochs = 50_000

    df = pd.read_csv(f"{DATAPATH}/Wide-Long_2-Policy_Summary.csv")
    df = df.query('landing_rate_4_leg >= 0.6')

    
    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train, test = train_test_split(df,test_size=0.2,random_state=33)

    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train[['RREV_flip_mean','OF_y_flip_mean','flip_height_mean']].to_numpy())
    X_test = torch.FloatTensor(test[['RREV_flip_mean','OF_y_flip_mean','flip_height_mean']].to_numpy())

    y_train = torch.FloatTensor(train[['My_d']].to_numpy())
    y_test = torch.FloatTensor(test[['My_d']].to_numpy())








    # train_model(epochs,X_train,y_train)
    model = torch.load(f'{BASEPATH}/Func_approx_My.pt')

    ## MODEL EVALUATION

    with torch.no_grad():
        y_eval = model.forward(X_test)

    for ii in range(len(y_eval)):
        # print(f"LR_test: {y_test[ii,1].numpy():.1f} \t LR_eval: {y_eval[ii,1].numpy():.1f}")
        print(f"My_test: {y_test[ii,0].numpy():.1f} \t My_eval: {y_eval[ii,0].numpy():.1f} \t%Error: {((y_test[ii,0]-y_eval[ii,0])/y_test[ii,0]).numpy():.2f}")

    # ## DEFINE EVALUATION RANGE 
    # x1_eval = np.linspace(-3,3,10).reshape(-1,1)
    # x2_eval = np.linspace(-3,3,10).reshape(-1,1)
    # X1_eval,X2_eval = np.meshgrid(x1_eval,x2_eval)
    # X_eval = np.stack((X1_eval.flatten(),X2_eval.flatten()),axis=1)
    # X_eval = torch.FloatTensor(X_eval)


    # with torch.no_grad():
    #     y_eval = model.forward(X_eval)





# ## PLOT REGIONS
#     fig = plt.figure(1,figsize=(8,7))


#     ## SUBPLOT 1 (FUNCTION)
#     ax1 = fig.add_subplot(2,2,1,projection='3d')
#     cmap = mpl.cm.jet
#     norm = mpl.colors.Normalize(vmin=0,vmax=1.0)
#     ax1.scatter(
#         df['OF_y_flip_mean'],
#         df['RREV_flip_mean'],
#         2.1-df['flip_height_mean'],
#         c = df['landing_rate_4_leg'],
#         cmap=cmap,norm=norm,alpha=0.8,depthshade=False
#     )

#     ax1.set_xlabel('OF_y')
#     ax1.set_xlim(-20,0)
#     ax1.set_xticks([-20,-15,-10,-5,0])


#     ax1.set_ylabel('RREV')
#     ax1.set_ylim(0,8)
#     ax1.set_yticks([0,2,4,6,8])

#     ax1.set_zlabel('d_ceiling')
#     ax1.set_zlim(0,1.0)
#     ax1.set_zticks([0,0.2,0.4,0.6,0.8,1.0])

#     ## SUBPLOT 3 (FUNCTION)
#     ax3 = fig.add_subplot(2,2,3,projection='3d')
#     cmap = mpl.cm.jet
#     norm = mpl.colors.Normalize(vmin=0,vmax=10.0)
#     ax3.scatter(
#         df['OF_y_flip_mean'],
#         df['RREV_flip_mean'],
#         2.1-df['flip_height_mean'],
#         c = df['My_d'],
#         cmap=cmap,norm=norm,alpha=0.8,depthshade=False
#     )

#     ax3.set_xlabel('OF_y')
#     ax3.set_xlim(-20,0)
#     ax3.set_xticks([-20,-15,-10,-5,0])


#     ax3.set_ylabel('RREV')
#     ax3.set_ylim(0,8)
#     ax3.set_yticks([0,2,4,6,8])

#     ax3.set_zlabel('d_ceiling')
#     ax3.set_zlim(0,1.0)
#     ax3.set_zticks([0,0.2,0.4,0.6,0.8,1.0])

    
#     # ax1.plot_trisurf(
#     #     X_eval[:,0].flatten(),
#     #     X_eval[:,1].flatten(),
#     #     myfun(X_eval[:,0].flatten(),X_eval[:,1].flatten())[0],
#     #     cmap='viridis',linewidth=0.2,antialiased=True)
#     # ax1.set_xlabel('x1')
#     # ax1.set_ylabel('x2')
#     # ax1.set_title('Function')
#     # ax1.set_xlim(-3,3)
#     # ax1.set_ylim(-3,3)

#     # ## SUBPLOT 2 (PREDICTION)
#     # ax2 = fig.add_subplot(2,3,2,projection='3d')
#     # ax2.plot_trisurf(
#     #     X_eval[:,0].flatten(),
#     #     X_eval[:,1].flatten(),
#     #     y_eval[:,0].flatten(),
#     #     cmap='viridis',linewidth=0.2,antialiased=True)
#     # ax2.set_xlabel('x1')
#     # ax2.set_ylabel('x2')
#     # ax2.set_title('Prediction')
#     # ax2.set_xlim(-3,3)
#     # ax2.set_ylim(-3,3)

#     # ## SUBPLOT 3 (ERROR)
#     # ax2 = fig.add_subplot(2,3,3,projection='3d')
#     # ax2.plot_trisurf(
#     #     X_eval[:,0].flatten(),
#     #     X_eval[:,1].flatten(),
#     #     np.abs(y_eval[:,0].flatten()-myfun(X_eval[:,0].flatten(),X_eval[:,1].flatten())[0]),
#     #     cmap='viridis',linewidth=0.2,antialiased=True)
#     # ax2.set_xlabel('x1')
#     # ax2.set_ylabel('x2')
#     # ax2.set_title('Error')
#     # ax2.set_xlim(-3,3)
#     # ax2.set_ylim(-3,3)

#     # ## SUBPLOT 4 (FUNCTION)
#     # ax1 = fig.add_subplot(2,3,4,projection='3d')
#     # ax1.plot_trisurf(
#     #     X_eval[:,0].flatten(),
#     #     X_eval[:,1].flatten(),
#     #     myfun(X_eval[:,0].flatten(),X_eval[:,1].flatten())[1],
#     #     cmap='viridis',linewidth=0.2,antialiased=True)
#     # ax1.set_xlabel('x1')
#     # ax1.set_ylabel('x2')
#     # ax1.set_title('Function')
#     # ax1.set_xlim(-3,3)
#     # ax1.set_ylim(-3,3)

#     # ## SUBPLOT 5 (PREDICTION)
#     # ax2 = fig.add_subplot(2,3,5,projection='3d')
#     # ax2.plot_trisurf(
#     #     X_eval[:,0].flatten(),
#     #     X_eval[:,1].flatten(),
#     #     y_eval[:,1].flatten(),
#     #     cmap='viridis',linewidth=0.2,antialiased=True)
#     # ax2.set_xlabel('x1')
#     # ax2.set_ylabel('x2')
#     # ax2.set_title('Prediction')
#     # ax2.set_xlim(-3,3)
#     # ax2.set_ylim(-3,3)

#     # ## SUBPLOT 6 (ERROR)
#     # ax2 = fig.add_subplot(2,3,6,projection='3d')
#     # ax2.plot_trisurf(
#     #     X_eval[:,0].flatten(),
#     #     X_eval[:,1].flatten(),
#     #     np.abs(y_eval[:,1].flatten()-myfun(X_eval[:,0].flatten(),X_eval[:,1].flatten())[1]),
#     #     cmap='viridis',linewidth=0.2,antialiased=True)
#     # ax2.set_xlabel('x1')
#     # ax2.set_ylabel('x2')
#     # ax2.set_title('Error')
#     # ax2.set_xlim(-3,3)
#     # ax2.set_ylim(-3,3)
    


#     fig.tight_layout()
#     plt.show()
