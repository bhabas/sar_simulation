import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split

BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"
os.system("clear")



def myfun(x1,x2,x3):
    y1 = np.sqrt(x1**2 + x2**2 + x3**2)
    y1 = [1 if 1 < ii < 2 else 0 for ii in y1.flatten()]
    # [1 if 2 < i < 5 else 0 for i in x]
    # y2 = np.sin(np.sqrt(x1**2 + x2**2)) # sinus

    return y1

## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=3,h1=10,h2=10,out_features=1):
        super().__init__()

        # Input Layer (3 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h1) # Fully connected layer
        self.fc2 = nn.Linear(h1,h2)
        self.out = nn.Linear(h2,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = torch.tanh(self.fc1(x))
        x = torch.tanh(self.fc2(x))
        x = torch.sigmoid(self.out(x))


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

    torch.save(model,f'{BASEPATH}/Func_approx_3D.pt')
    return model


if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    batch_size = 30
    epochs = 5_000

    range_train = [-3,3]

    x1 = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)
    x2 = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)
    x3 = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)

    X1,X2,X3 = np.meshgrid(x1,x2,x3)

    y1 = myfun(X1,X2,X3)

    data_array = np.stack((X1.flatten(),X2.flatten(),X1.flatten(),y1)).T
    df = pd.DataFrame(data_array,columns=['X1','X2','X3','y1'])

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train, test = train_test_split(df,test_size=0.2,random_state=33)

    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train[['X1','X2','X3']].to_numpy())
    X_test = torch.FloatTensor(test[['X1','X2','X3']].to_numpy())

    y_train = torch.FloatTensor(train[['y1']].to_numpy())
    y_test = torch.FloatTensor(test[['y1']].to_numpy())








    # train_model(epochs,X_train,y_train)
    model = torch.load(f'{BASEPATH}/Func_approx_3D.pt')

    ## DEFINE EVALUATION RANGE 
    x1_eval = np.linspace(-3,3,10).reshape(-1,1)
    x2_eval = np.linspace(-3,3,10).reshape(-1,1)
    x3_eval = np.linspace(-3,3,10).reshape(-1,1)

    X1_eval,X2_eval,X3_eval = np.meshgrid(x1_eval,x2_eval,x3_eval)
    X_eval = np.stack((X1_eval.flatten(),X2_eval.flatten(),X3_eval.flatten()),axis=1)
    X_eval = torch.FloatTensor(X_eval)


    # with torch.no_grad():
    #     y_eval = model.forward(X_test)
    #     y_eval = np.round(y_eval.numpy(),0)

    #     for ii in range(len(y_eval)):
    #         print(f"y1: {y_test[ii,0].numpy():.1f} \t y1_eval: {y_eval[ii,0]:.1f} \t%Error: {((y_test[ii,0].numpy()-y_eval[ii,0])/y_test[ii,0].numpy()):.2f}")

    with torch.no_grad():
        y_eval = model.forward(X_eval)
        y_eval = np.round(y_eval.numpy(),0)

    fig = plt.figure(1,figsize=(12,6))

    ## SUBPLOT 1 (FUNCTION)
    ax1 = fig.add_subplot(2,3,1,projection='3d')
    ax1.scatter(
        X_eval[:,0].flatten(),
        X_eval[:,1].flatten(),
        X_eval[:,2].flatten(),
        c = myfun(X_eval[:,0].flatten(),X_eval[:,1].flatten(),X_eval[:,2].flatten()),
        cmap='jet',linewidth=0.2,antialiased=True)
    ax1.set_xlabel('x1')
    ax1.set_ylabel('x2')
    ax1.set_ylabel('x3')
    ax1.set_title('Function')
    ax1.set_xlim(-3,3)
    ax1.set_ylim(-3,3)
    ax1.set_zlim(-3,3)


    ## SUBPLOT 1 (FUNCTION)
    ax2 = fig.add_subplot(2,3,2,projection='3d')
    ax2.scatter(
        X_eval[:,0].flatten(),
        X_eval[:,1].flatten(),
        X_eval[:,2].flatten(),
        c = y_eval[:,0],
        cmap='jet',linewidth=0.2,antialiased=True)
    ax2.set_xlabel('x1')
    ax2.set_ylabel('x2')
    ax2.set_ylabel('x3')
    ax2.set_title('Function')
    ax2.set_xlim(-3,3)
    ax2.set_ylim(-3,3)
    ax2.set_zlim(-3,3)

    plt.show()

    # ## SUBPLOT 2 (PREDICTION)
    # ax2 = fig.add_subplot(2,3,2,projection='3d')
    # ax2.plot_trisurf(
    #     X_eval[:,0].flatten(),
    #     X_eval[:,1].flatten(),
    #     y_eval[:,0].flatten(),
    #     cmap='viridis',linewidth=0.2,antialiased=True)
    # ax2.set_xlabel('x1')
    # ax2.set_ylabel('x2')
    # ax2.set_title('Prediction')
    # ax2.set_xlim(-3,3)
    # ax2.set_ylim(-3,3)

    # ## SUBPLOT 3 (ERROR)
    # ax2 = fig.add_subplot(2,3,3,projection='3d')
    # ax2.plot_trisurf(
    #     X_eval[:,0].flatten(),
    #     X_eval[:,1].flatten(),
    #     np.abs(y_eval[:,0].flatten()-myfun(X_eval[:,0].flatten(),X_eval[:,1].flatten())[0]),
    #     cmap='viridis',linewidth=0.2,antialiased=True)
    # ax2.set_xlabel('x1')
    # ax2.set_ylabel('x2')
    # ax2.set_title('Error')
    # ax2.set_xlim(-3,3)
    # ax2.set_ylim(-3,3)

    # ## SUBPLOT 4 (FUNCTION)
    # ax1 = fig.add_subplot(2,3,4,projection='3d')
    # ax1.plot_trisurf(
    #     X_eval[:,0].flatten(),
    #     X_eval[:,1].flatten(),
    #     myfun(X_eval[:,0].flatten(),X_eval[:,1].flatten())[1],
    #     cmap='viridis',linewidth=0.2,antialiased=True)
    # ax1.set_xlabel('x1')
    # ax1.set_ylabel('x2')
    # ax1.set_title('Function')
    # ax1.set_xlim(-3,3)
    # ax1.set_ylim(-3,3)

    # ## SUBPLOT 5 (PREDICTION)
    # ax2 = fig.add_subplot(2,3,5,projection='3d')
    # ax2.plot_trisurf(
    #     X_eval[:,0].flatten(),
    #     X_eval[:,1].flatten(),
    #     y_eval[:,1].flatten(),
    #     cmap='viridis',linewidth=0.2,antialiased=True)
    # ax2.set_xlabel('x1')
    # ax2.set_ylabel('x2')
    # ax2.set_title('Prediction')
    # ax2.set_xlim(-3,3)
    # ax2.set_ylim(-3,3)

    # ## SUBPLOT 6 (ERROR)
    # ax2 = fig.add_subplot(2,3,6,projection='3d')
    # ax2.plot_trisurf(
    #     X_eval[:,0].flatten(),
    #     X_eval[:,1].flatten(),
    #     np.abs(y_eval[:,1].flatten()-myfun(X_eval[:,0].flatten(),X_eval[:,1].flatten())[1]),
    #     cmap='viridis',linewidth=0.2,antialiased=True)
    # ax2.set_xlabel('x1')
    # ax2.set_ylabel('x2')
    # ax2.set_title('Error')
    # ax2.set_xlim(-3,3)
    # ax2.set_ylim(-3,3)
    


    # fig.tight_layout()
    # plt.show()
