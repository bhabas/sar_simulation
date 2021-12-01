import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
import os
import pandas as pd

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F


def myfun(x):
    y1 = np.sin(x)
    # y_valid = [1 if range_min < i < range_max else 0 for i in x]
    # y_valid = np.reshape(y_valid,(-1,1))
    y2 = np.cos(x)

    return y1,y2
        


## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=1,h1=16,h2=16,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h1) # Fully connected layer
        self.fc2 = nn.Linear(h1,h2)
        self.out = nn.Linear(h2,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.tanh(self.fc1(x))
        x = F.tanh(self.fc2(x))
        x1 = self.out(x)
        x2 = self.out(x)
        ## Try NN to predict valid region and then combine both

        return x1,x2


def train_model(X_train,y_train1,y_train2):
    ## INITIALIZE NEURAL NETWORK MODEL
    torch.manual_seed(3)
    model = Model()

    ## DEFINE OPTIMIZER AND LOSS FUNCTION (CRITERION)
    criterion1 = nn.MSELoss(reduction='mean')
    criterion2 = nn.MSELoss(reduction='mean')

    optimizer = torch.optim.Adam(model.parameters(),lr=0.05) # Model parameters are the layers of model


    epochs = 50
    losses = []

    for i in range(epochs):

        ## MODEL PREDICTION
        y_pred1,y_pred2 = model.forward(X_train)

        ## CALCULATE LOSS/ERROR
        loss1 = criterion1(y_pred1,y_train1)
        loss2 = criterion2(y_pred2,y_train2)
        loss = loss2

        losses.append(loss)

        if i%10 == 1:
            print(f"epoch {i} and loss is: {loss}")

        

        ## BACKPROPAGATION
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()



    # ## PLOT LOSS VS EPOCHS
    # plt.plot(range(epochs),losses)
    # plt.ylabel('Loss')
    # plt.xlabel('Epoch')

    # plt.show()

    torch.save(model,'Func_approx_1D_2D.pt')
    return model


def test_model(model):

    ## DEFINE EVALUATION RANGE
    X_eval = np.linspace(-3,10,50).reshape(-1,1)
    X_eval = torch.FloatTensor(X_eval)
    with torch.no_grad():
        y_eval = model.forward(X_eval)



    ## PLOT NETWORK OUTPUTS
    fig = plt.figure(1, figsize=(12,6))

    ax3 = fig.add_subplot(2,2,3)
    ax3.plot(X_eval,y_eval,color='red')
    ax3.grid()
    ax3.set_xlabel('x')
    ax3.set_ylabel('f(x)')

    ax4 = fig.add_subplot(2,2,4)
    ax4.plot(X_eval,np.abs(y_eval))


    # ax1.axvspan(X.flatten()[0], X.flatten()[-1], alpha=0.15, color='limegreen')
    # ax1.plot(X_eval, np.sin(X_eval), '-', color='royalblue', linewidth=1.0)
    # ax1.plot(X_eval, y_eval, '-', label='output', color='darkorange', linewidth=2.0)
    # ax1.plot(X_train, np.sin(X_train), '.', color='royalblue')

    # ax1.grid()
    # ax1.set_xlabel('x')
    # ax1.set_ylabel('f(x)')
    # ax1.set_title('%d neurons in hidden layer with %d epochs of training' % (8,epochs))
    # ax1.legend(['Function: f(x)', 'Neural Network: g(x)', 'Training Set'])

    # ax2 = fig.add_subplot(1,2,2)
    # ax2.axvspan(X.flatten()[0], X.flatten()[-1], alpha=0.15, color='limegreen')
    # ax2.plot(X_eval, np.abs(y_eval-np.sin(X_eval)), '-', label='output', color='firebrick', linewidth=2.0)

    # ax2.grid()
    # ax2.set_xlabel('x')
    # ax2.set_ylabel('Error')
    # ax2.set_title('Absolute difference between prediction and actual function')
    # ax2.legend(['Error |f(x)-g(x)|'])
    # fig.tight_layout()
    plt.show()

if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    batch_size = 50
    range_min = 1
    range_max = 4

    X = np.linspace(-1,7,batch_size).reshape(-1,1)
    y,y_valid = myfun(X)

    data_array = np.stack((X.flatten(),y.flatten(),y_valid.flatten())).T
    df = pd.DataFrame(data_array,columns=['X','y','y_valid'])


    ## SPLIT DATA FEATURES INTO TRAINING DATA & TESTING DATA
    train, test = train_test_split(df, test_size=0.2,random_state=33)

    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train[['X']].to_numpy())
    X_test = torch.FloatTensor(test[['X']].to_numpy())

    y_train1 = torch.FloatTensor(train[['y']].to_numpy())
    y_test1 = torch.FloatTensor(test[['y']].to_numpy())

    y_train2 = torch.FloatTensor(train[['y_valid']].to_numpy())
    y_test2 = torch.FloatTensor(test[['y_valid']].to_numpy())

    train_model(X_train,y_train1,y_train2)
    model = torch.load('Func_approx_1D_2D.pt')
    # test_model(model)



    ## DEFINE EVALUATION RANGE
    X_eval = np.linspace(-3,10,50).reshape(-1,1)
    X_eval = torch.FloatTensor(X_eval)
    with torch.no_grad():
        y_eval1,y_eval2 = model.forward(X_eval)



    ## PLOT NETWORK OUTPUTS
    fig = plt.figure(1, figsize=(12,6))

    ax1 = fig.add_subplot(2,2,1)
    ax1.plot(X_eval,y_eval1,color='blue')
    ax1.plot(X_train,myfun(X_train,range_min,range_max)[0],'.')

    ax1.grid()
    ax1.set_xlabel('x')
    ax1.set_ylabel('f(x)')
    ax1.set_title('%d neurons in hidden layer with %d epochs of training' % (8,10000))
    ax1.legend(['Function: f(x)', 'Neural Network: g(x)', 'Training Set'])

    ax2 = fig.add_subplot(2,2,2)
    ax2.plot(X_eval,np.abs(y_eval1 - myfun(X_eval,range_min,range_max)[0]))

    ax2.set_xlabel('x')
    ax2.set_ylabel('Error')
    ax2.set_title('Absolute difference between prediction and actual function')
    ax2.legend(['Error |f(x)-g(x)|'])



    ax3 = fig.add_subplot(2,2,3)
    ax3.plot(X_eval,y_eval2,color='red')
    ax3.axvspan(range_min, range_max, alpha=0.15, color='limegreen')

    ax3.grid()
    ax3.set_xlabel('x')
    ax3.set_ylabel('f(x)')

    ax4 = fig.add_subplot(2,2,4)
    ax4.plot(X_eval,np.abs(y_eval2-myfun(X_eval,range_min,range_max)[1]))
    ax4.axvspan(range_min,range_max, alpha=0.15, color='limegreen')

    fig.tight_layout()
    plt.show()
