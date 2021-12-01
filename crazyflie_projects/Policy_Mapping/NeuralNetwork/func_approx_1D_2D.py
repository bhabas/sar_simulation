import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split


def myfun(x):
    y1 = np.sin(x)
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

        return x1

def train_model(epochs,X_train,y1_train,y2_train):

    ## INITIALIZE NEURAL NETWORK MODEL
    torch.manual_seed(22)
    model = Model()

    criterion1 = nn.MSELoss(reduction='mean')

    optimizer = torch.optim.Adam(model.parameters(),lr=0.01) #  Model parameters are the layers of model
    losses = []

    for ii in range(epochs):

        ## MODEL PREDICTION
        y1_pred = model.forward(X_train)

        ## CALCULATE LOSS/ERROR
        loss1 = criterion1(y1_pred,y1_train)

        loss = loss1

        losses.append(loss)

        if ii%10 == 1:
            print(f"epoch {ii} and loss is: {loss}")

        ## BACKPROPAGATION
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    torch.save(model,'Func_approx_1D_2D.pt')
    return model


if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    batch_size = 50
    epochs = 5000

    range_train = [-1,7]
    X = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)
    y1,y2 = myfun(X)

    data_array = np.stack((X.flatten(),y1.flatten(),y2.flatten())).T
    df = pd.DataFrame(data_array,columns=['X','y1','y2'])

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train, test = train_test_split(df,test_size=0.2,random_state=33)

    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train['X'].to_numpy()).reshape((-1,1))
    X_test = torch.FloatTensor(test['X'].to_numpy()).reshape((-1,1))

    y1_train = torch.FloatTensor(train['y1'].to_numpy()).reshape((-1,1))
    y1_test = torch.FloatTensor(test['y1'].to_numpy()).reshape((-1,1))

    y2_train = torch.FloatTensor(train['y2'].to_numpy()).reshape((-1,1))
    y2_test = torch.FloatTensor(test['y2'].to_numpy()).reshape((-1,1))


    # train_model(epochs,X_train,y1_train,y2_train)
    model = torch.load('Func_approx_1D_2D.pt')

    ## DEFINE EVALUATION RANGE 
    X_eval = np.linspace(-3,10,50).reshape(-1,1)
    X_eval = torch.FloatTensor(X_eval)

    with torch.no_grad():
        y1_eval = model.forward(X_eval)


    ## PLOT NETWORK OUTPUTS
    fig = plt.figure(1,figsize=(12,6))

    ax1 = fig.add_subplot(2,2,1)
    ax1.plot(X_eval,y1_eval,color='blue',label='NN Output')
    ax1.plot(X_train,myfun(X_train)[0],'.',label='Training Data')
    ax1.axvspan(range_train[0],range_train[1], alpha=0.15, color='limegreen',label='Training Range')

    ax1.grid()
    ax1.set_xlabel('x')
    ax1.set_ylabel('f(x)')
    ax1.legend()

    ax2 = fig.add_subplot(2,2,2)
    ax2.plot(X_eval,np.abs(y1_eval - myfun(X_eval)[0]),label=('Error |f(x) - f\'(x)|'))
    ax2.axvspan(range_train[0],range_train[1], alpha=0.15, color='limegreen',label='Training Range')


    ax2.grid()
    ax2.set_xlabel('x')
    ax2.set_ylabel('Error')
    ax2.set_title('Absolute difference between prediction and actual function')
    ax2.legend()


    fig.tight_layout()
    plt.show()
