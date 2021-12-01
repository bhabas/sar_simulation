import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split


def myfun(x):
    y1 = np.sin(x)
    y2 = [1 if 2 < i < 5 else 0 for i in x]
    y2 = np.reshape(y2,(-1,1))

    return y1,y2

## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=1,h1=5,h2=5,out_features=2):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h1) # Fully connected layer
        self.fc2 = nn.Linear(h1,h2)
        self.out = nn.Linear(h2,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.sigmoid(self.fc1(x))
        x = F.sigmoid(self.fc2(x))
        x = self.out(x)
        x[:,1] = F.sigmoid(x[:,1])


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

    torch.save(model,'Func_approx_1D_2D.pt')
    return model


if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    batch_size = 50
    epochs = 10_000

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

    y_train = torch.FloatTensor(train[['y1','y2']].to_numpy())
    y_test = torch.FloatTensor(test[['y1','y2']].to_numpy())


    train_model(epochs,X_train,y_train)
    model = torch.load('Func_approx_1D_2D.pt')

    ## DEFINE EVALUATION RANGE 
    X_eval = np.linspace(-3,10,50).reshape(-1,1)
    X_eval = torch.FloatTensor(X_eval)

    with torch.no_grad():
        y_eval = model.forward(X_eval)


    ## PLOT NETWORK OUTPUTS
    fig = plt.figure(1,figsize=(12,6))

    ax1 = fig.add_subplot(2,2,1)
    ax1.plot(X_eval,y_eval[:,0],'g',alpha=0.5,label='NN Output')
    ax1.plot(X_eval,myfun(X_eval)[0].flatten(),'g--',alpha=0.5,label='f(x) Output')
    ax1.plot(X_train,myfun(X_train)[0],'.',label='Training Data')
    ax1.axvspan(range_train[0],range_train[1], alpha=0.15, color='limegreen',label='Training Range')

    ax1.grid()
    ax1.set_xlabel('x')
    ax1.set_ylabel('f(x)')
    ax1.set_title('NN Output vs Function Output')
    ax1.set_xlim(-5,10)
    ax1.legend()

    ax2 = fig.add_subplot(2,2,2)
    ax2.plot(X_eval,np.abs(y_eval[:,0] - myfun(X_eval)[0].flatten()),color='red',label=('Error |f(x) - f_NN(x)|'))
    ax2.axvspan(range_train[0],range_train[1], alpha=0.15, color='limegreen',label='Training Range')


    ax2.grid()
    ax2.set_xlabel('x')
    ax2.set_xlim(-5,10)
    ax2.set_ylabel('Error')
    ax2.set_title('Error Between Prediction and Actual Function')
    ax2.legend()

    ax3 = fig.add_subplot(2,2,3)
    ax3.plot(X_eval,y_eval[:,1],'b',alpha=0.5,label='NN Output')
    ax3.plot(X_eval,myfun(X_eval)[1].flatten(),'b--',alpha=0.5,label='f(x) Output')
    ax3.plot(X_train,myfun(X_train)[1],'.',label='Training Data')
    ax3.axvspan(range_train[0],range_train[1], alpha=0.15, color='limegreen',label='Training Range')

    ax3.grid()
    ax3.set_xlabel('x')
    ax3.set_ylabel('g(x)')
    ax3.set_xlim(-5,10)
    ax3.legend()

    ax4 = fig.add_subplot(2,2,4)
    ax4.plot(X_eval,np.abs(y_eval[:,1] - myfun(X_eval)[1].flatten()),color='red',label=('Error |g(x) - g_NN(x)|'))
    ax4.axvspan(range_train[0],range_train[1], alpha=0.15, color='limegreen',label='Training Range')


    ax4.grid()
    ax4.set_xlabel('x')
    ax4.set_xlim(-5,10)
    ax4.set_ylabel('Error')
    ax4.legend()



    fig.tight_layout()
    plt.show()
