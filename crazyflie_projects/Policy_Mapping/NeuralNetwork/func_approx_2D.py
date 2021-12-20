import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split

BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"



def myfun(x1,x2):
    y1 = np.power(x1,2)-np.power(x2,2)
    

    return y1

## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=2,h=6,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h) # Fully connected layer
        self.fc2 = nn.Linear(h,h)
        self.out = nn.Linear(h,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = torch.tanh(self.fc1(x))
        x = torch.tanh(self.fc2(x))
        x = self.out(x)

        return x

def train_model(epochs,X_train,y_train,X_test,y_test):

    ## INITIALIZE NEURAL NETWORK MODEL
    model = Model()
    optimizer = torch.optim.Adam(model.parameters(),lr=0.01) #  Model parameters are the layers of model

    ## DEFINE TRAINING LOSS
    criterion = nn.MSELoss(reduction='mean')
    losses_train = []

    ## DEFINE VALIDATION LOSS
    criterion_val = nn.MSELoss(reduction='mean')
    losses_test = []


    for ii in range(epochs):

        ## MODEL PREDICTION
        y_pred = model.forward(X_train)


        ## CALCULATE LOSS/ERROR
        loss_train = criterion(y_pred,y_train)
        losses_train.append(loss_train.item())

        ## CALCULATED VALIDATION LOSS
        with torch.no_grad():
            y_pred_test = model.forward(X_test)

        loss_test = criterion_val(y_pred_test,y_test)
        losses_test.append(loss_test.item())

        if ii%10 == 1:
            print(f"epoch {ii} and loss is: {loss_train}")

        ## BACKPROPAGATION
        optimizer.zero_grad()
        loss_train.backward()
        optimizer.step()

    torch.save(model,f'{BASEPATH}/Pickle_Files/Func_approx_2D.pt')
    
    ## PLOT LOSSES AND ACCURACIES
    fig = plt.figure(1,figsize=(12,8))
    ax1 = fig.add_subplot(1,1,1)
    ax1.plot(losses_train,label="Training Loss")
    ax1.plot(losses_test,label="Validation Loss")
    ax1.set_ylabel("Loss")
    ax1.set_title("Losses")
    ax1.set_ylim(0)
    ax1.legend()
    ax1.grid()


    plt.tight_layout()
    plt.show()
    return model


if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    torch.manual_seed(22)
    np.random.seed(0)


    batch_size = 50
    range_train = [-3,3]


    x1 = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)
    x2 = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)
    X1,X2 = np.meshgrid(x1,x2)

    y = myfun(X1,X2)

    ## CONVERT DATA INTO DATAFRAME
    data_array = np.stack((X1.flatten(),X2.flatten(),y.flatten()),axis=1)
    df = pd.DataFrame(data_array,columns=['X1','X2','y'])

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.2,random_state=33)

    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train_df[['X1','X2']].to_numpy())
    X_test = torch.FloatTensor(test_df[['X1','X2']].to_numpy())

    y_train = torch.FloatTensor(train_df[['y']].to_numpy()).reshape((-1,1))
    y_test = torch.FloatTensor(test_df[['y']].to_numpy()).reshape((-1,1))

    ## TRAIN NN MODEL
    epochs = 3_000
    train_model(epochs,X_train,y_train,X_test,y_test)

    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Pickle_Files/Func_approx_2D.pt')
    with torch.no_grad():
        y_pred_test = model.forward(X_test)

    ## DEFINE PLOTTING RANGE
    x1_plot = np.linspace(-3,3,20).reshape(-1,1)
    x2_plot = np.linspace(-3,3,20).reshape(-1,1)
    X1_plot,X2_plot = np.meshgrid(x1_plot,x2_plot)
    X_plot = np.stack((X1_plot.flatten(),X2_plot.flatten()),axis=1)
    X_plot = torch.FloatTensor(X_plot)

    with torch.no_grad():
        y_pred_plot = model.forward(X_plot)


    fig = plt.figure(1,figsize=(12,4))

    ## SUBPLOT 1 (FUNCTION)
    ax1 = fig.add_subplot(1,3,1,projection='3d')
    ax1.plot_trisurf(
        X_plot[:,0].flatten(),
        X_plot[:,1].flatten(),
        myfun(X_plot[:,0].flatten(),X_plot[:,1].flatten()),
        cmap='viridis',linewidth=0.2,antialiased=True)
    ax1.set_xlabel('x1')
    ax1.set_ylabel('x2')
    ax1.set_title('Function')
    ax1.set_xlim(-3,3)
    ax1.set_ylim(-3,3)

    ## SUBPLOT 2 (PREDICTION)
    ax2 = fig.add_subplot(1,3,2,projection='3d')
    ax2.plot_trisurf(
        X_plot[:,0].flatten(),
        X_plot[:,1].flatten(),
        y_pred_plot[:,0].flatten(),
        cmap='viridis',linewidth=0.2,antialiased=True)
    ax2.set_xlabel('x1')
    ax2.set_ylabel('x2')
    ax2.set_title('Prediction')
    ax2.set_xlim(-3,3)
    ax2.set_ylim(-3,3)

    ## SUBPLOT 3 (ERROR)
    ax3 = fig.add_subplot(1,3,3,projection='3d')
    ax3.plot_trisurf(
        X_plot[:,0].flatten(),
        X_plot[:,1].flatten(),
        np.abs(myfun(X_plot[:,0].flatten(),X_plot[:,1].flatten()) - y_pred_plot[:,0].flatten()),
        cmap='viridis',linewidth=0.2,antialiased=True)
    ax3.set_xlabel('x1')
    ax3.set_ylabel('x2')
    ax3.set_title('Error')
    ax3.set_xlim(-3,3)
    ax3.set_ylim(-3,3)



    fig.tight_layout()
    plt.show()
