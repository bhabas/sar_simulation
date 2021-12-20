import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split

import plotly.graph_objects as go


BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"



def myfun(x1,x2,x3):
    y = np.sqrt(x1**2 + x2**2 + x3**2)
    
    return y

## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=3,h=5,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h) # Fully connected layer
        self.fc2 = nn.Linear(h,h)
        self.out = nn.Linear(h,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = torch.sigmoid(self.fc1(x))
        x = torch.sigmoid(self.fc2(x))
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

    torch.save(model,f'{BASEPATH}/Pickle_Files/Func_approx_3D.pt')
    
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


    batch_size = 20
    range_train = [-3,3]


    x1 = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)
    x2 = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)
    x3 = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)
    X1,X2,X3 = np.meshgrid(x1,x2,x3)

    y = myfun(X1,X2,X3)

    ## CONVERT DATA INTO DATAFRAME
    data_array = np.stack((X1.flatten(),X2.flatten(),X3.flatten(),y.flatten()),axis=1)
    df = pd.DataFrame(data_array,columns=['X1','X2','X3','y'])

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.2,random_state=33)

    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train_df[['X1','X2','X3']].to_numpy())
    X_test = torch.FloatTensor(test_df[['X1','X2','X3']].to_numpy())

    y_train = torch.FloatTensor(train_df[['y']].to_numpy()).reshape((-1,1))
    y_test = torch.FloatTensor(test_df[['y']].to_numpy()).reshape((-1,1))

    ## TRAIN NN MODEL
    epochs = 3_000
    # train_model(epochs,X_train,y_train,X_test,y_test)

    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Pickle_Files/Func_approx_3D.pt')
    with torch.no_grad():
        y_pred_test = model.forward(X_test)

    ## DEFINE PLOTTING RANGE
    x1_plot = np.linspace(-3,3,20).reshape(-1,1)
    x2_plot = np.linspace(-3,3,20).reshape(-1,1)
    x3_plot = np.linspace(-3,3,20).reshape(-1,1)
    X1_plot,X2_plot,X3_plot = np.meshgrid(x1_plot,x2_plot,x3_plot)
    X_plot = np.stack((X1_plot.flatten(),X2_plot.flatten(),X3_plot.flatten()),axis=1)
    X_plot = torch.FloatTensor(X_plot)

    with torch.no_grad():
        y_pred_plot = model.forward(X_plot)

    fig = go.Figure()

    fig.add_trace(
        go.Isosurface(
            x=X1_plot.flatten(),
            y=X2_plot.flatten(),
            z=X3_plot.flatten(),
            value=y_pred_plot,
            surface_count=3,
            opacity=0.2,
            isomin=0.79,
            isomax=2,
            caps=dict(x_show=False, y_show=False)
        ))

    # fig.add_trace(
    #     go.Scatter3d(
    #         x=X1_plot.flatten(),
    #         y=X2_plot.flatten(),
    #         z=X3_plot.flatten(),
    #         mode='markers',
    #         marker=dict(
    #             size=3,
    #             color=myfun(X1_plot.flatten(),X2_plot.flatten(),X3_plot.flatten()).flatten(),                # set color to an array/list of desired values
    #             colorscale='Viridis',   # choose a colorscale
    #             opacity=0.4)
    #     ))
    fig.show()

