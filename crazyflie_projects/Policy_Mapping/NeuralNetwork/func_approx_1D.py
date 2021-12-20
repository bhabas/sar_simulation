import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split

BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"


def myfun(x):
    y1 = np.sin(x) + np.cos(x)**2

    return y1

## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=1,h=6,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h) # Fully connected layer
        self.fc2 = nn.Linear(h,h)
        self.out = nn.Linear(h,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.sigmoid(self.fc1(x))
        x = F.sigmoid(self.fc2(x))
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
        y_pred_train = model.forward(X_train)


        ## CALCULATE TRAINING LOSS
        loss_train = criterion(y_pred_train,y_train)
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

    torch.save(model,f'{BASEPATH}/Pickle_Files/Func_approx_1D.pt')


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
    range_train = [-1,7]
    X = np.linspace(range_train[0],range_train[1],batch_size).reshape(-1,1)
    y1 = myfun(X)

    ## CONVERT DATA INTO DATAFRAME
    data_array = np.stack((X.flatten(),y1.flatten()),axis=1)
    df = pd.DataFrame(data_array,columns=['X1','y'])

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.2,random_state=33)

    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train_df['X1'].to_numpy()).reshape((-1,1))
    X_test = torch.FloatTensor(test_df['X1'].to_numpy()).reshape((-1,1))

    y_train = torch.FloatTensor(train_df[['y']].to_numpy()).reshape((-1,1))
    y_test = torch.FloatTensor(test_df[['y']].to_numpy()).reshape((-1,1))

    ## TRAIN NN MODEL
    epochs = 5_000
    # train_model(epochs,X_train,y_train,X_test,y_test)

    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Pickle_Files/Func_approx_1D.pt')
    
    with torch.no_grad():
        y_pred_test = model.forward(X_test)

    ## DEFINE PLOTTING RANGE 
    X_plot = np.linspace(-3,10,100).reshape(-1,1)
    X_plot = torch.FloatTensor(X_plot)

    with torch.no_grad():
        y_pred_plot = model.forward(X_plot)


    ## PLOT NETWORK OUTPUTS
    fig = plt.figure(1,figsize=(12,6))

    ax1 = fig.add_subplot(1,2,1)
    ax1.plot(X_plot,y_pred_plot[:,0],'g',alpha=0.5,label='NN Output')
    ax1.plot(X_plot,myfun(X_plot),'g--',alpha=0.5,label='f(x) Output')
    ax1.plot(X_test,y_pred_test,'.',label='Testing Data')
    ax1.axvspan(range_train[0],range_train[1], alpha=0.15, color='limegreen',label='Training Range')

    ax1.grid()
    ax1.set_xlabel('x')
    ax1.set_ylabel('f(x)')
    ax1.set_title('NN Output vs Function Output')
    ax1.set_xlim(-5,10)
    ax1.legend()

    ax2 = fig.add_subplot(1,2,2)
    ax2.plot(X_plot,np.abs(myfun(X_plot).flatten() - y_pred_plot[:,0]),color='red',label=('Error |f(x) - f_NN(x)|'))
    ax2.axvspan(range_train[0],range_train[1], alpha=0.15, color='limegreen',label='Training Range')


    ax2.grid()
    ax2.set_xlabel('x')
    ax2.set_xlim(-5,10)
    ax2.set_ylabel('Error')
    ax2.set_title('Error Between Prediction and Actual Function')
    ax2.legend()


    fig.tight_layout()
    plt.show()
