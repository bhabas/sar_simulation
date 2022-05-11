## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

## PYTORCH IMPROTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## SKLEARN IMPORTS
from sklearn.model_selection import train_test_split
from sklearn.datasets import make_blobs, make_circles
from sklearn.metrics import *
from sklearn import preprocessing

np.set_printoptions(suppress=True)
BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"


## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=2,h1=5,h2=5,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h1) # Fully connected layer
        self.fc2 = nn.Linear(h1,h2)
        self.out = nn.Linear(h2,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.sigmoid(self.fc1(x))
        x = F.sigmoid(self.fc2(x))
        x = F.sigmoid(self.out(x))

        return x

def train_model(epochs,X_train,y_train):

    ## INITIALIZE NEURAL NETWORK MODEL
    torch.manual_seed(22)
    model = Model()

    criterion = nn.BCELoss()

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

    torch.save(model,f'{BASEPATH}/Classifier_2D.pt')
    return model


if __name__ == '__main__':

    ## GENERATE DATA
    X, y = make_circles(
        n_samples=500,
        noise=0.1,
        random_state=0,
        shuffle=False,
        factor=0.1
    )

    X1 = X[:,0]
    X2 = X[:,1]


    ## CONVERT DATA TO DATAFRAME
    data_array = np.stack((X1,X2,y),axis=1)
    df = pd.DataFrame(data_array,columns=['X1','X2','y'])
    df = df.sort_values(by='y')


    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.2,random_state=33)
    test_df = test_df.sort_values(by='y')


    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train_df[['X1','X2']].to_numpy())
    X_test = torch.FloatTensor(test_df[['X1','X2']].to_numpy())

    y_train = torch.FloatTensor(train_df[['y']].to_numpy())
    y_test = torch.FloatTensor(test_df[['y']].to_numpy())

    ## TRAIN NN MODEL
    epochs = 500
    # train_model(epochs,X_train,y_train[:,0].reshape(-1,1))

    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Classifier_2D.pt')

    with torch.no_grad():
        y_pred = model.forward(X_test)
        y_pred = np.round(y_pred,0)
    print(balanced_accuracy_score(y_test[:,0],y_pred))
    print(confusion_matrix(y_test[:,0],y_pred,normalize=None))
    print(classification_report(y_test[:,0],y_pred))


    with torch.no_grad():
        x1 = np.linspace(-2,2,100)
        x2 = np.linspace(-2,2,100)

        X1_test,X2_test = np.meshgrid(x1,x2)
        X_contour = np.stack((X1_test.flatten(),X2_test.flatten()),axis=1)
        X_contour = torch.FloatTensor(X_contour)


        y_contour = model.forward(X_contour).reshape(100,100)
        # contours = measure.find_contours(y_contour.numpy(), 0.75)

        





    ## PLOT NETWORK OUTPUTS
    fig = plt.figure(1,figsize=(12,6))

    ax1 = fig.add_subplot(1,2,1)

    ax1.contour(X1_test,X2_test,y_contour,levels=[0.98],cmap='jet')
    ax1.scatter(
        test_df['X1'],
        test_df['X2'],
        c = test_df['y'],
        cmap='jet',linewidth=0.2,antialiased=True)

    

    ax1.grid()
    ax1.set_xlabel('X1')
    ax1.set_ylabel('X2')
    # ax1.set_xlim(-10,10)
    # ax1.set_ylim(-10,10)
    ax1.legend()
  

    fig.tight_layout()
    plt.show()
