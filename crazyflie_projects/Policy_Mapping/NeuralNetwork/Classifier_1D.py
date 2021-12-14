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
from sklearn.datasets import make_blobs
from sklearn.metrics import *
from sklearn import preprocessing


BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"


## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=1,h1=5,h2=5,out_features=1):
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
        # x[:,1] = F.sigmoid(x[:,1])


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

    torch.save(model,f'{BASEPATH}/Pickle_Files/Classifier_1D.pt')
    return model


if __name__ == '__main__':

    ## GENERATE DATA
    n_samples_1 = 500
    centers = [[0.0]]
    clusters_std = [3.0]
    X, y = make_blobs(
        n_features=1,
        n_samples=[n_samples_1],
        centers=centers,
        cluster_std=clusters_std,
        random_state=0,
        shuffle=False,
    )
    r = np.sqrt(X**2)
    y = [1 if ii < 2.5 else 0 for ii in r]
    y = np.array(y)

    scaler = preprocessing.StandardScaler().fit(X)
    X_scaled = scaler.transform(X)
    X = X_scaled


    ## CONVERT DATA TO DATAFRAME
    data_array = np.stack((X.flatten(),y),axis=1)
    df = pd.DataFrame(data_array,columns=['X1','y'])

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.2,random_state=33)

    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train_df['X1'].to_numpy()).reshape((-1,1))
    X_test = torch.FloatTensor(test_df['X1'].to_numpy()).reshape((-1,1))

    y_train = torch.FloatTensor(train_df[['y']].to_numpy())
    y_test = torch.FloatTensor(test_df[['y']].to_numpy())

    ## TRAIN NN MODEL
    epochs = 10_000
    # train_model(epochs,X_train,y_train)

    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Pickle_Files/Classifier_1D.pt')

    with torch.no_grad():
        y_pred = model.forward(X_test)
        y_pred = np.round(y_pred,0)
    print(balanced_accuracy_score(y_test[:,0],y_pred))
    print(confusion_matrix(y_test[:,0],y_pred,normalize=None))
    print(classification_report(y_test[:,0],y_pred))



    ## PLOT NETWORK OUTPUTS
    fig = plt.figure(1,figsize=(12,6))

    ax1 = fig.add_subplot(1,2,1)

    ax1.scatter(
        test_df['X1'],
        test_df['y'],
        c = test_df['y'],
        cmap='jet',linewidth=0.2,antialiased=True)

    ax1.grid()
    ax1.set_xlabel('x')
    ax1.set_ylabel('f(x)')
    ax1.set_title('NN Output vs Function Output')
    ax1.set_xlim(-10,10)
    ax1.legend()
  

    fig.tight_layout()
    plt.show()
