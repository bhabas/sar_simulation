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
from sklearn.datasets import make_blobs,make_moons
from sklearn.metrics import *
from sklearn import preprocessing


np.set_printoptions(suppress=True)
BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"

## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=2,h1=5,h2=5,out_features=1):
        super().__init__()
        h = 20
        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(2,h) # Fully connected layer
        self.out = nn.Linear(h,1)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.elu(self.fc1(x))
        x = torch.sigmoid(self.out(x))

        return x

def train_model(epochs,X_train,y_train):
    model = Model()
    weight = torch.tensor([0.1, 0.9])
    weight_ = weight[y_train.data.view(-1).long()].view_as(y_train)
    criterion = nn.BCELoss(reduction='none')


    optimizer = torch.optim.Adam(model.parameters(), lr=0.01)
    losses = []
    accuracy_list = []

    for ii in range(epochs):
        ## MODEL PREDICTION
        y_pred = model.forward(X_train)


        ## CALCULATE LOSS/ERROR
        loss = criterion(y_pred,y_train)
        loss_class_weighted = loss * weight_
        loss_class_weighted = loss_class_weighted.mean()
        losses.append(loss_class_weighted)
        y_pred_class = np.where(y_pred.detach().numpy()<0.5, 0, 1)
        accuracy = balanced_accuracy_score(y_train[:,0],y_pred_class)
        accuracy_list.append(accuracy)

        if ii%10 == 1:
            print(f"epoch {ii} and loss is: {loss}")

        ## BACKPROPAGATION
        optimizer.zero_grad()
        loss_class_weighted.backward()
        optimizer.step()

    torch.save(model,f'{BASEPATH}/Pickle_Files/Classifier_2D_BCE.pt')

    fig, ax = plt.subplots(2, 1, figsize=(12,8))
    ax[0].plot(losses)
    ax[0].set_ylabel('Loss')
    ax[0].set_title('Training Loss')

    ax[1].plot(accuracy_list)
    ax[1].set_ylabel('Classification Accuracy')
    ax[1].set_title('Training Accuracy')

    plt.tight_layout()
    # plt.show()

    return model



if __name__ == "__main__":

    ## GENERATE DATA
    np.random.seed(0)

    X,y = make_moons(500,noise=0.2)

    ## GENERATE DATA
    n_samples_1 = 500
    centers = [[0.0, 0.0]]
    clusters_std = [3.0]
    X, y = make_blobs(
        n_features=1,
        n_samples=[n_samples_1],
        centers=centers,
        cluster_std=clusters_std,
        random_state=0,
        shuffle=False,
    )
    X1 = X[:,0]
    X2 = X[:,1]

    r = np.sqrt(X1**2 + X2**2)
    y = [1 if ii < 1.0 else 0 for ii in r]
    y = np.array(y)


    ## CONVERT DATA TO DATAFRAME
    data_array = np.stack((X[:,0],X[:,1],y),axis=1)
    df = pd.DataFrame(data_array,columns=['X1','X2','y'])

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.25,random_state=73)


    # plt.figure(figsize=(12,8))
    # plt.scatter(train_df['X1'],train_df['X2'],c=train_df['y'])
    # plt.title('Moon Data')
    # plt.show()


    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train_df[['X1','X2']].to_numpy())
    X_test = torch.FloatTensor(test_df[['X1','X2']].to_numpy())

    y_train = torch.FloatTensor(train_df[['y']].to_numpy()).reshape(-1, 1)
    y_test = torch.FloatTensor(test_df[['y']].to_numpy()).reshape(-1, 1)

    ## TRAIN NN MODEL
    epochs = 1000
    torch.manual_seed(0)
    train_model(epochs,X_train,y_train[:,0].reshape(-1,1))

    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Pickle_Files/Classifier_2D_BCE.pt')

    with torch.no_grad():
        y_pred = model.forward(X_test)
        y_pred = np.round(y_pred,0)
    print(balanced_accuracy_score(y_test[:,0],y_pred))
    print(confusion_matrix(y_test[:,0],y_pred,normalize=None))
    print(classification_report(y_test[:,0],y_pred))


    


    




    # Plot the decision boundary
    # Determine grid range in x and y directions
    x_min, x_max = X[:, 0].min()-0.1, X[:, 0].max()+0.1
    y_min, y_max = X[:, 1].min()-0.1, X[:, 1].max()+0.1

    # Set grid spacing parameter
    spacing = min(x_max - x_min, y_max - y_min) / 100

    # Create grid
    XX, YY = np.meshgrid(np.arange(x_min, x_max, spacing),
                np.arange(y_min, y_max, spacing))

    # Concatenate data to match input
    data = np.hstack((XX.ravel().reshape(-1,1), 
                    YY.ravel().reshape(-1,1)))

    # Pass data to predict method
    with torch.no_grad():

        data_t = torch.FloatTensor(data)
        y_pred = model.forward(data_t)
        y_pred_class = np.where(y_pred.detach().numpy()<0.5, 0, 1)


    # print(balanced_accuracy_score(y_test[:,0],y_pred))
    # print(confusion_matrix(y_test[:,0],y_pred,normalize=None))
    # print(classification_report(y_test[:,0],y_pred))

    clf = np.where(y_pred<0.5,0,1)

    Z = clf.reshape(XX.shape)

    plt.figure(2,figsize=(12,8))
    plt.contourf(XX, YY, Z, cmap=plt.cm.Accent, alpha=0.5)
    plt.scatter(X_train[:,0], X_train[:,1], c=y_train[:,0], 
                cmap=plt.cm.Accent)
    plt.show()



