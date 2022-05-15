## STANDARD IMPORTS
from importlib_metadata import requires
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

## PYTORCH IMPROTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## SKLEARN IMPORTS
from sklearn.model_selection import train_test_split
from sklearn.datasets import *
from sklearn.metrics import *
from sklearn import preprocessing

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping/NeuralNetwork"

def to_labels(pos_probs, threshold):
    return (pos_probs >= threshold).astype('int')


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
        x = torch.sigmoid(self.fc1(x))
        x = torch.sigmoid(self.fc2(x))
        x = torch.sigmoid(self.out(x))

        return x

def train_model(model,X_train,y_train,X_test,y_test,epochs=500):

    ## IMPLEMENT CLASS WEIGHTS
    class_weight = [1, 1] # Weights as binary classes [0,1]

    y_train_class = y_train
    weights_train = np.where(y_train_class==1,class_weight[1],class_weight[0]) # Classify and weight values

    y_test_class = y_test
    weights_test = np.where(y_test_class==1,class_weight[1],class_weight[0])


    ## DEFINE TRAINING LOSS
    criterion_train_class = nn.BCELoss(weight=torch.FloatTensor(weights_train))
    losses_train = []
    accuracies_train = []


    ## DEFINE VALIDATION LOSS
    criterion_test_class = nn.BCELoss(weight=torch.FloatTensor(weights_test))
    losses_test = []
    accuracies_test = []

    optimizer = torch.optim.Adam(model.parameters(),lr=0.01) #  Model parameters are the layers of model

    jj=0

    for ii in range(epochs):


        ## MODEL PREDICTION
        y_pred_train = model.forward(X_train)

        ## CALC TRAINING LOSS
        loss_train_class = criterion_train_class(y_pred_train,y_train_class)
        loss = loss_train_class
        losses_train.append(loss.item())

        ## CALC VALIDATION LOSS
        with torch.no_grad():
            y_pred_test = model.forward(X_test)
            loss_test_class = criterion_test_class(y_pred_test,y_test_class)
            losses_test.append(loss_test_class.item())



        if ii%10 == 0:
            print(f"epoch {ii} and loss is: {losses_train[-1]}")


            # ## FIND OPTIMAL THRESHOLD
            # threshold_arr = np.arange(0, 1, 0.001)
            # scores = [f1_score(y_train, to_labels(y_pred_train.numpy(), threshold)) for threshold in threshold_arr]
            # idx = np.argmax(scores)
            # threshold = threshold_arr[idx]


            with torch.no_grad():
                x1_cgrid = np.linspace(-5,5,100)    # x1 contour grid
                x2_cgrid = np.linspace(-5,5,100)    # x2 contour grid

                X1_cgrid,X2_cgrid = np.meshgrid(x1_cgrid,x2_cgrid)
                X_cgrid = np.stack((X1_cgrid.flatten(),X2_cgrid.flatten()),axis=1)
                X_cgrid = torch.FloatTensor(X_cgrid)

                y_contour = model.forward(X_cgrid).reshape(100,100).numpy()


            ## PLOT DATA/NETWORK OUTPUT
            fig = plt.figure()

            ax = fig.add_subplot(111)
            ax.contour(X1_cgrid,X2_cgrid,y_contour,cmap='jet')
            # ax1.contour(X1_test,X2_test,y_contour,levels=[threshold_arr[ix]],cmap='jet')
            ax.scatter(train_df['X1'],train_df['X2'],c=train_df['y'],cmap='jet')
            

            ax.grid()
            ax.set_title(f'Epoch: {ii:04d}')
            ax.set_xlabel('X1')
            ax.set_ylabel('X2')
            ax.set_xlim(-2,2)
            ax.set_ylim(-2,2)
            # ax.set_aspect('equal')
            # ax1.legend()
        

            fig.tight_layout()
            fig.savefig(f'/tmp/images/Image_{jj:04d}.png')
            plt.close(fig)

            jj+=1

        ## BACKPROPAGATION
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()


    ## PLOT LOSSES AND ACCURACIES
    fig = plt.figure(1,figsize=(12,8))
    ax1 = fig.add_subplot(2,1,1)
    ax1.plot(losses_train,label="Training Loss")
    ax1.plot(losses_test,label="Validation Loss")
    ax1.set_ylabel("Loss")
    ax1.set_title("Losses")
    ax1.set_yscale('log')
    ax1.legend()
    ax1.grid()

    ax2 = fig.add_subplot(2,1,2)
    ax2.plot(accuracies_train,label="Training Accuracy")
    ax2.plot(accuracies_test,label="Validation Accuracy")
    ax2.set_ylim(0,1.1)
    ax2.set_ylabel("Classification Accuracy")
    ax2.set_title("Balanced Accuracy")
    ax2.grid()
    ax2.legend(loc="lower right")


    plt.tight_layout()
    plt.show()

    torch.save(model,f'{BASEPATH}/Classifier_2D.pt')


if __name__ == '__main__':

    torch.manual_seed(22)
    

    model = Model()

    ## GENERATE DATA
    # X, y = make_classification(
    #         n_samples=10_000, 
    #         n_features=2, 
    #         n_redundant=0,
    #         n_clusters_per_class=1, 
    #         weights=[0.98], 
    #         flip_y=0, 
    #         random_state=2,
    #         class_sep=1.1)

    X, y = make_circles(
        n_samples=1000,
        noise=0.05,
        random_state=0,
        shuffle=False,
        factor=0.1
    )

    X1 = X[:,0]
    X2 = X[:,1]

    ## CONVERT DATA TO PANDAS DATAFRAME
    data_array = np.stack((X1,X2,y),axis=1)
    df = pd.DataFrame(data_array,columns=['X1','X2','y'])
    df = df.sort_values(by='y')


    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.25,random_state=33)
    test_df = test_df.sort_values(by='y')


    ## CONVERT DATA INTO TENSORS
    X_train = train_df[['X1','X2']].to_numpy()
    X_test = test_df[['X1','X2']].to_numpy()

    y_train = train_df[['y']].to_numpy()
    y_test = test_df[['y']].to_numpy()

    ## CONVERT DATA ARRAYS TO TENSORS
    X_train = torch.FloatTensor(X_train)
    X_test = torch.FloatTensor(X_test)

    y_train = torch.FloatTensor(y_train)
    y_test = torch.FloatTensor(y_test)
    




    ## TRAIN NN MODEL
    train_model(model,X_train,y_train,X_test,y_test,epochs=500)

    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Classifier_2D.pt')

    # with torch.no_grad():
    #     y_pred = model.forward(X_test)

    # ## FIND OPTIMAL THRESHOLD
    # def to_labels(pos_probs, threshold):
    #     return (pos_probs >= threshold).astype('int')

    # threshold_arr = np.arange(0, 1, 0.001)
    # scores = [f1_score(y_test, to_labels(y_pred.numpy(), threshold)) for threshold in threshold_arr]
    # ix = np.argmax(scores)
    # print(f'Best Threshold: {threshold_arr[ix]:.3f} \tF-Score: {scores[ix]:.3f}')

    ## FIND DECISION CONTOUR
    with torch.no_grad():
        x1_cgrid = np.linspace(-5,5,100)    # x1 contour grid
        x2_cgrid = np.linspace(-5,5,100)    # x2 contour grid

        X1_cgrid,X2_cgrid = np.meshgrid(x1_cgrid,x2_cgrid)
        X_cgrid = np.stack((X1_cgrid.flatten(),X2_cgrid.flatten()),axis=1)
        X_cgrid = torch.FloatTensor(X_cgrid)

        y_contour = model.forward(X_cgrid).reshape(100,100).numpy()


    ## PLOT DATA/NETWORK OUTPUT
    fig = plt.figure()

    ax = fig.add_subplot(111)
    ax.contour(X1_cgrid,X2_cgrid,y_contour,cmap='jet')
    # ax1.contour(X1_test,X2_test,y_contour,levels=[threshold_arr[ix]],cmap='jet')
    ax.scatter(train_df['X1'],train_df['X2'],c=train_df['y'],cmap='jet')
    

    ax.grid()
    ax.set_xlabel('X1')
    ax.set_ylabel('X2')
    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    # ax.set_aspect('equal')
    # ax1.legend()
  

    fig.tight_layout()
    plt.show()
