## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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

import plotly.graph_objects as go


np.set_printoptions(suppress=True)
BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"

## DEFINE NN MODEL
class NN_Flip_Classifier(nn.Module):
    def __init__(self,in_features=3,h=10,out_features=1):
        super().__init__()
        self.fc1 = nn.Linear(in_features,h) # Fully connected layer
        self.fc2 = nn.Linear(h,h)
        self.out = nn.Linear(h,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = torch.sigmoid(self.fc1(x))
        x = torch.sigmoid(self.fc2(x))
        x = torch.sigmoid(self.out(x))

        return x

def train_model(epochs,X_train,y_train,X_test,y_test):
    model = NN_Flip_Classifier()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.01)

    class_weight = [0.1, 0.9]                                       # Weights as binary classes [0,1]
    
    ## DEFINE TRAINING LOSS
    weights = np.where(y_train==1,class_weight[1],class_weight[0])  # Convert class weights to element weights
    criterion = nn.BCELoss(weight=torch.FloatTensor(weights))       
    losses_train = []
    accuracies_train = []

    ## DEFINE VALIDATION LOSS
    weights_val = np.where(y_test==1,class_weight[1],class_weight[0])   # Convert class weights to element weights
    criterion_val = nn.BCELoss(weight=torch.FloatTensor(weights_val))   
    losses_test = []
    accuracies_test = []


    for ii in range(epochs):

        ## MODEL PREDICTION
        y_pred_train = model.forward(X_train)

        ## CALC TRAINING LOSS
        loss_train = criterion(y_pred_train,y_train)
        losses_train.append(loss_train.item())

        ## CALC TRAINING ACCURACY
        pred_cutoff = 0.5 
        y_pred_train_class = np.where(y_pred_train.detach().numpy() < pred_cutoff,0,1)
        accuracy_train = balanced_accuracy_score(y_train[:,0],y_pred_train_class)
        accuracies_train.append(accuracy_train)

        
        ## CALC VALIDATION LOSS
        with torch.no_grad():
            y_pred_test = model.forward(X_test)

        loss_test = criterion_val(y_pred_test,y_test)
        losses_test.append(loss_test.item())

        ## CALC TESTING ACCURACY
        y_pred_test_class = np.where(y_pred_test.detach().numpy() < pred_cutoff,0,1)
        accuracy = balanced_accuracy_score(y_test[:,0],y_pred_test_class)
        accuracies_test.append(accuracy)

        if ii%10 == 1:
            print(f"epoch {ii} and loss is: {loss_train}")

        ## BACKPROPAGATION
        optimizer.zero_grad()
        loss_train.backward()
        optimizer.step()

    torch.save(model,f'{BASEPATH}/Pickle_Files/Flip_Network.pt')

    ## PLOT LOSSES AND ACCURACIES
    fig = plt.figure(1,figsize=(12,8))
    ax1 = fig.add_subplot(2,1,1)
    ax1.plot(losses_train,label="Training Loss")
    ax1.plot(losses_test,label="Validation Loss")
    ax1.set_ylabel("Loss")
    ax1.set_title("Losses")
    ax1.set_ylim(0)
    ax1.legend()
    ax1.grid()

    ax2 = fig.add_subplot(2,1,2)
    ax2.plot(accuracies_train,label="Training Accuracry")
    ax2.plot(accuracies_test,label="Validation Accuracy")
    ax2.set_ylim(0,1.1)
    ax2.set_ylabel("Classification Accuracy")
    ax2.set_title("Balanced Accuracy")
    ax2.grid()
    ax2.legend(loc="lower right")


    plt.tight_layout()
    plt.show()

    return model



if __name__ == "__main__":

    ## GENERATE DATA
    
    model_config = "Wide-Long"
    df_raw = pd.read_csv(f"crazyflie_projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")


    X1 = df_raw["OF_y_flip_mean"]
    X2 = df_raw["RREV_flip_mean"]
    X3 = df_raw["flip_d_mean"]
    y = df_raw["landing_rate_4_leg"].to_numpy().reshape(-1,1)
    y = np.where(y < 0.9,0,1)

    ## REGULARIZE DATA
    X = np.stack((X1,X2,X3),axis=1)
    scaler = preprocessing.StandardScaler().fit(X)
    X_scaled = scaler.transform(X)
    X = X_scaled



    ## SAVE SCALING DATA
    df_scale = pd.DataFrame(
        np.vstack((scaler.mean_,scaler.scale_,scaler.var_)).T,
        columns=['mean','scale','var'])
    df_scale.to_csv(f"{BASEPATH}/Info/Scaler_Classifier.csv",index=False,float_format="%.2f")

    data_array = np.hstack((X,y))
    df = pd.DataFrame(data_array,columns=['X1','X2','X3','y'])
    df = df.sort_values(by='y')

    

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.25,random_state=73)
    test_df = test_df.sort_values(by='y')



    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train_df[['X1','X2','X3']].to_numpy())
    X_test = torch.FloatTensor(test_df[['X1','X2','X3']].to_numpy())

    y_train = torch.FloatTensor(train_df[['y']].to_numpy())
    y_test = torch.FloatTensor(test_df[['y']].to_numpy())



    ## TRAIN NN MODEL
    epochs = 2000

    torch.manual_seed(0)
    train_model(epochs,X_train,y_train[:,0].reshape(-1,1),X_test,y_test[:,0].reshape(-1,1))



    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Pickle_Files/Flip_Network.pt')

    with torch.no_grad():
        y_pred_test = model.forward(X_test)
        y_pred_test_class = np.where(y_pred_test.detach().numpy() < 0.5,0,1)

        y_error = (y_pred_test - y_test).numpy()

    print(balanced_accuracy_score(y_test[:,0],y_pred_test_class))
    print(confusion_matrix(y_test[:,0],y_pred_test_class,normalize=None))
    print(classification_report(y_test[:,0],y_pred_test_class))



    ## SAVE ERROR VALUES TO CSV
    y_pred_df = pd.DataFrame(np.hstack((y_test,y_pred_test_class,y_error)),columns=['y_test','y_pred_test','y_error'])
    y_pred_df.to_csv(f'{BASEPATH}/Info/NN_Flip_Classifier_Errors.csv',index=False,float_format="%.2f")

    # ## PLOT ERROR VARIANCE
    # plt.hist(y_error, bins=30,histtype='stepfilled', color='steelblue')
    # plt.show()




    ## PLOT DECISION BOUNDARY
    # DETERMINE GRID RANGE IN X AND Y DIRECTIONS
    x_min, x_max = X[:, 0].min()-0.1, X[:, 0].max()+0.1
    y_min, y_max = X[:, 1].min()-0.1, X[:, 1].max()+0.1
    z_min, z_max = X[:, 2].min()-0.1, X[:, 2].max()+0.1


    ## SET GRID SPACING PARAMETER
    spacing = min(x_max-x_min, y_max-y_min, z_max-z_min) / 25

    ## CREATE GRID
    XX, YY, ZZ = np.meshgrid(
            np.linspace(x_min, x_max, 30),
            np.linspace(y_min, y_max, 30),
            np.linspace(z_min, z_max, 30))

    ## CONCATENATE DATA TO MATCH INPUT
    grid_data = np.hstack((
        XX.ravel().reshape(-1,1), 
        YY.ravel().reshape(-1,1),
        ZZ.ravel().reshape(-1,1)))

    
    ## PASS DATA TO PREDICT METHOD
    with torch.no_grad():
        grid_data = torch.FloatTensor(grid_data)
        y_pred_grid = model.forward(grid_data)


    fig = go.Figure()

    fig.add_trace(
        go.Isosurface(
            x=XX.flatten(),
            y=YY.flatten(),
            z=ZZ.flatten(),
            value=y_pred_grid.flatten(),
            surface_count=1,
            opacity=0.4,
            isomin=0.9,
            isomax=0.9,            
            caps=dict(x_show=False, y_show=False)
        ))



    fig.add_trace(
        go.Scatter3d(
            x=df['X1'],
            y=df['X2'],
            z=df['X3'],
            mode='markers',
            marker=dict(
                size=3,
                color=df['y'],                # set color to an array/list of desired values
                colorscale='Viridis',   # choose a colorscale
                opacity=0.4)
        ))

    fig.update_layout(scene = dict(
                    xaxis_title='OF_x',
                    yaxis_title='RREV',
                    zaxis_title='D_ceiling'),
                    )


    fig.show()



