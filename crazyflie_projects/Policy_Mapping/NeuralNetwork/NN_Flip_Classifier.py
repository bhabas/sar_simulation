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
        self.fc1 = nn.Linear(in_features,h)     # Layer 1 
        self.fc2 = nn.Linear(h,h)               # Layer 2
        self.out = nn.Linear(h,out_features)    # Layer 3

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.elu(self.fc1(x))
        x = F.elu(self.fc2(x))
        x = torch.sigmoid(self.out(x))

        return x

def train_model(epochs,X_train,y_train,X_test,y_test):
    model = NN_Flip_Classifier()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.01)

    class_weight = [0.1, 0.5] # Weights as binary classes [0,1]
    
    ## DEFINE TRAINING LOSS
    weights = np.where(y_train==1,class_weight[1],class_weight[0])      # Convert class weights to element weights
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
    torch.manual_seed(0)
    np.random.seed(0)

    model_config = "Wide-Long"
    df_raw = pd.read_csv(f"crazyflie_projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    df_bounds = pd.read_csv(f"crazyflie_projects/Policy_Mapping/NeuralNetwork/dataBounds.csv")
    df_raw = pd.concat([df_raw,df_bounds])


    RREV = df_raw["RREV_flip_mean"]
    OF_y = df_raw["OF_y_flip_mean"]
    d_ceil = df_raw["flip_d_mean"]
    y = df_raw["landing_rate_4_leg"].to_numpy().reshape(-1,1)
    y = np.where(y < 0.8,0,1)

    ## REGULARIZE DATA
    X = np.stack((RREV,OF_y,d_ceil),axis=1)
    scaler = preprocessing.StandardScaler().fit(X)
    X_scaled = scaler.transform(X)

    data_array = np.hstack((X_scaled,y))
    df = pd.DataFrame(data_array,columns=['RREV','OF_y','d_ceil','y'])
    df = df.sort_values(by='y')


    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.25,random_state=73)
    test_df = test_df.sort_values(by='y')



    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train_df[['RREV','OF_y','d_ceil']].to_numpy())
    X_test = torch.FloatTensor(test_df[['RREV','OF_y','d_ceil']].to_numpy())

    y_train = torch.FloatTensor(train_df[['y']].to_numpy())
    y_test = torch.FloatTensor(test_df[['y']].to_numpy())



    ## TRAIN NN MODEL
    epochs = 500
    # train_model(epochs,X_train,y_train[:,0].reshape(-1,1),X_test,y_test[:,0].reshape(-1,1))



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
    # y_pred_df = pd.DataFrame(np.hstack((y_test,y_pred_test_class,y_error)),columns=['y_test','y_pred_test','y_error'])
    # y_pred_df.to_csv(f'{BASEPATH}/NN_Flip_Classifier_Errors.csv',index=False,float_format="%.2f")

    # ## PLOT ERROR VARIANCE
    # plt.hist(y_error, bins=30,histtype='stepfilled', color='steelblue')
    # plt.show()


    ## PASS DATA TO PREDICT METHOD
    with torch.no_grad():
        test_data = np.array([1.322,-0.497,0.0]).reshape(1,-1)
        grid_data = scaler.transform(test_data)
        test_data = torch.FloatTensor(test_data)
        print(model.forward(test_data))

    ## PLOT DECISION BOUNDARY


    ## CREATE GRID
    RREV_grid, OF_y_grid, d_ceil_grid = np.meshgrid(
        np.linspace(RREV.min(), RREV.max(), 30),
        np.linspace(OF_y.min(), OF_y.max(), 30),
        np.linspace(d_ceil.min(), d_ceil.max(), 30)
    )

    ## CONCATENATE DATA TO MATCH INPUT
    grid_data = np.hstack((
        RREV_grid.ravel().reshape(-1,1), 
        OF_y_grid.ravel().reshape(-1,1),
        d_ceil_grid.ravel().reshape(-1,1)))

    ## PASS DATA TO PREDICT METHOD
    with torch.no_grad():
        grid_data = scaler.transform(grid_data)
        grid_data = torch.FloatTensor(grid_data)
        y_pred_grid = model.forward(grid_data)

    grid_data = scaler.inverse_transform(grid_data)


    fig = go.Figure()

    fig.add_trace(
        go.Volume(
        # go.Isosurface(
            x=grid_data[:,1].flatten(),
            y=grid_data[:,0].flatten(),
            z=grid_data[:,2].flatten(),
            value=y_pred_grid.flatten(),
            surface_count=10,
            opacity=0.1,
            isomin=0.85,
            isomax=0.90,
            colorscale='Viridis',  
            cmin=0,
            cmax=1,     
            caps=dict(x_show=False, y_show=False)
        ))



    fig.add_trace(
        go.Scatter3d(
            x=df_raw["OF_y_flip_mean"],
            y=df_raw["RREV_flip_mean"],
            z=df_raw["flip_d_mean"],
            mode='markers',
            marker=dict(
                size=3,
                color=df_raw["landing_rate_4_leg"],                # set color to an array/list of desired values
                colorscale='Viridis',   # choose a colorscale
                opacity=1.0)
        ))
    h_c = 2.1   
    z_0 = 0.4

    V = 2.0
    theta = np.array([90,70,50,40]).reshape(1,-1)
    vz = V*np.sin(np.radians(theta))
    vx = V*np.cos(np.radians(theta))

    t_max = (h_c-z_0)/vz
    t = np.linspace(0.0,2,50).reshape(-1,1)
    d_t = h_c - (z_0 + vz*t)
    OFy_t = -vx/(d_t)
    RREV_t = vz/(d_t)



    fig.update_layout(
    scene = dict(
        xaxis_title = 'OF_y',
        yaxis_title = 'RREV',
        zaxis_title = 'D_ceiling',
        xaxis = dict(range=[-15,0]),
        yaxis = dict(range=[0,8]),
        zaxis = dict(range=[0,2])
        ))


    fig.show()

    with torch.no_grad():
        X = torch.FloatTensor([-1.6974,  0.4014, -1.1264])
        y = model.forward(X)
        ii = 0
        f = open(f'{BASEPATH}/Info/NN_Layers_Flip_{model_config}.data','ab')
        f.truncate(0) ## Clears contents of file

        ## EXTEND SCALER ARRAY DIMENSIONS
        scaler_means = scaler.mean_.reshape(-1,1)
        scaler_stds = scaler.scale_.reshape(-1,1)
        
        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,scaler_means,
                    fmt='%.6f',
                    delimiter='\t',
                    comments='',
                    header=f"{scaler_means.shape[0]} {scaler_means.shape[1]}",
                    footer="\n")

        np.savetxt(f,scaler_stds,
                    fmt='%.6f',
                    delimiter='\t',
                    comments='',
                    header=f"{scaler_means.shape[0]} {scaler_means.shape[1]}",
                    footer="\n")

        ## SAVE NN LAYER VALUES
        for name, layer in model.named_modules():
            if ii > 0: # Skip initialization layer

                W = layer.weight.numpy()
                np.savetxt(f,W,
                    fmt='%.6f',
                    delimiter='\t',
                    comments='',
                    header=f"{W.shape[0]} {W.shape[1]}",
                    footer="\n")


                b = layer.bias.numpy().reshape(-1,1)
                np.savetxt(f,b,
                    fmt='%.6f',
                    delimiter='\t',
                    comments='',
                    header=f"{b.shape[0]} {b.shape[1]}",
                    footer="\n")

            ii+=1

        f.close()

        print(y)



