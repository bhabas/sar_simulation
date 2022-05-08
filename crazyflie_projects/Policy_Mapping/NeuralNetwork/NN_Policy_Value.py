## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os

## PYTORCH IMPROTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## SKLEARN IMPORTS
from sklearn.model_selection import train_test_split
from sklearn.metrics import *

import plotly.graph_objects as go

from NN_Flip_Classifier import NN_Trainer

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping"




## DEFINE NN MODEL
class NN_Policy_Model(nn.Module):
    def __init__(self,in_features=3,h=10,out_features=1):
        super().__init__()

        self.fc1 = nn.Linear(in_features,h)     # Layer 1
        self.fc2 = nn.Linear(h,h)               # Layer 2
        self.fc3 = nn.Linear(h,h)               # Layer 2
        self.out = nn.Linear(h,out_features)    # Layer 3

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.elu(self.fc1(x))
        x = F.elu(self.fc2(x))
        x = F.elu(self.fc2(x))
        x = self.out(x)

        return x

class PolicyNetwork(NN_Trainer):
    def __init__(self,model,model_initials):
        super().__init__(model,model_initials,1)

        self.model = model
        self.model_initials = model_initials

    def trainModel(self, X_train, y_train, X_test, y_test, epochs=500):

        ## CONVERT DATA ARRAYS TO TENSORS
        X_train = torch.FloatTensor(self.scaleData(X_train))
        X_test = torch.FloatTensor(self.scaleData(X_test))

        y_train = torch.FloatTensor(y_train)
        y_test = torch.FloatTensor(y_test)

        ## INIT MODEL AND OPTIMIZER
        optimizer = torch.optim.Adam(self.model.parameters(), lr=0.01)
    
        ## DEFINE TRAINING LOSS
        criterion_train = nn.MSELoss(reduction='mean')
        losses_train = []

        ## DEFINE VALIDATION LOSS
        criterion_test = nn.MSELoss(reduction='mean')
        losses_test = []

        for ii in range(epochs):

            ## CALC TRAINING LOSS
            y_pred_train = self.model.forward(X_train)
            loss_train = criterion_train(y_pred_train,y_train)
            losses_train.append(loss_train.item())

            ## CALC VALIDATION LOSS
            with torch.no_grad():
                y_pred_test = self.model.forward(X_test)
                loss_test = criterion_test(y_pred_test,y_test)
                losses_test.append(loss_test.item())

            if ii%10 == 1:
                print(f"epoch {ii} and loss is: {loss_train}")

            ## BACKPROPAGATION
            optimizer.zero_grad()
            loss_train.backward()
            optimizer.step()

    
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

    def plotModel(self,df_custom):

        ## CREATE GRID
        Tau_grid, OF_y_grid, d_ceil_grid = np.meshgrid(
            np.linspace(0.15, 0.35, 10),
            np.linspace(-10, 1, 10),
            np.linspace(0.25, 1.0, 10)
        )

        ## CONCATENATE DATA TO MATCH INPUT
        X_grid = np.stack((
            Tau_grid.flatten(),
            OF_y_grid.flatten(),
            d_ceil_grid.flatten()),axis=1)

        y_pred_grid = self.modelPredict(X_grid).numpy().flatten()

        fig = go.Figure()

        ## PLOT DATA POINTS
        # fig.add_trace(
        #     go.Scatter3d(
        #         ## DATA
        #         x=X_grid[:,1].flatten(),
        #         y=X_grid[:,0].flatten(),
        #         z=X_grid[:,2].flatten(),

        #         ## HOVER DATA
        #         # customdata=df_custom,
        #         # hovertemplate=" \
        #         #     <b>LR: %{customdata[3]:.3f}</b> \
        #         #     <br>OFy: %{customdata[10]:.3f} Vel: %{customdata[0]:.2f} </br> \
        #         #     <br>Tau: %{customdata[8]:.3f} Phi: %{customdata[1]:.0f}</br> \
        #         #     <br>D_ceil: %{customdata[12]:.3f}</br>",

        #         ## MARKER
        #         mode='markers',
        #         marker=dict(
        #             size=3,
        #             color=y_pred_grid,                # set color to an array/list of desired values
        #             cmin=4,
        #             cmax=9,
        #             colorscale='Viridis',   # choose a colorscale
        #             opacity=1.0
        #         )
        #     )
        # )

        X_grid = np.stack((
            df_custom["Tau"],
            df_custom["OFy"],
            df_custom["d_ceil"]),axis=1)
        y_pred = self.modelPredict(X_grid).numpy().flatten()
        error = df_custom["My_d"].to_numpy().flatten() - y_pred
    
        ## PLOT DATA POINTS
        fig.add_trace(
            go.Scatter3d(
                ## DATA
                x=X_grid[:,1].flatten(),
                y=X_grid[:,0].flatten(),
                z=X_grid[:,2].flatten(),

                ## HOVER DATA
                customdata=np.stack((error,df_custom["My_d"].to_numpy().flatten(),y_pred),axis=1),
                hovertemplate=
                "<br>My: %{customdata[1]:.3f}</br> \
                 <br>Error: %{customdata[0]:.3f}</br> \
                 <br>Pred: %{customdata[2]:.3f}</br>",
                    

                ## MARKER
                mode='markers',
                marker=dict(
                    size=3,
                    color=df_custom["My_d"].to_numpy().flatten(),
                    # color=y_pred,
                    # color = np.abs(error),
                    cmin=0,
                    cmax=10,
                    colorscale='Viridis',   # choose a colorscale
                    opacity=1.0
                )
            )
        )

        fig.update_layout(
            scene=dict(
                xaxis_title='OFy [rad/s]',
                yaxis_title='Tau [s]',
                zaxis_title='D_ceiling [m]',
                xaxis_range=[-20,1],
                yaxis_range=[0.4,0.1],
                zaxis_range=[0,1.2],
            ),
        )
        fig.show()


if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    torch.manual_seed(0)
    np.random.seed(0)

    model_initials = "NL_Raw"
    model = NN_Policy_Model()
    Policy_NN = PolicyNetwork(model,model_initials)

    ## LOAD DATA 
    # df = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_Raw/NL_PolicyVal_Trials_Raw_1.csv").sample(frac=0.05) # Collected data


    path = f"{BASEPATH}/Data_Logs/NL_Raw/"
    all_files = []
    for file in os.listdir(f"{BASEPATH}/Data_Logs/NL_Raw/"):
        if file.startswith("NL_PolicyVal"):
            all_files.append(os.path.join(path, file))

    df = pd.concat((pd.read_csv(f).sample(frac=0.02) for f in all_files))

    ## ORGANIZE DATA
    Tau = df["Tau"]
    OF_y = df["OF_y"]
    d_ceil = df["d_ceil"]
    X = np.stack((Tau,OF_y,d_ceil),axis=1)
    y = df["My_d"]

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING SETS
    data_array = np.stack((Tau,OF_y,d_ceil,y),axis=1)
    df = pd.DataFrame(data_array,columns=['Tau','OFy','d_ceil','My_d'])

    train_df, test_df = train_test_split(df,test_size=0.25,random_state=73)
    X_train = train_df[['Tau','OFy','d_ceil']].to_numpy()
    y_train = train_df[['My_d']].to_numpy()

    X_test = test_df[['Tau','OFy','d_ceil']].to_numpy()
    y_test = test_df[['My_d']].to_numpy()

    Param_Path = f'{BASEPATH}/NeuralNetwork/Info/NN_Layers_Policy_{model_initials}.h'
    # Policy_NN.createScaler(X)
    # Policy_NN.trainModel(X_train,y_train,X_test,y_test,epochs=400)
    # Policy_NN.saveParams(Param_Path)

    Policy_NN.loadModelFromParams(Param_Path)
    # Policy_NN.evalModel(X,y)
    Policy_NN.plotModel(df)


