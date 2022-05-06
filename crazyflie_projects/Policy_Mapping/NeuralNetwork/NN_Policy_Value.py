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
from sklearn.metrics import *
from sklearn import preprocessing
from imblearn.over_sampling import RandomOverSampler

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
        self.out = nn.Linear(h,out_features)    # Layer 3

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = torch.sigmoid(self.fc1(x))
        x = torch.sigmoid(self.fc2(x))
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
        fig.add_trace(
            go.Scatter3d(
                ## DATA
                x=X_grid[:,1].flatten(),
                y=X_grid[:,0].flatten(),
                z=X_grid[:,2].flatten(),

                ## HOVER DATA
                # customdata=df_custom,
                # hovertemplate=" \
                #     <b>LR: %{customdata[3]:.3f}</b> \
                #     <br>OFy: %{customdata[10]:.3f} Vel: %{customdata[0]:.2f} </br> \
                #     <br>Tau: %{customdata[8]:.3f} Phi: %{customdata[1]:.0f}</br> \
                #     <br>D_ceil: %{customdata[12]:.3f}</br>",

                ## MARKER
                mode='markers',
                marker=dict(
                    size=3,
                    color=y_pred_grid,                # set color to an array/list of desired values
                    cmin=4,
                    cmax=9,
                    colorscale='Viridis',   # choose a colorscale
                    opacity=1.0
                )
            )
        )

        X_grid = np.stack((
            df_custom["Tau"],
            df_custom["OFy"],
            df_custom["d_ceil"]),axis=1)
        y_pred = self.modelPredict(X_grid).numpy().flatten()
        error = df_custom["My_d"].to_numpy().flatten() - y_pred
    
        # ## PLOT DATA POINTS
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
        #             color=y_pred,                # set color to an array/list of desired values
        #             cmin=0,
        #             cmax=10,
        #             colorscale='Viridis',   # choose a colorscale
        #             opacity=1.0
        #         )
        #     )
        # )

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
    df = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_Raw/NL_PolicyVal_Trials_Raw_1.csv").sample(frac=0.05) # Collected data

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
    Policy_NN.createScaler(X)
    Policy_NN.trainModel(X_train,y_train,X_test,y_test,epochs=10000)
    Policy_NN.saveParams(Param_Path)

    Policy_NN.loadModelFromParams(Param_Path)
    # Policy_NN.evalModel(X,y)
    Policy_NN.plotModel(df)


    # ## EVALUATE NN MODEL
    # model = torch.load(f'{BASEPATH}/Pickle_Files/Policy_Network.pt')

    # with torch.no_grad():
    #     y_pred_test = model.forward(X_test.float())
    #     y_error = (y_pred_test-y_test).numpy()

    #     rms = mean_squared_error(y_test, y_pred_test, squared=False)
    #     print(f"RMSE: {rms:.5f} Standard Deviation: {y_error.std():.2f}")

    # # ## SAVE ERROR VALUES TO CSV
    # # y_pred_df = pd.DataFrame(np.hstack((y_test,y_pred_test,y_error)),columns=['y_test','y_pred_test','y_error'])
    # # y_pred_df.to_csv(f'{BASEPATH}/NN_Policy_Value_Errors.csv',index=False,float_format="%.2f")


    # # ## PLOT ERROR VARIANCE
    # # plt.hist(y_error, bins=30,histtype='stepfilled', color='steelblue')
    # # plt.show()
    
    


    # ## DEFINE PLOTTING RANGE
    # X_plot = np.stack((
    #     RREV,
    #     OF_y,
    #     d_ceil),axis=1)

    # with torch.no_grad():
    #     X_plot = scaler.transform(X_plot)
    #     X_plot = torch.FloatTensor(X_plot)
    #     y_pred_plot = model.forward(X_plot.float())

    # X_plot = scaler.inverse_transform(X_plot)

    # fig = go.Figure()


    # fig.add_trace(
    #     go.Scatter3d(
    #         x=X_plot[:,0].flatten(),
    #         y=X_plot[:,1].flatten(),
    #         z=X_plot[:,2].flatten(),
    #         mode='markers',
    #         marker=dict(
    #             size=2,
    #             color=y_pred_plot.flatten(),
    #             colorbar=dict(title="Colorbar"),
    #             colorscale='jet',
    #             opacity=0.4)
    #     ))

    # fig.update_layout(
    #     scene = dict(
    #         xaxis_title='OF_x',
    #         yaxis_title='RREV',
    #         zaxis_title='d_ceiling',
    #         xaxis = dict(nticks=4, range=[-20,0],),
    #         yaxis = dict(nticks=4, range=[0,8],),
    #         zaxis = dict(nticks=4, range=[0,1],),
    #         ),
    #     scene_aspectmode='cube'
    
    # )

    # fig.show()
