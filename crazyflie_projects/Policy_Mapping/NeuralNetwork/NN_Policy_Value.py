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


import plotly.graph_objects as go

np.set_printoptions(suppress=True)
BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"



## DEFINE NN MODEL
class NN_Policy_Value(nn.Module):
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

def train_model(epochs,X_train,y_train,X_test,y_test):

    ## INITIALIZE NEURAL NETWORK MODEL
    model = NN_Policy_Value()
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

    torch.save(model,f'{BASEPATH}/Pickle_Files/Policy_Network.pt')
    
    ## PLOT LOSSES AND ACCURACIES
    fig = plt.figure(1,figsize=(12,8))
    ax1 = fig.add_subplot(1,1,1)
    ax1.plot(losses_train,label="Training Loss")
    ax1.plot(losses_test,label="Validation Loss")
    ax1.set_ylabel("Loss")
    ax1.set_title("Losses")
    ax1.set_ylim(0,4)
    ax1.legend()
    ax1.grid()


    plt.tight_layout()
    plt.show()
    return model


if __name__ == '__main__':

    ## GENERATE TRAINING DATA
    torch.manual_seed(22)
    np.random.seed(0)


    model_config = "Wide-Long"
    df_raw = pd.read_csv(f"crazyflie_projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    df_raw = df_raw.query("landing_rate_4_leg >= 0.8")


    RREV = df_raw["RREV_flip_mean"]
    OF_y = df_raw["OF_y_flip_mean"]
    d_ceil = df_raw["flip_d_mean"]
    y = df_raw["My_d"].to_numpy().reshape(-1,1)

    ## REGULARIZE DATA
    X = np.stack((RREV,OF_y,d_ceil),axis=1)
    scaler = preprocessing.StandardScaler().fit(X)
    X_scaled = scaler.transform(X)

    ## CONVERT DATA INTO DATAFRAME
    data_array = np.hstack((X_scaled,y))
    df = pd.DataFrame(data_array,columns=['RREV','OF_y','d_ceil','y'])


    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.2,random_state=33)



    ## CONVERT DATA INTO TENSORS
    X_train = torch.FloatTensor(train_df[['RREV','OF_y','d_ceil']].to_numpy())
    X_test = torch.FloatTensor(test_df[['RREV','OF_y','d_ceil']].to_numpy())

    y_train = torch.FloatTensor(train_df[['y']].to_numpy()).reshape((-1,1))
    y_test = torch.FloatTensor(test_df[['y']].to_numpy()).reshape((-1,1))



    ## TRAIN NN MODEL
    epochs = 2_600
    # train_model(epochs,X_train,y_train,X_test,y_test)



    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Pickle_Files/Policy_Network.pt')

    with torch.no_grad():
        y_pred_test = model.forward(X_test.float())
        y_error = (y_pred_test-y_test).numpy()

        rms = mean_squared_error(y_test, y_pred_test, squared=False)
        print(f"RMSE: {rms:.5f} Standard Deviation: {y_error.std():.2f}")

    # ## SAVE ERROR VALUES TO CSV
    # y_pred_df = pd.DataFrame(np.hstack((y_test,y_pred_test,y_error)),columns=['y_test','y_pred_test','y_error'])
    # y_pred_df.to_csv(f'{BASEPATH}/NN_Policy_Value_Errors.csv',index=False,float_format="%.2f")


    # ## PLOT ERROR VARIANCE
    # plt.hist(y_error, bins=30,histtype='stepfilled', color='steelblue')
    # plt.show()
    
    


    ## DEFINE PLOTTING RANGE
    X_plot = np.stack((
        RREV,
        OF_y,
        d_ceil),axis=1)

    with torch.no_grad():
        X_plot = scaler.transform(X_plot)
        X_plot = torch.FloatTensor(X_plot)
        y_pred_plot = model.forward(X_plot.float())

    X_plot = scaler.inverse_transform(X_plot)

    fig = go.Figure()


    fig.add_trace(
        go.Scatter3d(
            x=X_plot[:,0].flatten(),
            y=X_plot[:,1].flatten(),
            z=X_plot[:,2].flatten(),
            mode='markers',
            marker=dict(
                size=2,
                color=y_pred_plot.flatten(),
                colorbar=dict(title="Colorbar"),
                colorscale='jet',
                opacity=0.4)
        ))

    fig.update_layout(
        scene = dict(
            xaxis_title='OF_x',
            yaxis_title='RREV',
            zaxis_title='d_ceiling',
            xaxis = dict(nticks=4, range=[-20,0],),
            yaxis = dict(nticks=4, range=[0,8],),
            zaxis = dict(nticks=4, range=[0,1],),
            ),
        scene_aspectmode='cube'
    
    )

    fig.show()

    with torch.no_grad():
        X = torch.FloatTensor([-1.6974,  0.4014, -1.1264])
        y = model.forward(X)
        ii = 0
        f = open(f'{BASEPATH}/Info/NN_Layers_Policy_{model_config}.h','a')
        f.truncate(0) ## Clears contents of file
        f.write("static char NN_Params_Policy[] = {\n")

        ## EXTEND SCALER ARRAY DIMENSIONS
        scaler_means = scaler.mean_.reshape(-1,1)
        scaler_stds = scaler.scale_.reshape(-1,1)
        
        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,scaler_means,
                    fmt='"%.4f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_means.shape[0]},"\t"{scaler_means.shape[1]},"',
                    footer='"*"\n')

        np.savetxt(f,scaler_stds,
                    fmt='"%.4f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_stds.shape[0]},"\t"{scaler_stds.shape[1]},"',
                    footer='"*"\n')

        ## SAVE NN LAYER VALUES
        for name, layer in model.named_modules():
            if ii > 0: # Skip initialization layer

                W = layer.weight.numpy()
                np.savetxt(f,W,
                    fmt='"%.4f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{W.shape[0]},"\t"{W.shape[1]},"',
                    footer='"*"\n')


                b = layer.bias.numpy().reshape(-1,1)
                np.savetxt(f,b,
                    fmt='"%.4f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{b.shape[0]},"\t"{b.shape[1]},"',
                    footer='"*"\n')

            ii+=1

        f.write("};")
        f.close()

        print(y)