## STANDARD IMPORTS
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

## SKLEARN IMPORTS
from sklearn import svm
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
from sklearn.datasets import make_blobs
from sklearn.metrics import *


import plotly.graph_objects as go


## LOCAL IMPORTS
# from NN_Training import *


os.system("clear")
BASEPATH = "crazyflie_projects/Policy_Mapping/ML_Comparison"


if __name__ == "__main__":

    ## GENERATE DATA
    model_config = "Wide-Long"
    df_raw = pd.read_csv(f"crazyflie_projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")


    X1 = df_raw["OF_y_flip_mean"]
    X2 = df_raw["RREV_flip_mean"]
    X3 = df_raw["flip_d_mean"]
    y = df_raw["landing_rate_4_leg"].to_numpy().reshape(-1,1)
    y = np.where(y < 0.8,0,1)

    ## REGULARIZE DATA
    X = np.stack((X1,X2,X3),axis=1)
    scaler = preprocessing.StandardScaler().fit(X)
    X_scaled = scaler.transform(X)
    X = X_scaled


    data_array = np.hstack((X,y))
    df = pd.DataFrame(data_array,columns=['X1','X2','X3','y'])
    df = df.sort_values(by='y')

    

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.25,random_state=73)
    test_df = test_df.sort_values(by='y')


    ## CONVERT DATA INTO TENSORS
    X_train = train_df[['X1','X2','X3']].to_numpy()
    X_test = test_df[['X1','X2','X3']].to_numpy()

    y_train = train_df[['y']].to_numpy()
    y_test = test_df[['y']].to_numpy()



    # ## PLOT DATA
    # fig = plt.figure(1,figsize=(8,8))

    # ## PLOT MODEL
    # ax1 = fig.add_subplot(111,projection='3d')
    # ax1.scatter(
    #     df['X1'],
    #     df['X2'],
    #     df['X3'],
    #     c = df['y'],
    #     cmap='jet',linewidth=0.2,antialiased=True)
    # ax1.set_xlabel('X1')
    # ax1.set_ylabel('X2')
    # ax1.set_ylabel('X3')
    # ax1.set_title('Function')
    # # ax1.set_xlim(-10,10)
    # # ax1.set_ylim(-10,10)
    # # ax1.set_zlim(-10,10)

    # fig.tight_layout()
    # plt.show()

    ## TRAIN SVM MODEL
    clf = svm.SVC(kernel='rbf',gamma="auto",class_weight={1: 10})
    clf.fit(X_train,y_train[:,0])

    ## EVALUATE SVM MODEL
    y_pred_SVM = clf.predict(X_test)
    print(balanced_accuracy_score(y_test[:,0],y_pred_SVM))
    print(confusion_matrix(y_test[:,0],y_pred_SVM,normalize=None))
    print(classification_report(y_test[:,0],y_pred_SVM))




    

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
    Z = clf.decision_function(np.c_[XX.ravel(), YY.ravel(), ZZ.ravel()])
    Z = Z.reshape(XX.shape)
    # y_pred_grid = model.forward(grid_data)


    fig = go.Figure()

    fig.add_trace(
        go.Isosurface(
            x=XX.flatten(),
            y=YY.flatten(),
            z=ZZ.flatten(),
            value=Z.flatten(),
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


