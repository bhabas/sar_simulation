## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

## SKLEARN IMPORTS
from sklearn.model_selection import train_test_split
from sklearn.metrics import *
from sklearn import preprocessing
from sklearn.svm import OneClassSVM


import plotly.graph_objects as go

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping"



if __name__ == "__main__":

    ## SET SEEDS
    np.random.seed(0)


    ## LOAD TRAINING DATA
    r = np.linspace(0,1,5)
    theta = np.linspace(0,2*np.pi,15)
    phi = np.linspace(0,2*np.pi,15)

    R,THETA,PHI = np.meshgrid(r,theta,phi)

    XX = R*np.cos(PHI)*np.cos(THETA)
    YY = R*np.cos(PHI)*np.sin(THETA)
    ZZ = R*np.sin(PHI)

    X_train = np.stack((XX.flatten(),YY.flatten(),ZZ.flatten()),axis=1)

    clf = OneClassSVM(nu=0.8,kernel='rbf')
    clf.fit(X_train)

    ## LOAD CONTOUR DATA
    r = np.linspace(0,3,20)
    theta = np.linspace(0,2*np.pi,20)
    phi = np.linspace(0,2*np.pi,20)

    R_c,THETA_c,PHI_c = np.meshgrid(r,theta,phi)

    XX_c = R_c*np.cos(PHI_c)*np.cos(THETA_c)
    YY_c = R_c*np.cos(PHI_c)*np.sin(THETA_c)
    ZZ_c = R_c*np.sin(PHI_c)


    Z_contour = clf.decision_function(np.c_[XX_c.ravel(), YY_c.ravel(),ZZ_c.ravel()])
    Z_contour = Z_contour.reshape(XX_c.shape)



    fig = go.Figure()

    ## PLOT DATA POINTS
    fig.add_trace(
        go.Scatter3d(
            ## DATA
            x=XX.flatten(),
            y=YY.flatten(),
            z=ZZ.flatten(),


            ## MARKER
            mode='markers',
            marker=dict(
                size=3,
                colorscale='Viridis',   # choose a colorscale
                opacity=0.4)
        )
    )

    fig.add_trace(
        go.Isosurface(
            ## DATA
            x=XX_c.flatten(),
            y=YY_c.flatten(),
            z=ZZ_c.flatten(),
            value = Z_contour.flatten(),
            isomin=-0.5,
            isomax=0.5
        )
    )

    fig.update_layout(
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z',
            xaxis_range=[-2,2],
            yaxis_range=[-2,2],
            zaxis_range=[-2,2],
        ),
    )
    fig.show()