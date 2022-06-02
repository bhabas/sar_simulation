import numpy as np
from sklearn import preprocessing
from sklearn.svm import OneClassSVM
import matplotlib.pyplot as plt
import pandas as pd

import plotly.graph_objects as go

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping"


## LOAD DATA
df_raw = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_DR/NL_LR_Trials_DR.csv").dropna() # Collected data
# df_raw = df_raw.query("60 <= phi_IC <= 70")
df_raw = df_raw.query("LR_4leg >= 0.8")


## ORGANIZE DATA
Tau = df_raw["Tau_flip_mean"]
OF_y = df_raw["OFy_flip_mean"]
d_ceil = df_raw["D_ceil_flip_mean"]
landing_rate = df_raw["LR_4leg"]

X_train = np.stack((Tau,OF_y,d_ceil),axis=1)

## SCALE TRAINING DATA
scaler = preprocessing.StandardScaler().fit(X_train)
X_train = scaler.transform(X_train)


## FIT OC_SVM MODEL
clf = OneClassSVM(nu=0.2,gamma=2)
clf.fit(X_train)


## CREATE CONTOUR VOLUME
tmp = np.linspace(-4,4,30)
XX, YY, ZZ = np.meshgrid(tmp,tmp,tmp)
X_grid = np.stack((XX.flatten(), YY.flatten(), ZZ.flatten()),axis=1)
Z = clf.decision_function(X_grid)

## SCALE CONTOUR VOLUME BACK TO NORMAL COORDINATES
X_grid = scaler.inverse_transform(X_grid)


## TEST OUT DECISION FUNCTION
def dec_func(clf,X_test):

    def kernel(x,xi,gamma):
        return np.exp(-gamma*np.sum((x-xi)**2))

    val = 0
    for ii in range(len(clf.support_vectors_)):
        val += clf.dual_coef_[0,ii]*kernel(clf.support_vectors_[ii],X_test,clf._gamma)
    val += clf.intercept_

    return val

X_test = np.array([[0,0,0]])


## PLOT FIGURE
fig = go.Figure()

fig.add_trace(
    go.Volume(
        x=X_grid[:, 0],
        y=X_grid[:, 1],
        z=X_grid[:, 2],
        value = Z.flatten(),
        isomin=-0.1,
        isomax=0.0,
        surface_count=1,
        colorscale='Viridis',
        opacity=0.3
        )
)
X_train = scaler.inverse_transform(X_train)

fig.add_trace(
    go.Scatter3d(
        x=X_train[:, 0],
        y=X_train[:, 1],
        z=X_train[:, 2],

        mode='markers',
        marker=dict(
        size=3,
        color=landing_rate,                # set color to an array/list of desired values+
        cmin=0,
        cmax=1,
        colorscale='Viridis',   # choose a colorscale
        opacity=0.6
    )
    )
)



fig.update_layout(
    scene=dict(
        xaxis_title='Tau [s]',
        yaxis_title='OFy [rad/s]',
        zaxis_title='D_ceil [m]',
        xaxis_range=[0.35,0.2],
        yaxis_range=[0,-20],
        zaxis_range=[0,1.2]
    ),
)
fig.show()
