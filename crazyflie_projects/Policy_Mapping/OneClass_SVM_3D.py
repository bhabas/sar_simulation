import numpy as np
from sklearn import preprocessing
from sklearn.svm import OneClassSVM
import matplotlib.pyplot as plt
import pandas as pd

import plotly.graph_objects as go

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping"



# np.random.seed(0)


## LOAD DATA
df_raw = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_DR/NL_LR_Trials_DR.csv").dropna() # Collected data
# df_raw = df_raw.query("60 <= phi_IC <= 70")
df_raw = df_raw.query("LR_4leg >= 0.8")


## ORGANIZE DATA
Tau = df_raw["Tau_flip_mean"]
OF_y = df_raw["OFy_flip_mean"]
d_ceil = df_raw["D_ceil_flip_mean"]
landing_rate = df_raw["LR_4leg"]

X2 = np.stack((Tau,OF_y,d_ceil),axis=1)
scaler = preprocessing.StandardScaler().fit(X2)
X2 = scaler.transform(X2)

clf = OneClassSVM(nu=0.2,gamma=2)
clf.fit(X2)

# Learn a frontier for outlier detection with several classifiers
xx2, yy2, zz2 = np.meshgrid(np.linspace(-4, 4, 30), np.linspace(-4, 4, 30),np.linspace(-4, 4, 30))
X_grid = np.stack((xx2.flatten(), yy2.flatten(), zz2.flatten()),axis=1)

Z2 = clf.decision_function(X_grid)
X_grid = scaler.inverse_transform(X_grid)


fig = go.Figure()

fig.add_trace(
    go.Volume(
        x=X_grid[:, 0],
        y=X_grid[:, 1],
        z=X_grid[:, 2],
        value = Z2.flatten(),
        isomin=-0.1,
        isomax=0.0,
        surface_count=1,
        colorscale='Viridis',
        opacity=0.3
        )
)
X2 = scaler.inverse_transform(X2)

fig.add_trace(
    go.Scatter3d(
        x=X2[:, 0],
        y=X2[:, 1],
        z=X2[:, 2],

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
