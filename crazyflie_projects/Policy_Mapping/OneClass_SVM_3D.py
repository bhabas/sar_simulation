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




# ## LOAD TRAINING DATA
# r = np.linspace(0,1,5)
# theta = np.linspace(0,2*np.pi,15)
# ZZ = np.linspace(-1,1,15)

# R,THETA,ZZ = np.meshgrid(r,theta,ZZ)

# XX = R*np.cos(THETA)
# YY = R*np.sin(THETA)
# ZZ = ZZ

# X2 = np.stack((XX.flatten(),YY.flatten(),ZZ.flatten()),axis=1)
scaler = preprocessing.StandardScaler().fit(X2)
X2 = scaler.transform(X2)


# Learn a frontier for outlier detection with several classifiers
xx2, yy2, zz2 = np.meshgrid(np.linspace(-4, 4, 20), np.linspace(-4, 4, 20),np.linspace(-4, 4, 20))
clf = OneClassSVM(nu=0.1,gamma=0.4)
plt.figure(2)
clf.fit(X2)
Z2 = clf.decision_function(np.c_[xx2.ravel(), yy2.ravel(), zz2.ravel()])
Z2 = Z2.reshape(xx2.shape)
# plt.contour(xx2, yy2, Z2, levels=[0], linewidths=2, colors='k')




fig = go.Figure()

fig.add_trace(
    go.Volume(
        x=xx2.flatten(),
        y=yy2.flatten(),
        z=zz2.flatten(),
        value = Z2.flatten(),
        isomin=-0.1,
        isomax=0.1,
        surface_count=3,
        colorscale='Viridis',
        opacity=0.1
        )
)


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
        opacity=0.5
    )
    )
)



fig.update_layout(
    scene=dict(
        xaxis_title='X',
        yaxis_title='Y',
        xaxis_range=[xx2.min(), xx2.max()],
        yaxis_range=[yy2.min(), yy2.max()],
    ),
)
fig.show()

# # Plot the results (= shape of the data points cloud)
# plt.figure(2)  # "banana" shape
# plt.title("Outlier detection on a real data set (wine recognition)")
# plt.scatter(X2[:, 0], X2[:, 1], color="tab:blue")
# plt.xlim((xx2.min(), xx2.max()))
# plt.ylim((yy2.min(), yy2.max()))

# plt.ylabel("color_intensity")
# plt.xlabel("flavanoids")

# plt.show()