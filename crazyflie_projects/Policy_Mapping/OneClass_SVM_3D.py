import numpy as np
from sklearn.svm import OneClassSVM
import matplotlib.pyplot as plt

import plotly.graph_objects as go

## LOAD TRAINING DATA
r = np.linspace(0,1,5)
theta = np.linspace(0,2*np.pi,15)
ZZ = np.linspace(-1,1,15)

R,THETA,ZZ = np.meshgrid(r,theta,ZZ)

XX = R*np.cos(THETA)
YY = R*np.sin(THETA)
ZZ = ZZ

X2 = np.stack((XX.flatten(),YY.flatten(),ZZ.flatten()),axis=1)


# Learn a frontier for outlier detection with several classifiers
xx2, yy2, zz2 = np.meshgrid(np.linspace(-2, 2, 20), np.linspace(-2, 2, 20),np.linspace(-2, 2, 20))
clf_name = "OCSVM"
clf = OneClassSVM(nu=0.3, gamma=8)
plt.figure(2)
clf.fit(X2)
Z2 = clf.decision_function(np.c_[xx2.ravel(), yy2.ravel(), zz2.ravel()])
Z2 = Z2.reshape(xx2.shape)
# plt.contour(xx2, yy2, Z2, levels=[0], linewidths=2, colors='k')




fig = go.Figure()

fig.add_trace(
    go.Isosurface(
        x=xx2.flatten(),
        y=yy2.flatten(),
        z=zz2.flatten(),
        value = Z2.flatten(),
        isomin=0.0,
        isomax=0.01,
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
        color=X2[:, 2],                # set color to an array/list of desired values
        colorscale='Viridis',   # choose a colorscale
        opacity=0.1
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