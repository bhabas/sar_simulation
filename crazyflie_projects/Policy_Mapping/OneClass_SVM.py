import numpy as np
from sklearn.svm import OneClassSVM
import matplotlib.pyplot as plt
import matplotlib.font_manager
from sklearn.datasets import load_wine

import plotly.graph_objects as go

# # Get data
# X2 = load_wine()["data"][:, [6, 9]]  # "banana"-shaped
# X2[:,1] = (X2[:,1]-7)/5

## LOAD TRAINING DATA
r = np.linspace(0,1,5)
theta = np.linspace(0,2*np.pi,15)

R,THETA = np.meshgrid(r,theta)

XX = R*np.cos(THETA)
YY = R*np.sin(THETA)

X2 = np.stack((XX.flatten(),YY.flatten()),axis=1)


# Learn a frontier for outlier detection with several classifiers
xx2, yy2 = np.meshgrid(np.linspace(-2, 2, 20), np.linspace(-2, 2, 20))
clf_name = "OCSVM"
clf = OneClassSVM(nu=0.1, gamma=2)
plt.figure(2)
clf.fit(X2)
Z2 = clf.decision_function(np.c_[xx2.ravel(), yy2.ravel()])
Z2 = Z2.reshape(xx2.shape)
plt.contour(xx2, yy2, Z2, levels=[0], linewidths=2, colors='k')




fig = go.Figure()

fig.add_trace(
    go.Contour(
        x=xx2.flatten(),
        y=yy2.flatten(),
        z=Z2.flatten(),
        line_width=2,
        line_smoothing=0,
        contours=dict(
        start=0,
        end=0,
        size=0.5,
        showlabels = True, # show labels on contours
        )
    )
)

fig.add_trace(
    go.Scatter(
        x=X2[:, 0],
        y=X2[:, 1],
        mode='markers'
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

# Plot the results (= shape of the data points cloud)
plt.figure(2)  # "banana" shape
plt.title("Outlier detection on a real data set (wine recognition)")
plt.scatter(X2[:, 0], X2[:, 1], color="tab:blue")
plt.xlim((xx2.min(), xx2.max()))
plt.ylim((yy2.min(), yy2.max()))
plt.legend(
    loc="upper center",
    prop=matplotlib.font_manager.FontProperties(size=11),
)
plt.ylabel("color_intensity")
plt.xlabel("flavanoids")

plt.show()