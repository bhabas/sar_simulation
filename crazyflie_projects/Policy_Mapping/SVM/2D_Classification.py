import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm
from sklearn import preprocessing


## INITIALIZE DATA SET
# If values are different then label=True, else label=False
xx, yy = np.meshgrid(np.linspace(-3, 3, 500), np.linspace(-3, 3, 500))
np.random.seed(0)
X = np.random.randn(300, 2)
Y = np.logical_xor(X[:, 0] > 0, X[:, 1] > 0)
X[:,0] = 2*X[:,0]

## REGULARIZE DATA
scaler = preprocessing.StandardScaler().fit(X)
X_scaled = scaler.transform(X)
X = X_scaled



## TRAIN THE MODEL
clf = svm.NuSVC(gamma="auto")
clf.fit(X, Y)

## PLOT THE DECISION FUNCTION FOR EACH DATAPOINT ON THE GRID
Z = clf.decision_function(np.c_[xx.ravel(), yy.ravel()])
Z = Z.reshape(xx.shape)

## PLOT VALUES
plt.imshow(
    Z,
    interpolation="nearest",
    extent=(-6,6,-6,6),
    aspect="auto",
    origin="lower",
    cmap=plt.cm.viridis,
)
contours = plt.contour(xx, yy, Z, levels=[0], linewidths=2, linestyles="dashed")
plt.scatter(X[:, 0], X[:, 1], s=30, c=Y, cmap=plt.cm.Paired, edgecolors="k")
plt.xticks(())
plt.yticks(())
plt.axis([-6,6,-6,6])
plt.show()