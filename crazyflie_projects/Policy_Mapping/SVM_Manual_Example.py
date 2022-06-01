import matplotlib.pyplot as plt
from sklearn import svm
from sklearn.datasets import make_blobs
import numpy as np


# # we create 40 separable points
# X, y = make_blobs(n_samples=40, centers=2, random_state=6)


X = np.array([
    [0, 0], 
    [1, 1]])
    
y = [0, 1]
# fit the model, don't regularize for illustration purposes
clf = svm.SVC(kernel="linear")
clf.fit(X, y)

xx = np.linspace(-3,3,100)
yy = np.linspace(-3,3,100)
XX,YY = np.meshgrid(xx,yy)
X_contour = np.stack((XX.flatten(),YY.flatten()),axis=1)
Z = clf.decision_function(X_contour)

X_test = np.array([[0.5,0.5]])

fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(X[:, 0], X[:, 1], c=y, s=30,)

cs = ax.contour(XX,YY,Z.reshape(XX.shape),levels=[-1,0,1])
ax.clabel(cs, inline=True)
# plot support vectors
ax.scatter(
    clf.support_vectors_[:, 0],
    clf.support_vectors_[:, 1],
    s=100,
    linewidth=1,
    facecolors="none",
    edgecolors="k",
)
plt.show()