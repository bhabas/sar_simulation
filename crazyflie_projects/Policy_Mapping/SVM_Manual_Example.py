import matplotlib.pyplot as plt
from sklearn import svm
from sklearn.datasets import make_blobs
import numpy as np


# # we create 40 separable points
X, y = make_blobs(n_samples=40, centers=2, random_state=6)

def kernel(x,xi,gamma):
    return np.exp(-gamma*np.linalg.norm(x-xi)**2)
    # np.sum(np.power((actual_value-predicted_value),2))

def dec_func(clf,X_test):
    val = 0
    for ii in range(len(clf.support_vectors_)):
        val += clf.dual_coef_[0,ii]*kernel(clf.support_vectors_[ii],X_test,clf._gamma)
    val += clf.intercept_

    return val


# X = np.array([
#     [0, 0], 
#     [1, 1]])
    
# y = [0, 1]
# fit the model, don't regularize for illustration purposes
clf = svm.SVC(kernel="rbf")
clf.fit(X, y)

tmp = np.linspace(-10,10,100)
XX,YY = np.meshgrid(tmp,tmp)
X_contour = np.stack((XX.flatten(),YY.flatten()),axis=1)
Z = clf.decision_function(X_contour)
# Z = dec_func(clf,X_contour)

X_test = np.array([[7.65,0.83]])
# clf.dual_coef_[0,1]*kernel(clf.support_vectors_[1],X_test,clf._gamma)



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