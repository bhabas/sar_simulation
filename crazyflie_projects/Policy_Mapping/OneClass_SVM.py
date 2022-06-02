import numpy as np
from sklearn.svm import OneClassSVM
import matplotlib.pyplot as plt
import matplotlib.font_manager
from sklearn.datasets import load_wine

def kernel(x,xi,gamma):
    # return np.exp(-gamma*np.linalg.norm(x-xi)**2)
    # np.sum(np.power((actual_value-predicted_value),2))
    return np.exp(-gamma*np.sum((x-xi)**2))

def dec_func(clf,X_test):
    val = 0
    for ii in range(len(clf.support_vectors_)):
        val += clf.dual_coef_[0,ii]*kernel(clf.support_vectors_[ii],X_test,clf._gamma)
    val += clf.intercept_

    return val

# Get data
X_train = load_wine()["data"][:, [6, 9]]  # "banana"-shaped

# Learn a frontier for outlier detection with several classifiers
xx, yy = np.meshgrid(np.linspace(-1, 5.5, 500), np.linspace(-2.5, 19, 500))
clf_name = "OCSVM"
clf = OneClassSVM(nu=0.2, gamma=0.65)
clf.fit(X_train)
Z = clf.decision_function(np.c_[xx.ravel(), yy.ravel()])
Z = Z.reshape(xx.shape)

X_test = np.array([[7.65,0.83]])

fig = plt.figure()
ax = fig.add_subplot(111)
cs = ax.contour(xx, yy, Z, levels=[-1,0,1], linewidths=2, colors='k')
ax.clabel(cs, inline=True)

ax.scatter(X_train[:, 0], X_train[:, 1], color="tab:blue")
ax.set_xlim((xx.min(), xx.max()))
ax.set_ylim((yy.min(), yy.max()))

ax.set_ylabel("color_intensity")
ax.set_xlabel("flavanoids")

plt.show()