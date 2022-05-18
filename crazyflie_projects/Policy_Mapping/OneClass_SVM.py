import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm

XX,YY  = np.meshgrid(np.linspace(-5,5,500), np.linspace(-5,5,500))

## GENERATE TRAINING DATA
X = 0.3*np.random.randn(100,2)
X_train = np.r_[X+2,X-2]

## GENERATESOME REGULAR NOVEL OBSERVATIONS
X = 0.3 * np.random.randn(20,2)
X_test = np.r_[X+2,X-2]

## GENERATE SOME ABNORMAL NOVEL OBSERVATIONS
X_outliers = np.random.uniform(low=-4, high=4, size=(20,2))

## FIT THE MODEL
clf = svm.OneClassSVM(nu=0.1, kernel="rbf", gamma=0.1)
clf.fit(X_train)

y_pred_train = clf.predict(X_train)
y_pred_test = clf.predict(X_test)
y_pred_outliers = clf.predict(X_outliers)

n_error_train = y_pred_train[y_pred_train == -1].size
n_error_test = y_pred_test[y_pred_test == -1].size
n_error_outliers = y_pred_outliers[y_pred_outliers == -1].size

## PLOT THE LINE, THE POINTS, AND NEAREST VECTORS TO THE PLAN
Z = clf.decision_function(np.c_[XX.ravel(),YY.ravel()])
Z = Z.reshape(XX.shape)




## PLOT DATA
fig = plt.figure()
ax = fig.add_subplot(111)

ax.contourf(XX, YY, Z, levels=np.linspace(Z.min(), 0, 20), cmap=plt.cm.PuBu)
ax.contourf(XX, YY, Z, levels=[0, Z.max()], colors="palevioletred")
ax.contour(XX,YY,Z,levels=[0],linewidths=2,colors='darkred')

s=40
b1 = ax.scatter(X_train[:,0],X_train[:,1],c='white',edgecolors='k',s=s)
# b2 = ax.scatter(X_test[:,0],X_test[:,1],c='blueviolet',edgecolors='k',s=s)
# c = ax.scatter(X_outliers[:,0],X_outliers[:,1],c='gold',edgecolors='k',s=s)




ax.set_xlim(-5,5)
ax.set_ylim(-5,5)
ax.grid()


plt.show()


