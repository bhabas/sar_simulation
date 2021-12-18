import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd


from sklearn import svm
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
from sklearn.datasets import make_blobs
import os

os.system("clear")


np.random.seed(0)

n_samples_1 = 1000
centers = [[0.0,0.0,0.0]]
clusters_std = [3.0]
X, y = make_blobs(
    n_features=3,
    n_samples=[n_samples_1],
    centers=centers,
    cluster_std=clusters_std,
    random_state=0,
    shuffle=False,
)

X1 = X[:,0]
X2 = X[:,1]
X3 = X[:,2]

r = np.sqrt(X1**2 + X2**2 + X3**2)
y = [1 if ii < 2.5 else 0 for ii in r]


data_array = np.stack((X1.flatten(),X2.flatten(),X3.flatten(),y,r)).T
df = pd.DataFrame(data_array,columns=['X1','X2','X3','y','r'])

## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
train, test = train_test_split(df,test_size=0.2,random_state=33)
test = test.sort_values(by='r')

## CONVERT DATA INTO TENSORS
X_train = train[['X1','X2','X3']].to_numpy()
X_test = test[['X1','X2','X3']].to_numpy()

y_train = train[['y']].to_numpy().flatten()
y_test = test[['y','r']].to_numpy()


## TRAIN THE MODEL
clf = svm.SVC(kernel='rbf',gamma="auto",class_weight={1: 20})
clf.fit(X_train,y_train)


## EVALUATE MODEL
y_pred = clf.predict(X_test)
# r_test = np.sqrt(X_test[:,0]**2 + X_test[:,1]**2 + X_test[:,2]**2)

from sklearn.metrics import accuracy_score, balanced_accuracy_score,confusion_matrix,classification_report,roc_curve,roc_auc_score


count = 0
for ii in range(len(y_pred)):

    if y_test[ii,0] == y_pred[ii]:
        count += 1
        correct = True
    else:
        correct = False

    print(f"y_test: {y_test[ii,0]:.1f} \t y_pred: {y_pred[ii]:.1f} \t radius: {y_test[ii,1]:.2f} \t Correct: {str(correct)}")

print(f"Correct: {count/len(y_pred):.2f}")
print(accuracy_score(y_test[:,0],y_pred))
print(balanced_accuracy_score(y_test[:,0],y_pred))
print(confusion_matrix(y_test[:,0],y_pred,normalize='all'))
print(classification_report(y_test[:,0],y_pred))

# fpr, tpr, thresholds = roc_curve(y_test[:,0],y_pred, pos_label=1)
# plt.plot(tpr,fpr)
# plt.show()

print(roc_auc_score(y_test[:,0],y_pred))



fig = plt.figure(1,figsize=(8,8))

## SUBPLOT 1 (FUNCTION)
ax1 = fig.add_subplot(111,projection='3d')
ax1.scatter(
    X[:,0].flatten(),
    X[:,1].flatten(),
    X[:,2].flatten(),
    c = y,
    cmap='jet',linewidth=0.2,antialiased=True)
ax1.set_xlabel('x1')
ax1.set_ylabel('x2')
ax1.set_ylabel('x3')
ax1.set_title('Function')
# ax1.set_xlim(-3,3)
# ax1.set_ylim(-3,3)
# ax1.set_zlim(-3,3)


plt.show()