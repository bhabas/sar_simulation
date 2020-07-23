import scipy.io
import numpy as np

# # test for scipy.io
# mat1 = scipy.io.loadmat('/home/pan/catkin_ws/src/robot landing/4. rl/src/DMP_upward.mat')
# mat2 = scipy.io.loadmat('/home/pan/catkin_ws/src/robot landing/4. rl/src/DMP_rotation.mat')

# tau = np.array( mat1['DMP_upward']['tau'])

# test for numpy.vstack
t = np.linspace(start=0, stop=1, num=20)
y = np.zeros(shape=[3,20])

c = np.vstack((t, y[0,:]))
